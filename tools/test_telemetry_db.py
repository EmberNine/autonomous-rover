"""import_run and telemetry_queries against a real Postgres: the schema, the replace-on-reimport,
the transaction and the page's queries.

Runs only when TELEMETRY_TEST_DB is set (a libpq connection string; CI's telemetry-db job sets it,
against a postgres:16 service). Then psycopg must import: a missing one is a failure, not a skip.
Everything happens in a schema of its own, dropped at the end, so a real database is safe:
    TELEMETRY_TEST_DB="dbname=rover" python3 -m pytest -q tools/test_telemetry_db.py
"""
import datetime
import math
import os
import uuid

import pytest

if not os.environ.get('TELEMETRY_TEST_DB'):
    pytest.skip('TELEMETRY_TEST_DB not set (a Postgres to test against)', allow_module_level=True)

import psycopg  # noqa: E402

import telemetry_queries as queries  # noqa: E402
from import_run import apply_schema, load_run  # noqa: E402

START = datetime.datetime(2026, 10, 27, 10, 15, tzinfo=datetime.timezone(datetime.timedelta(hours=-5)))


@pytest.fixture
def connection():
    schema = f'telemetry_test_{uuid.uuid4().hex[:12]}'
    with psycopg.connect(os.environ['TELEMETRY_TEST_DB'], autocommit=True) as connection:
        connection.execute(f'CREATE SCHEMA {schema}')
        connection.execute(f'SET search_path TO {schema}')
        try:
            apply_schema(connection)
            yield connection
        finally:
            connection.execute(f'DROP SCHEMA {schema} CASCADE')


def run(sha='a', source='real', route='half_circle', minutes=0, **changes):
    values = {'name': f'{source}_{route}_{sha}', 'route': route, 'source': source,
              'started_at': START + datetime.timedelta(minutes=minutes), 'git_commit': '416b48d',
              'recording_sha256': sha * 64, 'tape_chord_m': None, 'gyro_heading_rad': None}
    values.update(changes)
    return values


# Three sides of a 1 m square, 20 ms apart: path length 3 m, ending at (0, 1) facing -x.
SQUARE = [(0, 0.0, 0.0, 0.0, 0.1), (20_000_000, 0.5, 0.0, 0.0, 0.1), (40_000_000, 1.0, 0.0, 0.0, 0.1),
          (60_000_000, 1.0, 1.0, math.pi / 2, 0.1), (80_000_000, 0.0, 1.0, math.pi, 0.0)]
BATTERY = [(-400_000_000, 7.95, True), (600_000_000, 7.9, True), (1_600_000_000, None, False)]


def count(connection, table):
    return connection.execute(f'SELECT count(*) FROM {table}').fetchone()[0]


def test_a_run_and_its_samples_are_loaded(connection):
    run_id = load_run(connection, run(tape_chord_m=1.186), SQUARE, BATTERY)
    (stored,) = queries.runs(connection)
    assert stored['run_id'] == run_id and stored['tape_chord_m'] == 1.186 and stored['gyro_heading_rad'] is None
    assert stored['started_at'] == START
    assert (count(connection, 'odom_samples'), count(connection, 'battery_samples')) == (5, 3)


def test_importing_the_same_recording_again_replaces_it(connection):
    load_run(connection, run(), SQUARE, BATTERY)
    run_id = load_run(connection, run(tape_chord_m=1.19, gyro_heading_rad=2.969), SQUARE[:3], BATTERY)
    (stored,) = queries.runs(connection)
    assert stored['run_id'] == run_id and (stored['tape_chord_m'], stored['gyro_heading_rad']) == (1.19, 2.969)
    assert (count(connection, 'runs'), count(connection, 'odom_samples')) == (1, 3)


def test_a_negative_tape_chord_is_refused(connection):
    with pytest.raises(psycopg.errors.CheckViolation):
        load_run(connection, run(tape_chord_m=-0.1), SQUARE, BATTERY)
    assert count(connection, 'runs') == 0


def test_a_failed_import_changes_nothing(connection):
    load_run(connection, run(), SQUARE, BATTERY)
    with pytest.raises(psycopg.errors.UniqueViolation):
        load_run(connection, run(), SQUARE + [SQUARE[-1]], BATTERY)     # a duplicate t_ns
    assert (count(connection, 'runs'), count(connection, 'odom_samples')) == (1, 5)


def test_the_page_finds_the_latest_sim_run_of_the_route(connection):
    load_run(connection, run('a', 'real'), SQUARE, BATTERY)
    older = load_run(connection, run('b', 'sim', minutes=-10), SQUARE, [])
    latest = load_run(connection, run('c', 'sim', minutes=-5), SQUARE, [])
    load_run(connection, run('d', 'sim', route='straight', minutes=1), SQUARE, [])
    assert [row['run_id'] for row in queries.sim_runs(connection, 'half_circle')] == [latest, older]
    assert [row['source'] for row in queries.runs(connection)] == ['sim', 'real', 'sim', 'sim']


def test_odom_and_battery_come_back_in_seconds(connection):
    real = load_run(connection, run('a'), SQUARE, BATTERY)
    sim = load_run(connection, run('b', 'sim'), SQUARE[:2], [])
    rows = queries.odom(connection, [real, sim])
    assert [(row['run_id'], row['t_s']) for row in rows] == \
        [(real, 0.0), (real, 0.02), (real, 0.04), (real, 0.06), (real, 0.08), (sim, 0.0), (sim, 0.02)]
    assert isinstance(rows[1]['t_s'], float)
    assert [(row['t_s'], row['voltage_v'], row['present']) for row in queries.battery(connection, real)] == \
        [(-0.4, pytest.approx(7.95), True), (0.6, pytest.approx(7.9), True), (1.6, None, False)]


def test_the_summary_is_the_last_sample_and_the_path_length(connection):
    run_id = load_run(connection, run(), SQUARE, BATTERY)
    summary = queries.summary(connection, run_id)
    assert summary['distance_m'] == pytest.approx(3.0)
    assert (summary['end_x_m'], summary['end_y_m']) == (0.0, 1.0) and summary['chord_m'] == 1.0
    assert summary['end_heading_rad'] == pytest.approx(math.pi)
    assert summary['duration_s'] == pytest.approx(0.08) and summary['samples'] == 5
    assert queries.summary(connection, run_id + 1) is None
