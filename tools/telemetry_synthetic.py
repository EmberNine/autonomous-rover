#!/usr/bin/env python3
"""Synthetic runs for the telemetry page, without a bag or ROS (docs/telemetry.md): a real and a
sim half circle and a sim straight, shaped like the Week 6 runs, loaded with import_run.load_run.
Names start with synthetic_; each is keyed by a hash of its name, so running this again replaces
them. Delete them with: DELETE FROM runs WHERE name LIKE 'synthetic_%';
    python3 tools/telemetry_synthetic.py [--db "dbname=rover"]
"""
import argparse
import datetime
import hashlib
import math

from import_run import apply_schema, load_run

PERIOD_NS = 20_000_000                     # /odom at 50 Hz


def arc(radius, speed, seconds, heading_scale=1.0, rest=2.0):
    """Odom rows: rest, a left arc of the radius at the speed (radius None: straight), rest."""
    rows, x, y, heading = [], 0.0, 0.0, 0.0
    total = rest * 2 + seconds
    for k in range(int(total * 1e9 / PERIOD_NS) + 1):
        t = k * PERIOD_NS / 1e9
        moving = rest <= t < rest + seconds
        v = speed if moving else 0.0
        if moving:
            dt = PERIOD_NS / 1e9
            rate = 0.0 if radius is None else heading_scale * speed / radius
            x += v * math.cos(heading) * dt
            y += v * math.sin(heading) * dt
            heading += rate * dt
        rows.append((k * PERIOD_NS, x, y, heading, v))
    return rows


def battery(seconds, start_v=7.92, drop_v=0.04):
    """1 Hz, the first one latched from before the first /odom."""
    return [(-600_000_000 + k * 1_000_000_000, start_v - drop_v * k / seconds + 0.004 * math.sin(k), True)
            for k in range(int(seconds) + 1)]


def runs(now):
    half = 0.643 * math.pi / 0.05            # the model's half circle at 0.05 m/s: ~40 s
    yield (dict(name='synthetic_real_half_circle', route='half_circle', source='real',
                started_at=now - datetime.timedelta(hours=1), tape_chord_m=1.186,
                gyro_heading_rad=2.969),
           arc(0.643 * 1.05, 0.05, half, heading_scale=0.97), battery(half + 4))
    yield (dict(name='synthetic_sim_half_circle', route='half_circle', source='sim',
                started_at=now - datetime.timedelta(minutes=30)),
           arc(0.643, 0.05, half), [])
    yield (dict(name='synthetic_sim_straight', route='straight', source='sim',
                started_at=now - datetime.timedelta(minutes=20)),
           arc(None, 0.10, 20.0), [])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--db', default='dbname=rover', help='libpq connection string')
    args = parser.parse_args(argv)

    import psycopg

    now = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0)
    with psycopg.connect(args.db, autocommit=True) as connection:
        apply_schema(connection)
        for run, odom, battery_rows in runs(now):
            run = {'git_commit': None, 'tape_chord_m': None, 'gyro_heading_rad': None,
                   **run, 'recording_sha256': hashlib.sha256(run['name'].encode()).hexdigest()}
            run_id = load_run(connection, run, odom, battery_rows)
            print(f'run {run_id}: {run["name"]}, {len(odom)} odom, {len(battery_rows)} battery samples')


if __name__ == '__main__':
    main()
