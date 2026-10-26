#!/usr/bin/env python3
"""Import one recorded run (a ros2 bag from tools/record_run.sh) into Postgres (docs/telemetry.md).

Reads /odom and /rover/battery from the bag and run.json next to its data file, and loads one
run: a row in runs, its odom_samples and battery_samples (tools/telemetry_schema.sql, applied
first). Time is t_ns, integer nanoseconds since the first /odom header stamp; the bag's own
receive times aren't used, so the Pi's clock and sim time are handled alike.

The run is keyed by the sha256 of the bag's data file: importing the same recording again
replaces that run (delete it and its samples, insert, in one transaction), so it never
duplicates. Two messages of one stream with the same header stamp fail the import: the time
is the key, and dropping one silently would hide a stamping bug.

Needs ROS 2 (rosbag2_py) sourced and psycopg (apt python3-psycopg); the parsing and the
transforms below don't, and are tested without either (tools/test_import_run.py). In WSL:
    source /opt/ros/jazzy/setup.bash
    python3 tools/import_run.py ~/rover-runs/real_half_circle_20261027_101500 \
        [--tape-chord 1.186 --gyro-heading 2.969] [--db "dbname=rover"]
"""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path

ROUTES = ('straight', 'half_circle')
SOURCES = ('real', 'sim')
ODOM_TOPIC, BATTERY_TOPIC = '/odom', '/rover/battery'
DATA_SUFFIXES = ('.mcap', '.db3')
SCHEMA = Path(__file__).resolve().parent / 'telemetry_schema.sql'


class RecordingError(Exception):
    """A recording that can't be imported as it is (the message says why)."""


def stamp_ns(stamp):
    """A builtin_interfaces/Time as exact integer nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw(orientation):
    """Heading (rad) of a quaternion, about z."""
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def unwrap(angles):
    """Angles in (-pi, pi] made continuous: each step taken as the shortest turn."""
    result = []
    for angle in angles:
        if result:
            step = math.remainder(angle - result[-1], 2 * math.pi)
            angle = result[-1] + step
        result.append(angle)
    return result


def check_unique(topic, stamps):
    seen = set()
    for stamp in stamps:
        if stamp in seen:
            raise RecordingError(f'{topic}: two messages with header stamp {stamp} ns; '
                               'the time is the key, so the import stops (nothing was written)')
        seen.add(stamp)


def odom_rows(messages):
    """/odom messages -> (stamps, rows): rows of (t_ns, x, y, heading, speed), in time order,
    t_ns from the first stamp. The stamps are absolute ns, for the battery's origin."""
    if not messages:
        raise RecordingError(f'no {ODOM_TOPIC} in the recording')
    stamps = [stamp_ns(message.header.stamp) for message in messages]
    check_unique(ODOM_TOPIC, stamps)
    ordered = sorted(zip(stamps, messages), key=lambda pair: pair[0])
    origin = ordered[0][0]
    headings = unwrap([yaw(message.pose.pose.orientation) for _, message in ordered])
    rows = [(stamp - origin, message.pose.pose.position.x, message.pose.pose.position.y, heading,
             message.twist.twist.linear.x)
            for (stamp, message), heading in zip(ordered, headings)]
    return origin, rows


def battery_rows(messages, origin):
    """/rover/battery messages -> rows of (t_ns, voltage or None, present), in time order. NaN
    (no pack, or not measured) is None: NULL, not a number."""
    stamps = [stamp_ns(message.header.stamp) for message in messages]
    check_unique(BATTERY_TOPIC, stamps)
    return sorted((stamp - origin, None if math.isnan(message.voltage) else message.voltage,
                   bool(message.present))
                  for stamp, message in zip(stamps, messages))


def data_file(recording):
    """The bag's one data file (.mcap or .db3). A split bag isn't supported: say so."""
    files = sorted(path for path in Path(recording).iterdir() if path.suffix in DATA_SUFFIXES)
    if len(files) != 1:
        raise RecordingError(f'{recording}: expected one .mcap or .db3 file, found {len(files)}'
                           + (f' ({", ".join(path.name for path in files)})' if files else ''))
    return files[0]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as file:
        for block in iter(lambda: file.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def load_metadata(path):
    """run.json (tools/record_run.sh): name, route, source, started_at, git_commit."""
    try:
        metadata = json.loads(Path(path).read_text())
    except FileNotFoundError:
        raise RecordingError(f'{path} missing (tools/record_run.sh writes it after the bag)') from None
    missing = [key for key in ('name', 'route', 'source', 'started_at') if not metadata.get(key)]
    if missing:
        raise RecordingError(f'{path}: missing {", ".join(missing)}')
    if metadata['route'] not in ROUTES:
        raise RecordingError(f'{path}: route {metadata["route"]!r}, expected one of {ROUTES}')
    if metadata['source'] not in SOURCES:
        raise RecordingError(f'{path}: source {metadata["source"]!r}, expected one of {SOURCES}')
    started_at = datetime.datetime.fromisoformat(metadata['started_at'])
    if started_at.tzinfo is None:
        raise RecordingError(f'{path}: started_at {metadata["started_at"]!r} has no UTC offset')
    return {'name': metadata['name'], 'route': metadata['route'], 'source': metadata['source'],
            'started_at': started_at, 'git_commit': metadata.get('git_commit') or None}


def apply_schema(connection):
    with connection.cursor() as cursor:
        cursor.execute(SCHEMA.read_text())


def load_run(connection, run, odom, battery):
    """One run into the database, replacing any run from the same recording, in one transaction:
    on any error nothing changes. run: the metadata plus recording_sha256, tape_chord_m and
    gyro_heading_rad (a re-import takes the ones given this time). The connection must be in
    autocommit mode, so that transaction() is the transaction and not a savepoint in an open one.
    Returns the new run_id."""
    with connection.transaction(), connection.cursor() as cursor:
        cursor.execute('DELETE FROM runs WHERE recording_sha256 = %s', (run['recording_sha256'],))
        cursor.execute(
            'INSERT INTO runs (name, route, source, started_at, git_commit, recording_sha256, '
            'tape_chord_m, gyro_heading_rad) '
            'VALUES (%(name)s, %(route)s, %(source)s, %(started_at)s, %(git_commit)s, '
            '%(recording_sha256)s, %(tape_chord_m)s, %(gyro_heading_rad)s) RETURNING run_id',
            run)
        run_id = cursor.fetchone()[0]
        with cursor.copy('COPY odom_samples (run_id, t_ns, x_m, y_m, heading_rad, speed_mps) '
                         'FROM STDIN') as copy:
            for row in odom:
                copy.write_row((run_id, *row))
        with cursor.copy('COPY battery_samples (run_id, t_ns, voltage_v, present) FROM STDIN') as copy:
            for row in battery:
                copy.write_row((run_id, *row))
    return run_id


def read_bag(recording):
    """{topic: [message, ...]} for /odom and /rover/battery, in bag order. Needs rosbag2_py."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(recording), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()
             if topic.name in (ODOM_TOPIC, BATTERY_TOPIC)}
    messages = {ODOM_TOPIC: [], BATTERY_TOPIC: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in types:
            messages[topic].append(deserialize_message(data, types[topic]))
    return messages


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('recording', type=Path, help='the bag directory (with run.json)')
    parser.add_argument('--tape-chord', type=float,
                        help='taped straight-line distance, start mark to end mark (m)')
    parser.add_argument('--gyro-heading', type=float,
                        help='the turn from tools/gyro_turn_check.py (rad)')
    parser.add_argument('--db', default='dbname=rover', help='libpq connection string')
    args = parser.parse_args(argv)

    import psycopg

    try:
        run = load_metadata(args.recording / 'run.json')
        run.update(recording_sha256=sha256(data_file(args.recording)), tape_chord_m=args.tape_chord,
                   gyro_heading_rad=args.gyro_heading)
        messages = read_bag(args.recording)
        origin, odom = odom_rows(messages[ODOM_TOPIC])
        battery = battery_rows(messages[BATTERY_TOPIC], origin)
    except RecordingError as error:
        raise SystemExit(f'not imported: {error}') from None
    with psycopg.connect(args.db, autocommit=True) as connection:
        apply_schema(connection)
        run_id = load_run(connection, run, odom, battery)
    print(f'run {run_id}: {run["name"]} ({run["source"]} {run["route"]}), {len(odom)} odom, '
          f'{len(battery)} battery samples, {odom[-1][0] / 1e9:.1f} s')


if __name__ == '__main__':
    main()
