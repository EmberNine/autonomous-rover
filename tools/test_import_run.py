"""import_run: the parsing and the transforms, without ROS or Postgres (fake messages)."""
import hashlib
import json
import math
from types import SimpleNamespace as NS

import pytest

from import_run import (RecordingError, battery_rows, data_file, load_metadata, odom_rows, sha256,
                        stamp_ns, unwrap, yaw)

EPOCH = 1_761_436_800          # 2025-10-26, a Pi wall-clock stamp: a double of it in ns isn't exact


def stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def quaternion(heading):
    return NS(x=0.0, y=0.0, z=math.sin(heading / 2), w=math.cos(heading / 2))


def odom(ns, x=0.0, y=0.0, heading=0.0, speed=0.0):
    return NS(header=NS(stamp=stamp(ns)),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0), orientation=quaternion(heading))),
              twist=NS(twist=NS(linear=NS(x=speed, y=0.0, z=0.0))))


def battery(ns, voltage, present=True):
    return NS(header=NS(stamp=stamp(ns)), voltage=voltage, present=present)


def test_stamps_are_exact_integer_nanoseconds():
    ns = EPOCH * 1_000_000_000 + 123_456_789
    assert stamp_ns(stamp(ns)) == ns
    assert int(float(ns)) != ns            # why t_ns is a bigint: the double loses the last digits


@pytest.mark.parametrize('heading', [0.0, 0.5, math.pi / 2, -2.0, 3.1])
def test_yaw_of_a_heading_quaternion(heading):
    assert yaw(quaternion(heading)) == pytest.approx(heading)


def test_unwrap_carries_a_half_circle_past_pi():
    assert unwrap([3.0, 3.1, -3.1, -3.0]) == pytest.approx([3.0, 3.1, 2 * math.pi - 3.1, 2 * math.pi - 3.0])
    assert unwrap([0.1, -0.1, 0.2]) == pytest.approx([0.1, -0.1, 0.2])
    assert unwrap([]) == []


def test_odom_rows_are_relative_to_the_first_stamp_and_in_time_order():
    start = EPOCH * 1_000_000_000
    origin, rows = odom_rows([odom(start + 20_000_000, x=0.002, speed=0.1),
                              odom(start, heading=0.0),
                              odom(start + 40_000_000, x=0.004, y=0.001, heading=0.01, speed=0.1)])
    assert origin == start
    assert [row[0] for row in rows] == [0, 20_000_000, 40_000_000]
    t_ns, x, y, heading, speed = rows[2]
    assert (x, y, speed) == (0.004, 0.001, 0.1) and heading == pytest.approx(0.01)


def test_the_odom_heading_is_unwrapped():
    _, rows = odom_rows([odom(0, heading=3.1), odom(10, heading=-3.1)])
    assert rows[1][3] == pytest.approx(2 * math.pi - 3.1)


def test_two_odom_messages_with_one_stamp_fail_loudly():
    with pytest.raises(RecordingError, match=r'/odom: two messages with header stamp 20 ns'):
        odom_rows([odom(0), odom(20), odom(20, x=1.0)])


def test_no_odom_is_an_error():
    with pytest.raises(RecordingError, match='no /odom'):
        odom_rows([])


def test_battery_rows_share_the_odom_origin_and_may_start_before_it():
    rows = battery_rows([battery(1_500, 7.9), battery(500, 7.95)], origin=1_000)
    assert rows == [(-500, 7.95, True), (500, 7.9, True)]


def test_no_pack_is_null_not_a_number():
    assert battery_rows([battery(0, math.nan, present=False)], origin=0) == [(0, None, False)]


def test_two_battery_messages_with_one_stamp_fail_loudly():
    with pytest.raises(RecordingError, match='/rover/battery: two messages'):
        battery_rows([battery(5, 7.9), battery(5, 7.9)], origin=0)


def test_the_data_file_is_the_one_mcap_or_db3(tmp_path):
    (tmp_path / 'metadata.yaml').write_text('x')
    (tmp_path / 'run.json').write_text('{}')
    (tmp_path / 'run_0.mcap').write_bytes(b'mcap data')
    assert data_file(tmp_path).name == 'run_0.mcap'
    assert sha256(data_file(tmp_path)) == hashlib.sha256(b'mcap data').hexdigest()


@pytest.mark.parametrize('names, found', [([], 'found 0'), (['a_0.mcap', 'a_1.mcap'], 'found 2 \\(a_0.mcap, a_1.mcap\\)')])
def test_none_or_a_split_bag_is_refused(tmp_path, names, found):
    for name in names:
        (tmp_path / name).write_bytes(b'')
    with pytest.raises(RecordingError, match=found):
        data_file(tmp_path)


def write_metadata(tmp_path, **changes):
    metadata = {'name': 'real_half_circle_20261027_101500', 'route': 'half_circle', 'source': 'real',
                'started_at': '2026-10-27T10:15:00-05:00', 'git_commit': 'abc1234'}
    metadata.update(changes)
    path = tmp_path / 'run.json'
    path.write_text(json.dumps({key: value for key, value in metadata.items() if value is not None}))
    return path


def test_run_json_is_read_with_its_utc_offset(tmp_path):
    run = load_metadata(write_metadata(tmp_path))
    assert run['route'] == 'half_circle' and run['source'] == 'real' and run['git_commit'] == 'abc1234'
    assert run['started_at'].utcoffset().total_seconds() == -5 * 3600


@pytest.mark.parametrize('changes, message', [
    ({'route': 'figure_eight'}, "route 'figure_eight'"),
    ({'source': 'replay'}, "source 'replay'"),
    ({'started_at': None}, 'missing started_at'),
    ({'started_at': '2026-10-27T10:15:00'}, 'no UTC offset'),
])
def test_a_bad_run_json_is_refused(tmp_path, changes, message):
    with pytest.raises(RecordingError, match=message):
        load_metadata(write_metadata(tmp_path, **changes))


def test_a_missing_run_json_says_where_it_comes_from(tmp_path):
    with pytest.raises(RecordingError, match='record_run.sh writes it'):
        load_metadata(tmp_path / 'run.json')
