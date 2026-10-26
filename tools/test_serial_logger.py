"""Tests for serial_logger.py. Run from the repo root: python3 -m pytest tools/

The main test proves the TelemetryLogger refactor changed nothing: it runs the
original serial_logger.py (tools/test_data/serial_logger_pre_refactor.py, kept as it was) and the
current one on the same recorded byte stream, with the same fake serial port
and the same fake clock, and requires the .log, the .log.times.csv, everything
printed (status lines, BAD lines, final summary) and the exit code to be
byte-identical. The stream holds only version-1 lines (no motor fields), the
format the original logger knows; on those the two must agree exactly.
"""
import io
import random
import re
import sys
import types
from pathlib import Path

import pytest

import serial_logger

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PRE_REFACTOR_LOGGER = REPOSITORY_ROOT / 'tools' / 'test_data' / 'serial_logger_pre_refactor.py'  # unrefactored


# --- Test doubles -------------------------------------------------------------

class FakeClock:
    """Stands in for the time module. Every call moves the clock on by a fixed
    step, so two runs that make the same calls in the same order get exactly
    the same times, and the status line comes out every ~40 reads."""

    def __init__(self):
        self.wall_seconds = 1_760_000_000.0
        self.monotonic_seconds = 5_000.0

    def time(self):
        self.wall_seconds += 0.0137
        return self.wall_seconds

    def monotonic(self):
        self.monotonic_seconds += 0.05
        return self.monotonic_seconds


class FakePort:
    """Returns the recorded chunks one per read(), then raises KeyboardInterrupt
    (what Ctrl+C does), so the logger stops exactly as it does for a user.
    Each read() takes time on the shared clock, like a real read waiting for
    bytes, so a host timestamp taken before the read instead of after it
    shows up as a different .times.csv."""

    READ_SECONDS = 0.021

    def __init__(self, chunks, clock):
        self.chunks = list(chunks)
        self.clock = clock
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        self.clock.wall_seconds += self.READ_SECONDS
        self.clock.monotonic_seconds += self.READ_SECONDS
        if not self.chunks:
            raise KeyboardInterrupt
        return self.chunks.pop(0)

    def close(self):
        self.closed = True


# --- Recorded stream ----------------------------------------------------------

def telemetry_line_v1(t_ms, rng):
    values = {name: rng.randint(-40000, 40000) for name in serial_logger.TELEMETRY_FIELDS_V1}
    values['t_ms'] = t_ms
    return (' '.join(f'{name}={values[name]}' for name in serial_logger.TELEMETRY_FIELDS_V1) + '\r\n').encode()


def motor_part(rng):
    return ' '.join(f'{name}={rng.randint(-100, 5000)}' for name in serial_logger.MOTOR_FIELDS).encode()


def recorded_stream(seed=20261005):
    """A v1 telemetry stream with every kind of trouble the logger handles."""
    rng = random.Random(seed)
    lines = []
    lines.append(telemetry_line_v1(4200, rng)[37:])          # port opened mid-line
    lines.append(b'imu_init=ok who=0x47\r\n')
    t_ms = 100
    for index in range(400):
        t_ms += 100
        if index in (50, 51, 260):
            t_ms += 300                                       # lost lines -> t_ms gaps
        line = telemetry_line_v1(t_ms, rng)
        if index in (20, 90, 150, 151, 152, 153, 154, 310):
            line = line[:rng.randint(10, len(line) - 5)] + b'\r\n'   # cut short
        elif index == 120:
            line = line[:-2] + line                           # two lines glued (lost '\r\n')
        elif index == 170:
            line = line.replace(b'gz=', b'gz=\xff')           # corrupted byte
        elif index == 200:
            line = line[:-2] + b'\n'                          # '\r' lost
        elif index == 230:
            line = b'imu_init=err_who_am_i who=0x00\r\n'      # board reset
            t_ms = 0
        lines.append(line)
    lines.append(telemetry_line_v1(t_ms + 100, rng)[:60])     # logging stopped mid-line
    return b''.join(lines)


def split_into_chunks(stream, seed=7):
    """Cuts the stream at random places, including inside '\\r\\n', with some
    empty reads (read timeouts) in between, like a real serial port."""
    rng = random.Random(seed)
    chunks, position = [], 0
    while position < len(stream):
        if rng.random() < 0.1:
            chunks.append(b'')
            continue
        size = rng.randint(1, 300)
        chunks.append(stream[position:position + size])
        position += size
    return chunks


# --- Running a logger ---------------------------------------------------------

def load_pre_refactor_logger(tmp_path):
    module_path = tmp_path / 'serial_logger_pre_refactor.py'
    module_path.write_bytes(PRE_REFACTOR_LOGGER.read_bytes())
    module_name = 'serial_logger_pre_refactor'
    import importlib.util
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_logger_main(module, chunks, output_path, monkeypatch, capsys):
    """Runs module.main() as 'serial_logger.py COM5 <output_path>' on the fake port."""
    clock = FakeClock()
    port = FakePort(chunks, clock)
    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(Serial=lambda *args, **kwargs: port))
    monkeypatch.setattr(module, 'time', clock)
    monkeypatch.setattr(sys, 'argv', ['serial_logger.py', 'COM5', str(output_path)])
    capsys.readouterr()  # discard anything printed before
    exit_code = module.main()
    printed = capsys.readouterr().out
    assert port.closed
    return {
        'exit_code': exit_code,
        'printed': printed,
        'log': Path(output_path).read_bytes(),
        'times_csv': Path(str(output_path) + '.times.csv').read_bytes(),
    }


# --- Tests --------------------------------------------------------------------

def test_refactored_logger_is_byte_identical_to_the_original(tmp_path, monkeypatch, capsys):
    original = load_pre_refactor_logger(tmp_path)
    stream = recorded_stream()
    chunks = split_into_chunks(stream)

    before = run_logger_main(original, chunks, tmp_path / 'original.log', monkeypatch, capsys)
    after = run_logger_main(serial_logger, chunks, tmp_path / 'refactored.log', monkeypatch, capsys)

    assert before['log'] == stream                       # sanity: the raw file is the stream, unchanged
    assert after['log'] == before['log']
    assert after['times_csv'] == before['times_csv']
    # The only change to the output since then: the read-gap field (2026-10-16).
    assert without_read_gap(after['printed']) == before['printed']
    assert after['printed'].count(' read_gap_max=') == after['printed'].count('\rgood=') + 1   # every status + summary
    assert after['exit_code'] == before['exit_code']

    # Sanity: the stream really exercised the interesting paths.
    printed = before['printed']
    final_line = printed.rstrip('\n').split('\n')[-1]
    assert final_line.startswith('Stopped. ')
    assert 'boot=2' in final_line
    assert 'bad=0' not in final_line
    assert 't_ms_gaps=0' not in final_line
    assert printed.count('\rgood=') >= 5                  # several status lines
    assert printed.count('BAD line') == serial_logger.MAX_BAD_LINES_SHOWN
    assert before['exit_code'] == 1


def test_identical_for_other_seeds_and_chunkings(tmp_path, monkeypatch, capsys):
    original = load_pre_refactor_logger(tmp_path)
    for seed in (1, 2, 3):
        stream = recorded_stream(seed)
        for chunk_seed in (11, 12):
            chunks = split_into_chunks(stream, chunk_seed)
            before = run_logger_main(original, chunks, tmp_path / f'o{seed}{chunk_seed}.log', monkeypatch, capsys)
            after = run_logger_main(serial_logger, chunks, tmp_path / f'r{seed}{chunk_seed}.log', monkeypatch, capsys)
            after['printed'] = without_read_gap(after['printed'])
            assert after == before


def without_read_gap(printed):
    """The output with the read-gap field (added after the original logger) taken out."""
    return re.sub(r' read_gap_max=\d+ms', '', printed)


def test_lines_with_motor_fields_are_good():
    rng = random.Random(3)
    raw_file, times_file = io.BytesIO(), io.StringIO()
    logger = serial_logger.TelemetryLogger(raw_file, times_file, print_function=lambda *args: None)
    stream = b''.join([
        b'imu_init=ok who=0x47\r\n',
        telemetry_line_v1(100, rng)[:-2] + b' ' + motor_part(rng) + b'\r\n',
        telemetry_line_v1(200, rng),                                      # v1 still good
        telemetry_line_v1(300, rng)[:-2] + b' ' + motor_part(rng) + b'\r\n',
    ])
    results = logger.feed(stream, 1.0)
    assert results == ['boot', 'good', 'good', 'good']
    assert logger.gaps == 0
    assert raw_file.getvalue() == stream


def test_motor_fields_are_all_or_nothing():
    rng = random.Random(4)
    v1_without_ending = telemetry_line_v1(100, rng)[:-2]
    full_motor_part = motor_part(rng)
    assert serial_logger.classify(v1_without_ending + b' ' + full_motor_part + b'\r\n') == 'good'
    # Cut inside the motor part, at a field boundary and inside a value.
    cut_at_field = full_motor_part.rsplit(b' ', 1)[0]
    assert serial_logger.classify(v1_without_ending + b' ' + cut_at_field + b'\r\n') == 'bad'
    assert serial_logger.classify(v1_without_ending + b' ' + full_motor_part[:12] + b'\r\n') == 'bad'
    # A trailing space, an extra field, or motor fields twice.
    assert serial_logger.classify(v1_without_ending + b' \r\n') == 'bad'
    assert serial_logger.classify(v1_without_ending + b' ' + full_motor_part + b' extra=1\r\n') == 'bad'
    assert serial_logger.classify(v1_without_ending + b' ' + full_motor_part + b' ' + full_motor_part + b'\r\n') == 'bad'


def steering_part(rng):
    return ' '.join(f'{name}={rng.randint(-100, 5000)}' for name in serial_logger.STEERING_FIELDS).encode()


def test_v3_lines_with_steering_fields_are_good():
    rng = random.Random(5)
    raw_file, times_file = io.BytesIO(), io.StringIO()
    logger = serial_logger.TelemetryLogger(raw_file, times_file, print_function=lambda *args: None)
    last_v3_line = telemetry_line_v1(400, rng)[:-2] + b' ' + motor_part(rng) + b' ' + steering_part(rng) + b'\r\n'
    stream = b''.join([
        b'imu_init=ok who=0x47\r\n',
        telemetry_line_v1(100, rng)[:-2] + b' ' + motor_part(rng) + b' ' + steering_part(rng) + b'\r\n',
        telemetry_line_v1(200, rng)[:-2] + b' ' + motor_part(rng) + b'\r\n',                   # v2 still good
        telemetry_line_v1(300, rng),                                                             # v1 still good
        last_v3_line,
    ])
    results = logger.feed(stream, 1.0)
    assert results == ['boot', 'good', 'good', 'good', 'good']
    assert logger.gaps == 0
    assert logger.last_good_line == last_v3_line
    assert raw_file.getvalue() == stream


def test_v3_field_list_is_v2_plus_steering():
    assert serial_logger.TELEMETRY_FIELDS_V3 == serial_logger.TELEMETRY_FIELDS_V2 + serial_logger.STEERING_FIELDS
    assert serial_logger.STEERING_FIELDS == ('rst', 's_pct', 's_us', 'sstate', 's_ok', 's_wd')


def test_steering_fields_are_all_or_nothing_and_only_after_the_motor_part():
    rng = random.Random(6)
    v1_without_ending = telemetry_line_v1(100, rng)[:-2]
    motor = motor_part(rng)
    steering = steering_part(rng)
    v2_without_ending = v1_without_ending + b' ' + motor
    assert serial_logger.classify(v2_without_ending + b' ' + steering + b'\r\n') == 'good'
    # Cut inside the steering part, at a field boundary and inside a value.
    assert serial_logger.classify(v2_without_ending + b' ' + steering.rsplit(b' ', 1)[0] + b'\r\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + steering[:9] + b'\r\n') == 'bad'
    # Steering without the motor part, steering before it, twice, or with an extra field.
    assert serial_logger.classify(v1_without_ending + b' ' + steering + b'\r\n') == 'bad'
    assert serial_logger.classify(v1_without_ending + b' ' + steering + b' ' + motor + b'\r\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + steering + b' ' + steering + b'\r\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + steering + b' extra=1\r\n') == 'bad'
    # Two fields swapped, a missing '\r', a trailing space.
    swapped = steering.replace(b'rst=', b'TMP=').replace(b's_pct=', b'rst=').replace(b'TMP=', b's_pct=')
    assert serial_logger.classify(v2_without_ending + b' ' + swapped + b'\r\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + steering + b'\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + steering + b' \r\n') == 'bad'


def battery_part(rng):
    return ' '.join(f'{name}={rng.randint(0, 9000)}' for name in serial_logger.BATTERY_FIELDS).encode()


def test_v4_lines_with_battery_fields_are_good_and_older_versions_still_are():
    rng = random.Random(7)
    raw_file, times_file = io.BytesIO(), io.StringIO()
    logger = serial_logger.TelemetryLogger(raw_file, times_file, print_function=lambda *args: None)
    v3 = lambda t: telemetry_line_v1(t, rng)[:-2] + b' ' + motor_part(rng) + b' ' + steering_part(rng)
    last_v4_line = v3(500) + b' ' + battery_part(rng) + b'\r\n'
    stream = b''.join([
        b'imu_init=ok who=0x47\r\n',
        v3(100) + b' ' + battery_part(rng) + b'\r\n',
        v3(200) + b'\r\n',                                                   # v3 still good
        telemetry_line_v1(300, rng)[:-2] + b' ' + motor_part(rng) + b'\r\n',  # v2 still good
        telemetry_line_v1(400, rng),                                          # v1 still good
        last_v4_line,
    ])
    assert logger.feed(stream, 1.0) == ['boot', 'good', 'good', 'good', 'good', 'good']
    assert logger.gaps == 0
    assert logger.last_good_line == last_v4_line


def test_v4_field_list_is_v3_plus_battery():
    assert serial_logger.TELEMETRY_FIELDS_V4 == serial_logger.TELEMETRY_FIELDS_V3 + serial_logger.BATTERY_FIELDS
    assert serial_logger.BATTERY_FIELDS == ('vbat', 'bstate', 'vdda')


def test_battery_fields_are_all_or_nothing_and_only_after_the_steering_part():
    rng = random.Random(8)
    v2_without_ending = telemetry_line_v1(100, rng)[:-2] + b' ' + motor_part(rng)
    steering = steering_part(rng)
    battery = battery_part(rng)
    v3_without_ending = v2_without_ending + b' ' + steering
    assert serial_logger.classify(v3_without_ending + b' ' + battery + b'\r\n') == 'good'
    # Cut inside the battery part, at a field boundary and inside a value.
    assert serial_logger.classify(v3_without_ending + b' ' + battery.rsplit(b' ', 1)[0] + b'\r\n') == 'bad'
    assert serial_logger.classify(v3_without_ending + b' ' + battery[:7] + b'\r\n') == 'bad'
    # Battery without the steering part, before it, twice, or with an extra field.
    assert serial_logger.classify(v2_without_ending + b' ' + battery + b'\r\n') == 'bad'
    assert serial_logger.classify(v2_without_ending + b' ' + battery + b' ' + steering + b'\r\n') == 'bad'
    assert serial_logger.classify(v3_without_ending + b' ' + battery + b' ' + battery + b'\r\n') == 'bad'
    assert serial_logger.classify(v3_without_ending + b' ' + battery + b' extra=1\r\n') == 'bad'
    # Two fields swapped.
    swapped = battery.replace(b'vbat=', b'TMP=').replace(b'vdda=', b'vbat=').replace(b'TMP=', b'vdda=')
    assert serial_logger.classify(v3_without_ending + b' ' + swapped + b'\r\n') == 'bad'


def speed_part(rng):
    return ' '.join(f'{name}={rng.randint(-480, 480)}' for name in serial_logger.SPEED_FIELDS).encode()


def test_v5_lines_with_speed_fields_are_good_and_older_versions_still_are():
    rng = random.Random(9)
    raw_file, times_file = io.BytesIO(), io.StringIO()
    logger = serial_logger.TelemetryLogger(raw_file, times_file, print_function=lambda *args: None)
    v3 = lambda t: telemetry_line_v1(t, rng)[:-2] + b' ' + motor_part(rng) + b' ' + steering_part(rng)
    v4 = lambda t: v3(t) + b' ' + battery_part(rng)
    last_v5_line = v4(600) + b' ' + speed_part(rng) + b'\r\n'
    stream = b''.join([
        b'imu_init=ok who=0x47\r\n',
        v4(100) + b' ' + speed_part(rng) + b'\r\n',
        v4(200) + b'\r\n',                                                    # v4 still good
        v3(300) + b'\r\n',                                                    # v3 still good
        telemetry_line_v1(400, rng)[:-2] + b' ' + motor_part(rng) + b'\r\n',   # v2 still good
        telemetry_line_v1(500, rng),                                           # v1 still good
        last_v5_line,
    ])
    assert logger.feed(stream, 1.0) == ['boot', 'good', 'good', 'good', 'good', 'good', 'good']
    assert logger.gaps == 0
    assert logger.last_good_line == last_v5_line


def test_v5_field_list_is_v4_plus_speed():
    assert serial_logger.TELEMETRY_FIELDS_V5 == serial_logger.TELEMETRY_FIELDS_V4 + serial_logger.SPEED_FIELDS
    assert serial_logger.SPEED_FIELDS == ('vset', 'vest', 'ctl', 'vok', 'vi', 'stall_n', 'enc_bad', 'mfault')


def test_speed_fields_are_all_or_nothing_and_only_after_the_battery_part():
    rng = random.Random(10)
    v3_without_ending = telemetry_line_v1(100, rng)[:-2] + b' ' + motor_part(rng) + b' ' + steering_part(rng)
    battery = battery_part(rng)
    speed = speed_part(rng)
    v4_without_ending = v3_without_ending + b' ' + battery
    assert serial_logger.classify(v4_without_ending + b' ' + speed + b'\r\n') == 'good'
    # Cut inside the speed part, at a field boundary and inside a value.
    assert serial_logger.classify(v4_without_ending + b' ' + speed.rsplit(b' ', 1)[0] + b'\r\n') == 'bad'
    assert serial_logger.classify(v4_without_ending + b' ' + speed[:7] + b'\r\n') == 'bad'
    # Speed without the battery part, before it, twice, or with an extra field.
    assert serial_logger.classify(v3_without_ending + b' ' + speed + b'\r\n') == 'bad'
    assert serial_logger.classify(v3_without_ending + b' ' + speed + b' ' + battery + b'\r\n') == 'bad'
    assert serial_logger.classify(v4_without_ending + b' ' + speed + b' ' + speed + b'\r\n') == 'bad'
    assert serial_logger.classify(v4_without_ending + b' ' + speed + b' extra=1\r\n') == 'bad'
    # Two fields swapped.
    swapped = speed.replace(b'vset=', b'TMP=').replace(b'vest=', b'vset=').replace(b'TMP=', b'vest=')
    assert serial_logger.classify(v4_without_ending + b' ' + swapped + b'\r\n') == 'bad'


def test_status_and_summary_text():
    raw_file, times_file = io.BytesIO(), io.StringIO()
    logger = serial_logger.TelemetryLogger(raw_file, times_file, print_function=lambda *args: None)
    assert logger.status_text() == 'good=0 bad=0 (0.00%) boot=0 t_ms_gaps=0'
    assert logger.summary_text() == 'Stopped. 0 lines: good=0 bad=0 boot=0 t_ms_gaps=0'
    assert times_file.getvalue() == 'host_time_unix,line,byte_offset,result\n'


# --- Binary mode (Pi link) ------------------------------------------------------------

import rover_link  # noqa: E402  (binary-mode tests only)


def v5_text_line(t_ms):
    return b' '.join(f'{key}={t_ms if key == "t_ms" else 0}'.encode() for key in serial_logger.TELEMETRY_FIELDS_V5) + b'\r\n'


BOOT = b'imu_init=OK who=0x47\r\n'


def state_fields(t_ms, **overrides):
    fields = {name: 0 for name in rover_link.STATE_FIELDS}
    fields.update(t_ms=t_ms, motor_state=2, enc_flags=rover_link.ENC_VALID_NOW, imu_flags=rover_link.IMU_OK)
    fields.update(overrides)
    return fields


def status_fields(**overrides):
    fields = {name: 0 for name in rover_link.STATUS_FIELDS}
    fields.update(motor_state=2, latch_reason=7, battery_state=1, vbat_mv=7921, imu_ok=1)
    fields.update(overrides)
    return fields


def hello_ack_frame(seq=0, build_flags=0):
    return rover_link.encode_frame(rover_link.HELLO_ACK, seq, rover_link.pack_hello_ack(
        {'build_flags': build_flags, 'reset_flags': 4, 'imu_ok': True, 'imu_who': 0x47, 'state_period_ms': 10,
         'status_period_ms': 100}))


def state_frame(seq, t_ms, **overrides):
    return rover_link.encode_frame(rover_link.STATE, seq & 0xFF, rover_link.pack_state(state_fields(t_ms, **overrides)))


def status_frame(seq, **overrides):
    return rover_link.encode_frame(rover_link.STATUS, seq & 0xFF, rover_link.pack_status(status_fields(**overrides)))


def sent_frames(writes):
    """(type, seq, payload) of every frame in the bytes written."""
    decoder = rover_link.FrameDecoder()
    decoder.feed(b'\x00')
    frames = []
    for data in writes:
        for status, frame, _ in decoder.feed(data):
            assert status == rover_link.OK, status
            frames.append((frame.type, frame.seq, frame.payload))
    return frames


def test_sequence_wraps_after_255():
    sequence = serial_logger.Sequence()
    assert [sequence.next() for _ in range(258)][-3:] == [255, 0, 1]


def test_hello_sender_retries_until_acknowledged_and_again_after_a_boot():
    hello = serial_logger.HelloSender(serial_logger.Sequence())
    assert len(hello.due(10.0)) == 1                      # at once
    assert hello.due(10.4) == []
    assert len(hello.due(10.5)) == 1                      # every HELLO_RETRY_SECONDS
    hello.acknowledged()
    assert hello.due(20.0) == []
    hello.board_in_text_mode(20.0, serial_logger.HELLO_AFTER_BOOT_SECONDS)
    assert hello.due(20.05) == []
    frames = hello.due(20.1)
    assert sent_frames(frames)[0][:2] == (rover_link.HELLO, 2)   # the shared seq goes on
    assert hello.sent_count == 3


def test_binary_logger_counts_frames_gaps_and_text_and_writes_rows():
    raw, times = io.BytesIO(), io.StringIO()
    logger = serial_logger.BinaryLogger(raw, times, print_function=lambda text: None)
    stream = v5_text_line(100) + v5_text_line(200)        # a board still in text mode
    stream += hello_ack_frame()
    stream += b''.join(state_frame(n, 1000 + 10 * n) for n in range(5))
    stream += status_frame(0)
    stream += state_frame(5, 1050)[:-4] + b'\x00'         # cut short: dropped
    stream += state_frame(7, 1070)                        # seq 6 lost: one gap, and a t_ms step of 20
    corrupted = bytearray(state_frame(8, 1080))
    corrupted[5] ^= 0x01
    stream += bytes(corrupted)                            # CRC error
    stream += BOOT + v5_text_line(100)                    # the board reset: text again
    stream += hello_ack_frame(seq=0) + state_frame(0, 50) + state_frame(1, 60)
    events = []
    for start in range(0, len(stream), 7):               # any chunking
        events += logger.feed(stream[start:start + 7], 123.0)
    assert raw.getvalue() == stream
    kinds = [event[0] if event[0] != 'frame' else event[1] for event in events]
    assert kinds.count('text') == 3 and kinds.count('boot') == 1 and kinds.count('HELLO_ACK') == 2
    assert kinds.count('STATE') == 8 and kinds.count('STATUS') == 1
    assert logger.counts['bad'] == 2 and logger.bad_counts == {'CRC_ERROR': 2}   # cut short: still COBS, wrong CRC
    assert logger.seq_gaps == {'STATE': 1}                # 4 -> 7; after the boot it starts over
    assert logger.t_ms_gaps == 1                          # 1040 -> 1070 (the reset restarts it too)
    assert logger.hello_ack['imu_who'] == 0x47
    assert logger.last_state['t_ms'] == 60 and logger.last_status['latch_reason'] == 7
    rows = times.getvalue().splitlines()
    assert rows[0] == 'host_time_unix,piece,result,type,seq'
    assert sum(',TEXT,' in row for row in rows) == 1      # boot + v5 line between two frames: text, not bad
    assert sum(',CRC_ERROR,' in row for row in rows) == 2 and sum(',OK,STATE,' in row for row in rows) == 8
    assert 'seq gaps: STATE=1' in logger.summary_text() and logger.exit_code() == 1


class BinaryFakePort:
    """Received (time, bytes) come out of read() when due; each read takes 10 ms of fake time."""

    def __init__(self, clock, received, stop_at):
        self.clock = clock
        self.received = sorted(received)
        self.stop_at = stop_at
        self.writes = []
        self.closed = False

    in_waiting = 0

    def read(self, size):
        self.clock.now += 0.01
        if self.clock.now >= self.stop_at:
            raise KeyboardInterrupt
        due = b''
        while self.received and self.received[0][0] <= self.clock.now:
            due += self.received.pop(0)[1]
        return due

    def write(self, data):
        self.writes.append((self.clock.now, data))

    def close(self):
        self.closed = True


class BinaryClock:
    def __init__(self):
        self.now = 50.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1_760_000_000.0 + self.now


def test_run_binary_logger_sends_only_hello_and_again_after_a_reset():
    clock = BinaryClock()
    received = [(51.2, hello_ack_frame()), (52.0, BOOT), (52.05, v5_text_line(100)), (53.0, hello_ack_frame())]
    port = BinaryFakePort(clock, received, stop_at=55.0)
    logger = serial_logger.BinaryLogger(io.BytesIO(), io.StringIO(), print_function=lambda text: None)
    shown = []
    serial_logger.run_binary_logger(port, clock, logger, serial_logger.HelloSender(serial_logger.Sequence()),
                                    shown.append)
    frames = sent_frames(data for _, data in port.writes)
    assert {frame_type for frame_type, _, _ in frames} == {rover_link.HELLO}    # never anything that could arm
    times = [when for when, _ in port.writes]
    # At once, every 0.5 s until the HELLO_ACK at 51.2 (50.0, 50.5, 51.0), none until the boot line,
    # then 0.1 s after it, every 0.5 s until the next HELLO_ACK at 53.0 (52.11, 52.61), then none.
    assert len([t for t in times if t < 51.2]) == 3 and not [t for t in times if 51.2 <= t < 52.1]
    assert 52.1 <= times[3] < 52.2 and len(times) == 5 and port.closed
    assert any(text.startswith('HELLO_ACK: protocol 1, normal build') for text in shown)
    assert any(text.startswith('board reset') for text in shown)


def test_frame_events_carry_the_piece_number_written_to_times_csv():
    times = io.StringIO()
    logger = serial_logger.BinaryLogger(None, times, print_function=lambda text: None)
    stream = hello_ack_frame() + b''.join(state_frame(n, 10 * n) for n in range(4))
    stream += state_frame(4, 40)[:-5] + b'\x00' + state_frame(5, 50)     # a bad piece between them
    events = []
    for start in range(0, len(stream), 5):
        events += logger.feed(stream[start:start + 5], 0.0)
    rows = [row.split(',') for row in times.getvalue().splitlines()[1:]]
    assert [event[4] for event in events if event[0] == 'frame'] == [int(row[1]) for row in rows if row[2] == 'OK']
    assert [event[4] for event in events if event[0] == 'frame'] == [1, 2, 3, 4, 5, 7]


def test_hello_ack_text_warns_about_a_test_build():
    fields = rover_link.unpack_hello_ack(rover_link.pack_hello_ack(
        {'build_flags': 0x04, 'reset_flags': 0, 'imu_ok': False, 'imu_who': 0, 'state_period_ms': 10,
         'status_period_ms': 100}))
    assert 'TEST BUILD (ENCODER_FAULT_TEST): never drive the truck with it' in serial_logger.hello_ack_text(fields)


# --- Stale data before the first HELLO_ACK (the ST-LINK bridge's leftover) ---------------

def pieces_bytes(stream):
    """What stale_bytes counts for a stream of pieces: each piece and its delimiter."""
    return sum(len(piece) + 1 for piece in stream.split(b'\x00') if piece)


def splice():
    """Like b5_pi_drop_1: leftover frames from before the port was opened (the last one
    cut short by the live stream), then the HELLO_ACK and live frames with other seqs and times."""
    stale = b''.join(state_frame(n, 142813 + 10 * n, speed_cps=4321) for n in range(3)) + status_frame(52)
    stale += state_frame(3, 142843)[:-5]                   # its piece ends at the HELLO_ACK's delimiter: bad
    live = hello_ack_frame() + b''.join(state_frame(156 + n, 167373 + 10 * n) for n in range(3)) + status_frame(41)
    return stale, live


def test_frames_and_bad_pieces_before_the_first_hello_ack_are_stale():
    raw, times = io.BytesIO(), io.StringIO()
    logger = serial_logger.BinaryLogger(raw, times, print_function=lambda text: None)
    stale, live = splice()
    events = []
    for start in range(0, len(stale + live), 7):              # in small chunks, as from the port
        events += logger.feed((stale + live)[start:start + 7], 0.0)
    assert raw.getvalue() == stale + live                     # raw bytes unchanged
    assert logger.stale_counts == {'frames': 4, 'bad': 1}
    assert logger.stale_bytes == pieces_bytes(stale)
    assert logger.counts['frames'] == 5 and logger.counts['bad'] == 0 and logger.bad_counts == {}
    assert logger.seq_gaps == {} and logger.t_ms_gaps == 0    # the splice is no gap
    assert [event[1] for event in events if event[0] == 'frame'] == ['HELLO_ACK', 'STATE', 'STATE', 'STATE', 'STATUS']
    assert not [event for event in events if event[0] == 'bad']
    assert logger.last_state['t_ms'] == 167393 and logger.last_status is not None
    rows = [row.split(',') for row in times.getvalue().splitlines()[1:]]
    assert [row[2] for row in rows] == ['STALE'] * 5 + ['OK'] * 5
    assert [row[3] for row in rows[:4]] == ['STATE', 'STATE', 'STATE', 'STATUS'] and rows[0][4] == '0'
    assert rows[4][3] in (rover_link.CRC_ERROR, rover_link.COBS_ERROR, rover_link.TOO_SHORT) and rows[4][4] == ''
    assert [int(row[1]) for row in rows] == list(range(1, 11))   # piece numbers as always
    assert logger.summary_text().endswith(
        f'; stale before HELLO_ACK: frames=4 bad=1 ({pieces_bytes(stale)} bytes)')
    assert logger.exit_code() == 0


def test_only_stale_data_shows_nothing_and_says_there_was_no_hello_ack():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    stale, _ = splice()
    assert logger.feed(stale + b'\x00', 0.0) == []
    assert logger.last_state is None and logger.last_status is None and logger.hello_ack is None
    assert logger.counts['frames'] == 0 and logger.status_text().startswith('frames=0 bad=0 seq_gaps=0')
    assert logger.stale_text() == f'frames=4 bad=1 ({pieces_bytes(stale)} bytes) (no HELLO_ACK in this log)'
    assert serial_logger.BinaryLogger(None, None).stale_text() == 'none (no HELLO_ACK in this log)'


def test_after_the_hello_ack_bad_pieces_and_gaps_still_count():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    stale, _ = splice()
    stream = stale + hello_ack_frame() + state_frame(0, 1000) + state_frame(2, 1020)    # one STATE lost
    stream += state_frame(3, 1030)[:-5] + b'\x00' + state_frame(4, 1040)               # one corrupted
    events = logger.feed(stream, 0.0)
    assert logger.stale_counts == {'frames': 4, 'bad': 1}
    assert logger.counts['bad'] == 1
    assert [event for event in events if event[0] == 'bad'] == [('bad', next(iter(logger.bad_counts)))]
    assert logger.seq_gaps == {'STATE': 2} and logger.t_ms_gaps == 2
    assert logger.exit_code() == 1


def test_a_later_hello_ack_changes_nothing():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    stream = hello_ack_frame() + state_frame(0, 1000) + hello_ack_frame(seq=1) + state_frame(2, 1030)
    stream += hello_ack_frame(seq=2)[:-4] + b'\x00' + state_frame(3, 1040)
    logger.feed(stream, 0.0)
    assert logger.stale_counts == {'frames': 0, 'bad': 0} and logger.stale_text() == 'none'
    assert logger.seq_gaps == {'STATE': 1} and logger.t_ms_gaps == 1 and logger.counts['bad'] == 1


def test_text_and_boot_lines_before_the_hello_ack_are_recognised_as_always():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    events = logger.feed(BOOT + v5_text_line(100) + hello_ack_frame() + state_frame(0, 1000), 0.0)
    assert [event[0] for event in events] == ['boot', 'text', 'frame', 'frame']
    assert logger.counts['boot'] == 1 and logger.counts['text'] == 1 and logger.stale_text() == 'none'


def test_a_hello_ack_before_this_sessions_hello_is_stale_too():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    logger.expect_hello()
    logger.feed(hello_ack_frame(seq=9) + state_frame(1, 100), 0.0)        # the bridge's leftover, ACK included
    assert logger.hello_ack is None and logger.stale_counts == {'frames': 2, 'bad': 0}
    logger.mark_hello_sent()
    logger.feed(hello_ack_frame(seq=0) + state_frame(7, 5000) + state_frame(8, 5010), 0.0)
    assert logger.hello_ack is not None and logger.counts['frames'] == 3 and logger.stale_counts['frames'] == 2
    assert logger.seq_gaps == {} and logger.t_ms_gaps == 0


def test_unit_tests_can_turn_the_stale_rule_off():
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None, stale_until_hello_ack=False)
    logger.feed(state_frame(0, 1000) + state_frame(1, 1010), 0.0)
    assert logger.counts['frames'] == 2 and logger.stale_text() == 'none'


def test_hello_sender_first_delay_once():
    hello = serial_logger.HelloSender(serial_logger.Sequence(), first_delay=serial_logger.HELLO_START_DELAY_SECONDS)
    assert hello.due(10.0) == [] and hello.due(10.09) == []
    assert len(hello.due(10.1)) == 1                     # 0.1 s after the first due()
    assert hello.due(10.59) == [] and len(hello.due(10.61)) == 1
    hello.acknowledged()
    hello.pending, hello.next_time = True, None          # motor_console's "hello": at once, no delay again
    assert len(hello.due(20.0)) == 1


def test_run_binary_logger_takes_no_leftover_hello_ack_for_the_answer():
    clock = BinaryClock()
    leftover = b''.join(state_frame(n, 900 + 10 * n) for n in range(3)) + hello_ack_frame(seq=3)
    received = [(50.01, leftover), (50.25, hello_ack_frame(seq=0) + state_frame(10, 3000) + state_frame(11, 3010))]
    port = BinaryFakePort(clock, received, stop_at=50.5)
    logger = serial_logger.BinaryLogger(io.BytesIO(), io.StringIO(), print_function=lambda text: None)
    hello = serial_logger.HelloSender(serial_logger.Sequence(), first_delay=serial_logger.HELLO_START_DELAY_SECONDS)
    shown = []
    serial_logger.run_binary_logger(port, clock, logger, hello, shown.append)
    times = [when for when, _ in port.writes]
    assert len(times) == 1 and 50.1 - 1e-9 <= times[0] < 50.12   # one HELLO, after the leftover arrived
    assert logger.stale_counts == {'frames': 4, 'bad': 0}
    assert logger.counts['frames'] == 3 and logger.type_counts == {'HELLO_ACK': 1, 'STATE': 2}
    assert len([text for text in shown if text.startswith('HELLO_ACK:')]) == 1


def test_main_binary_waits_before_hello_and_reports_the_leftover(tmp_path, capsys):
    import time as real_time

    class Port:
        def __init__(self):
            self.opened = real_time.monotonic()
            self.reads = 0
            self.writes = []
            self.answered = False
            self.closed = False

        in_waiting = 0

        def read(self, size):
            real_time.sleep(0.01)
            self.reads += 1
            if self.reads == 1:
                return state_frame(0, 900) + hello_ack_frame(seq=1)            # leftover, ACK included
            if self.writes and not self.answered:                              # the answer to the HELLO
                self.answered = True
                return hello_ack_frame(seq=0) + state_frame(5, 2000)
            if real_time.monotonic() - self.opened > 0.4:
                raise KeyboardInterrupt
            return b''

        def write(self, data):
            self.writes.append((real_time.monotonic() - self.opened, data))

        def close(self):
            self.closed = True

    port = Port()
    output = str(tmp_path / 'stale.log')
    assert serial_logger.main_binary(port, output) == 0
    assert len(port.writes) == 1 and port.writes[0][0] >= serial_logger.HELLO_START_DELAY_SECONDS
    assert 'frames: HELLO_ACK=1 STATE=1;' in capsys.readouterr().out.replace('\n', ' ')
    with open(output + '.times.csv', encoding='ascii') as file:
        assert [row.split(',')[2] for row in file.read().splitlines()[1:]] == ['STALE', 'STALE', 'OK', 'OK']


# --- Writes off the read path (QueuedWriter), read_gap_max, SIGHUP/SIGTERM -----------------
# 2026-10-16, b5_session_vbat_1: the 2 s flush blocked the read loop for up to 2.5 s
# during a docker pull and 1.6 s of frames were lost.

import os  # noqa: E402
import signal  # noqa: E402
import threading  # noqa: E402
import time as real_time  # noqa: E402

STALL_SECONDS = 2.0
posix_only = pytest.mark.skipif(sys.platform == 'win32', reason='SIGHUP is POSIX only')


class SlowFile:
    """A file whose write number stall_at (1-based) takes STALL_SECONDS, like the SD card
    under a docker pull. Wraps a real file (or BytesIO); works as a context manager."""

    def __init__(self, file, stall_at=3):
        self.file = file
        self.stall_at = stall_at
        self.writes = 0
        self.flushes = 0
        self.stalled_at = None

    def write(self, data):
        self.writes += 1
        if self.writes == self.stall_at:
            self.stalled_at = real_time.monotonic()
            real_time.sleep(STALL_SECONDS)
        return self.file.write(data)

    def flush(self):
        self.flushes += 1
        self.file.flush()

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        self.file.close()
        return False


def test_queued_writer_keeps_order_and_bytes_and_passes_flushes():
    target = SlowFile(io.BytesIO(), stall_at=0)
    writer = serial_logger.QueuedWriter(target)
    expected = b''
    for n in range(500):
        data = bytes([n % 251]) * (n % 37)
        writer.write(data)
        expected += data
        if n % 100 == 0:
            writer.flush()
    writer.finish()
    assert target.file.getvalue() == expected and target.flushes >= 5
    assert writer.problem_text() == '' and writer.dropped == 0


def test_queued_writer_never_blocks_the_caller_on_a_stalled_disk():
    target = SlowFile(io.BytesIO(), stall_at=1)
    writer = serial_logger.QueuedWriter(target)
    longest = 0.0
    for n in range(200):
        started = real_time.monotonic()
        writer.write(b'x' * 49)
        writer.flush()
        longest = max(longest, real_time.monotonic() - started)
        real_time.sleep(0.005)
    assert target.stalled_at is not None and longest < 0.05      # the 2 s stall never reached the caller
    writer.finish()
    assert target.file.getvalue() == b'x' * 49 * 200


def test_queued_writer_drops_and_counts_when_too_far_behind():
    release = threading.Event()

    class Blocked(io.BytesIO):
        def write(self, data):
            release.wait()
            return super().write(data)

    target = Blocked()
    writer = serial_logger.QueuedWriter(target, limit=100)
    writer.write(b'a' * 60)
    writer.write(b'b' * 60)                                      # 120 > 100 waiting: dropped, not waited for
    writer.write(b'c' * 40)
    release.set()
    deadline = real_time.monotonic() + 2.0
    while writer.queued and real_time.monotonic() < deadline:   # caught up again...
        real_time.sleep(0.01)
    writer.write(b'd' * 100)                                     # ...so the whole limit is free again
    writer.finish()
    assert target.getvalue() == b'a' * 60 + b'c' * 40 + b'd' * 100
    assert writer.dropped == 60 and writer.problem_text() == 'fell behind: 60 bytes not written'


def test_queued_writer_keeps_a_write_error_and_drops_the_rest():
    class Full(io.BytesIO):
        def write(self, data):
            if self.tell() >= 10:
                raise OSError(28, 'No space left on device')
            return super().write(data)

    writer = serial_logger.QueuedWriter(Full())
    for _ in range(4):
        writer.write(b'0123456789')
    writer.finish()
    assert writer.file.getvalue() == b'0123456789' and writer.dropped == 30
    assert writer.problem_text().startswith('failed ([Errno 28] No space left on device): 30 bytes')

    class FailsOnce(io.BytesIO):                                  # e.g. a momentary I/O error
        failed = False

        def write(self, data):
            if self.tell() >= 10 and not self.failed:
                self.failed = True
                raise OSError(5, 'Input/output error')
            return super().write(data)

    writer = serial_logger.QueuedWriter(FailsOnce())
    for _ in range(4):
        writer.write(b'0123456789')
    writer.finish()
    assert writer.file.getvalue() == b'0123456789' and writer.dropped == 30   # no silent hole: the rest is dropped


def test_read_gap_meter():
    meter = serial_logger.ReadGapMeter()
    assert meter.text() == ''
    for read_at in (None, 10.0, 10.02, 10.9, 10.92):
        meter.note(read_at)
    assert meter.text() == ' read_gap_max=880ms'
    logger = serial_logger.BinaryLogger(None, None)
    logger.feed(b'', 0.0)                                         # offline (link_dump): no field
    assert 'read_gap' not in logger.status_text() + logger.summary_text()
    logger.feed(b'', 0.0, 1.0)
    logger.feed(b'', 0.0, 1.25)
    assert logger.status_text().endswith(' read_gap_max=250ms')
    assert logger.summary_text().endswith('; read_gap_max=250ms')


class StreamingPort:
    """A board in real time, about 6 KB/s: in binary mode it answers HELLO, then sends STATE every
    10 ms and STATUS every 100 ms; in text mode it sends a v5 line every 66 ms. Every byte handed out
    is kept in delivered. After seconds, read() raises KeyboardInterrupt (Ctrl+C), or calls on_time
    (e.g. a signal) once instead. read_times: monotonic time of every read()."""

    in_waiting = 0

    def __init__(self, seconds, binary=True, on_time=None):
        self.start = real_time.monotonic()
        self.seconds = seconds
        self.binary = binary
        self.on_time = on_time
        self.streaming = not binary
        self.next_due = self.start
        self.n = 0
        self.status_n = 0
        self.pending = bytearray()
        self.delivered = bytearray()
        self.read_times = []
        self.writes = []
        self.closed = False

    def read(self, size):
        real_time.sleep(0.005)
        now = real_time.monotonic()
        self.read_times.append(now)
        if now - self.start > self.seconds:
            if self.on_time is None:
                raise KeyboardInterrupt
            self.on_time()
            self.on_time = lambda: None
            self.streaming = False
        while self.streaming and now >= self.next_due:
            if self.binary:
                self.pending += state_frame(self.n, 10 * self.n)
                if self.n % 10 == 0:
                    self.pending += status_frame(self.status_n)
                    self.status_n += 1
                self.next_due += 0.01
            else:
                self.pending += v5_text_line(100 * (self.n + 1))
                self.next_due += 0.066
            self.n += 1
        data = bytes(self.pending)
        self.pending.clear()
        self.delivered += data
        return data

    def write(self, data):
        self.writes.append(data)
        if self.binary and not self.streaming:
            self.pending += hello_ack_frame()
            self.streaming = True
            self.next_due = real_time.monotonic()

    def flush(self):
        pass

    def close(self):
        self.closed = True


def slow_open(made):
    """An open() for serial_logger / motor_console: every file it opens stalls once for 2 s."""
    def opener(path, mode='r', **options):
        file = SlowFile(open(path, mode, **options))
        made.append(file)
        return file
    return opener


def longest_read_gap(port):
    return max(b - a for a, b in zip(port.read_times, port.read_times[1:]))


def test_main_binary_loses_nothing_while_the_disk_stalls(tmp_path, monkeypatch, capsys):
    made = []
    monkeypatch.setattr(serial_logger, 'open', slow_open(made), raising=False)
    port = StreamingPort(seconds=3.5)
    output = str(tmp_path / 'stall.log')
    assert serial_logger.main_binary(port, output) == 0
    assert all(file.stalled_at is not None for file in made) and len(made) == 2   # both files stalled 2 s
    assert longest_read_gap(port) < 0.2                                        # the reads went on meanwhile
    assert (tmp_path / 'stall.log').read_bytes() == bytes(port.delivered)      # byte-exact
    summary = capsys.readouterr().out.strip().splitlines()[-1]
    assert 'bad: none; seq gaps: none; STATE t_ms gaps=0' in summary and 'WARNING' not in summary
    assert re.search(r'; read_gap_max=\d+ms$', summary)
    assert int(re.search(r'read_gap_max=(\d+)ms', summary).group(1)) < 200
    rows = (tmp_path / 'stall.log.times.csv').read_text().splitlines()
    assert len(rows) == 1 + port.n + port.status_n + 1                         # every frame, and the HELLO_ACK


def test_text_main_loses_nothing_while_the_disk_stalls(tmp_path, monkeypatch, capsys):
    made = []
    monkeypatch.setattr(serial_logger, 'open', slow_open(made), raising=False)
    port = StreamingPort(seconds=3.5, binary=False)
    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(Serial=lambda *args, **kwargs: port))
    output = tmp_path / 'stall_text.log'
    monkeypatch.setattr(sys, 'argv', ['serial_logger.py', 'COM5', str(output)])
    assert serial_logger.main() == 0
    assert all(file.stalled_at is not None for file in made) and len(made) == 2
    assert longest_read_gap(port) < 0.2 and port.closed
    assert output.read_bytes() == bytes(port.delivered)
    summary = capsys.readouterr().out.strip().splitlines()[-1]
    assert re.fullmatch(r'Stopped\. \d+ lines: good=\d+ bad=0 boot=0 t_ms_gaps=0 read_gap_max=\d+ms', summary)


def test_motor_console_binary_loop_loses_nothing_while_the_disk_stalls(tmp_path):
    import motor_console
    port = StreamingPort(seconds=3.5)
    raw_target, times_target = SlowFile(io.BytesIO(), stall_at=3), SlowFile(io.StringIO(), stall_at=3)
    with serial_logger.queued_writer(raw_target) as raw, serial_logger.queued_writer(times_target) as times:
        logger = serial_logger.BinaryLogger(raw, times, print_function=lambda text: None)
        keyboard = types.SimpleNamespace(kbhit=lambda: False, getwch=lambda: '')
        motor_console.run_binary_console(port, keyboard, real_time, logger,
                                         motor_console.StatusLine(lambda text: None, columns=lambda: 200),
                                         sleep=lambda seconds: None)
    assert raw_target.stalled_at is not None and longest_read_gap(port) < 0.2
    assert raw_target.file.getvalue() == bytes(port.delivered)
    assert logger.counts['bad'] == 0 and not logger.seq_gaps and logger.t_ms_gaps == 0


@posix_only
@pytest.mark.parametrize('name', serial_logger.EXIT_SIGNAL_NAMES)
def test_main_binary_stops_cleanly_on_sighup_and_sigterm(tmp_path, capsys, name):
    handlers = {number: signal.getsignal(number) for number in (signal.SIGHUP, signal.SIGTERM)}

    def kill_twice():                         # the second one is ignored
        os.kill(os.getpid(), getattr(signal, name))
        os.kill(os.getpid(), getattr(signal, name))

    port = StreamingPort(seconds=1.0, on_time=kill_twice)
    output = tmp_path / 'signal.log'
    started = real_time.monotonic()
    assert serial_logger.main_binary(port, str(output)) == 0
    assert real_time.monotonic() - started < 2.5 and port.closed
    assert output.read_bytes() == bytes(port.delivered)                       # written to the end
    printed = capsys.readouterr().out
    assert f'\n{name}: stopped as for Ctrl+C.\nStopped. frames: HELLO_ACK=1 STATE=' in printed
    assert {number: signal.getsignal(number) for number in handlers} == handlers


@posix_only
def test_exit_signals_keep_the_first_signal_and_ignore_the_rest():
    calls = []
    with serial_logger.ExitSignals(lambda: calls.append(1)) as signals:
        os.kill(os.getpid(), signal.SIGHUP)            # the SSH session goes...
        os.kill(os.getpid(), signal.SIGTERM)           # ...then the shutdown: ignored
        assert signals.received == 'SIGHUP' and calls == [1]


@posix_only
def test_text_main_stops_cleanly_on_sigterm(tmp_path, monkeypatch, capsys):
    port = StreamingPort(seconds=1.0, binary=False, on_time=lambda: os.kill(os.getpid(), signal.SIGTERM))
    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(Serial=lambda *args, **kwargs: port))
    output = tmp_path / 'signal_text.log'
    monkeypatch.setattr(sys, 'argv', ['serial_logger.py', 'COM5', str(output)])
    assert serial_logger.main() == 0
    assert output.read_bytes() == bytes(port.delivered) and port.closed
    lines = output.with_name('signal_text.log.times.csv').read_text().splitlines()
    assert len(lines) == 1 + port.n                                           # every line has its row
    assert '\nSIGTERM: stopped as for Ctrl+C.\nStopped. ' in capsys.readouterr().out


def test_a_write_problem_is_reported_and_fails_the_exit_code(tmp_path, monkeypatch, capsys):
    def full_open(path, mode='r', **options):
        file = open(path, mode, **options)

        class Full:
            def write(self, data):
                raise OSError(28, 'No space left on device')

            def flush(self):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *exception):
                file.close()
                return False

        return Full()

    monkeypatch.setattr(serial_logger, 'open', full_open, raising=False)
    assert serial_logger.main_binary(StreamingPort(seconds=0.5), str(tmp_path / 'full.log')) == 1
    printed = capsys.readouterr().out
    assert 'WARNING: writing the raw log failed ([Errno 28] No space left on device)' in printed
    assert 'that file is incomplete.' in printed
