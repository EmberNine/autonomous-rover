"""Tests for link_dump.py. Run from the repo root: python3 -m pytest tools/

A synthetic raw binary log (built with rover_link: a board in text mode, its
HELLO_ACK, STATE and STATUS frames, one lost and one corrupted frame, a reset
with its boot line and text, then frames again) goes through dump(), the CSV
writer and main(): every frame lands in its CSV row with every field, and
the summary and exit code say what went wrong.
"""
import csv
import io

import link_dump
import rover_link
import serial_logger

BOOT = b'imu_init=OK who=0x47\r\n'


def v5_text_line(t_ms):
    return b' '.join(f'{key}={t_ms if key == "t_ms" else 0}'.encode() for key in serial_logger.TELEMETRY_FIELDS_V5) + b'\r\n'


def hello_ack_frame():
    return rover_link.encode_frame(rover_link.HELLO_ACK, 0, rover_link.pack_hello_ack(
        {'build_flags': 0, 'reset_flags': 36, 'imu_ok': True, 'imu_who': 0x47, 'state_period_ms': 10,
         'status_period_ms': 100}))


def state_frame(seq, t_ms):
    fields = {name: 0 for name in rover_link.STATE_FIELDS}
    fields.update(t_ms=t_ms, count=seq * 7 - 100, speed_cps=-2450, gyro_z=-160, accel_z=81520, imu_samples=10,
                  motor_state=1, control_state=1, drive_seq_echo=seq)
    return rover_link.encode_frame(rover_link.STATE, seq & 0xFF, rover_link.pack_state(fields))


def status_frame(seq, t_ms):
    fields = {name: 0 for name in rover_link.STATUS_FIELDS}
    fields.update(t_ms=t_ms, setpoint_cps=2500, latch_reason=0, vbat_mv=7921, rx_ok=seq * 10, loop_us_max=702)
    return rover_link.encode_frame(rover_link.STATUS, seq & 0xFF, rover_link.pack_status(fields))


def clean_log(frames=30):
    log = v5_text_line(100) + hello_ack_frame()
    for n in range(frames):
        log += state_frame(n, 1003 + 10 * n)
        if n % 10 == 5:
            log += status_frame(n // 10, 1008 + 10 * n)
    return log


def test_a_clean_log_dumps_every_frame_and_exits_0(tmp_path):
    logger, rows = link_dump.dump(clean_log())
    assert [len(rows[name]) for name in ('STATE', 'STATUS', 'HELLO_ACK')] == [30, 3, 1]
    first = dict(zip(['piece', 'seq', *rover_link.STATE_FIELDS], rows['STATE'][0]))
    assert (first['seq'], first['t_ms'], first['count'], first['speed_cps'], first['accel_z']) == (0, 1003, -100,
                                                                                                    -2450, 81520)
    assert dict(zip(['piece', 'seq', *rover_link.STATUS_FIELDS], rows['STATUS'][2]))['rx_ok'] == 20
    assert logger.counts['bad'] == 0 and not logger.seq_gaps and logger.t_ms_gaps == 0
    assert link_dump.exit_code(logger) == 0

    log_path = tmp_path / 'clean.log'
    log_path.write_bytes(clean_log())
    assert link_dump.main([str(log_path)]) == 0
    with open(f'{log_path}.state.csv', newline='', encoding='ascii') as file:
        table = list(csv.reader(file))
    assert table[0] == ['piece', 'seq', *rover_link.STATE_FIELDS] and len(table) == 31
    assert table[1][2:4] == ['1003', '-100']
    with open(f'{log_path}.hello_ack.csv', newline='', encoding='ascii') as file:
        assert list(csv.reader(file))[1][2:] == ['1', '0', '36', '1', '71', '1000', '5000', '50', '10', '100']


def test_piece_numbers_match_the_loggers_times_csv():
    # The piece column must be each frame's own piece number, the same one the
    # live logger wrote to <log>.times.csv for it (so the two files can be
    # joined), not one number for the whole file.
    log = clean_log(12)
    log += state_frame(13, 1133)[:-5] + b'\x00'                 # a bad piece in between still counts
    log += BOOT + v5_text_line(100) + hello_ack_frame() + state_frame(0, 53)
    times = io.StringIO()
    live = serial_logger.BinaryLogger(None, times, print_function=lambda text: None)
    for start in range(0, len(log), 64):                          # the live logger reads in chunks
        live.feed(log[start:start + 64], 0.0)
    expected = {name: [] for name in link_dump.CSV_TYPES}
    for row in list(csv.reader(io.StringIO(times.getvalue())))[1:]:
        if row[2] == 'OK' and row[3] in expected:
            expected[row[3]].append((int(row[1]), int(row[4])))
    _, rows = link_dump.dump(log)
    for name in link_dump.CSV_TYPES:
        assert [(row[0], row[1]) for row in rows[name]] == expected[name], name
    pieces = [row[0] for row in rows['STATE']]
    assert pieces == sorted(set(pieces)) and pieces[0] < pieces[-1]


def test_a_lost_frame_alone_fails_the_dump():
    # Every piece intact, one STATE missing: a seq gap and a 20 ms t_ms step, and exit 1.
    log = clean_log(10) + state_frame(11, 1113)
    logger, rows = link_dump.dump(log)
    assert logger.counts['bad'] == 0 and logger.seq_gaps == {'STATE': 1} and logger.t_ms_gaps == 1
    assert link_dump.exit_code(logger) == 1
    # Only the t_ms step (a seq that follows on, a time that doesn't): exit 1 too.
    logger, _ = link_dump.dump(clean_log(10) + state_frame(10, 1113))
    assert not logger.seq_gaps and logger.t_ms_gaps == 1 and link_dump.exit_code(logger) == 1
    # A lost STATUS (no t_ms check for it): only its seq says so, and that is enough.
    logger, _ = link_dump.dump(clean_log(30).replace(status_frame(1, 1158), b''))
    assert logger.seq_gaps == {'STATUS': 1} and logger.t_ms_gaps == 0 and logger.counts['bad'] == 0
    assert link_dump.exit_code(logger) == 1


def test_losses_corruption_and_a_reset_are_reported(tmp_path, capsys):
    log = clean_log(10)
    log += state_frame(11, 1113)                    # seq 10 lost: a gap, and a 20 ms t_ms step
    body = rover_link.cobs_decode(state_frame(12, 1123)[1:-1])
    body = body[:-2] + bytes([body[-2] ^ 0x01, body[-1]])
    log += b'\x00' + rover_link.cobs_encode(body) + b'\x00'    # valid COBS, wrong CRC
    log += BOOT + v5_text_line(100)                 # reset: text between frames is not an error
    log += hello_ack_frame() + state_frame(0, 53) + state_frame(1, 63)
    logger, rows = link_dump.dump(log)
    assert len(rows['STATE']) == 13 and len(rows['HELLO_ACK']) == 2
    assert logger.bad_counts == {'CRC_ERROR': 1} and logger.seq_gaps == {'STATE': 1} and logger.t_ms_gaps == 1
    assert logger.counts['boot'] == 1 and logger.counts['text'] == 2
    assert link_dump.exit_code(logger) == 1
    log_path = tmp_path / 'bad.log'
    log_path.write_bytes(log)
    assert link_dump.main([str(log_path), '--out', str(tmp_path / 'out')]) == 1
    printed = capsys.readouterr().out
    assert 'bad: CRC_ERROR=1' in printed and 'seq gaps: STATE=1' in printed and 'STATE t_ms gaps=1' in printed
    assert (tmp_path / 'out.status.csv').is_file()

from pathlib import Path  # noqa: E402  (stale-data tests only)


# --- Stale data before the first HELLO_ACK ----------------------------------------------
# The two fixtures are the first ~2 KB of real logs (2026-10-16, Pi USB, truck build): the
# ST-LINK bridge's leftover from before the port was opened, spliced onto the live stream
# (b5_pi_drop_1: a 2-byte COBS_ERROR piece; b5_pi_after_drop_1: a CRC_ERROR piece and one
# live STATE before the HELLO_ACK), then the HELLO_ACK and live frames.

TEST_DATA = Path(__file__).parent / 'test_data'


def fixture(name):
    return (TEST_DATA / f'{name}_head.bin').read_bytes()


def test_real_stale_heads_are_stale_not_bad(tmp_path, capsys):
    for name, stale, stale_bytes, first_live_t_ms, states in (('b5_pi_drop_1', 33, 1812, 167373, 12),
                                                             ('b5_pi_after_drop_1', 23, 1325, 296423, 11)):
        logger, rows = link_dump.dump(fixture(name))
        assert logger.stale_counts == {'frames': stale, 'bad': 1} and logger.stale_bytes == stale_bytes, name
        assert logger.counts['bad'] == 0 and not logger.seq_gaps and logger.t_ms_gaps == 0, name
        assert link_dump.exit_code(logger) == 0, name
        assert len(rows['HELLO_ACK']) == 1 and len(rows['STATE']) == states, name
        assert rows['STATE'][0][2] == first_live_t_ms, name          # no stale row in the CSV
        log_path = tmp_path / f'{name}.log'
        log_path.write_bytes(fixture(name))
        assert link_dump.main([str(log_path)]) == 0
        assert f'stale before HELLO_ACK: frames={stale} bad=1 ({stale_bytes} bytes)' in capsys.readouterr().out


def test_bad_pieces_and_gaps_after_the_stale_part_still_fail():
    head = fixture('b5_pi_drop_1')
    logger, _ = link_dump.dump(head + state_frame(5, 1000)[:-5] + b'\x00')        # corrupted after the HELLO_ACK
    assert logger.counts['bad'] == 1 and link_dump.exit_code(logger) == 1
    logger, _ = link_dump.dump(clean_log(10) + state_frame(11, 1113))             # lost after the HELLO_ACK
    assert logger.stale_text() == 'none' and link_dump.exit_code(logger) == 1


def test_a_log_without_a_hello_ack_fails(capsys, tmp_path):
    frames = b''.join(state_frame(n, 1003 + 10 * n) for n in range(5))
    logger, rows = link_dump.dump(frames)
    assert logger.stale_counts == {'frames': 5, 'bad': 0} and rows['STATE'] == []
    assert link_dump.exit_code(logger) == 1
    log_path = tmp_path / 'no_ack.log'
    log_path.write_bytes(frames)
    assert link_dump.main([str(log_path)]) == 1
    assert '(no HELLO_ACK in this log)' in capsys.readouterr().out
    logger, _ = link_dump.dump(v5_text_line(100) * 3)                               # a text-only log
    assert link_dump.exit_code(logger) == 1 and logger.stale_text() == 'none (no HELLO_ACK in this log)'
