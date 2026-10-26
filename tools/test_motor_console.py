"""Tests for motor_console.py. Run from the repo root: python3 -m pytest tools/

The console loop runs against a fake serial port, a fake keyboard and a fake
clock, so these run anywhere (no Windows, no board). What they check:
keepalive timing, nothing sent before the first command, Esc and Ctrl+C,
"m 0" on every way out, board reset handling, that the log files are
exactly what serial_logger's TelemetryLogger writes for the same bytes, and
(with a fake kernel32) that QuickEdit is off while the loop runs and the
original console mode comes back on every way out. Steering: "s" keepalives
on their own schedule, "s 0" on Esc and on the way out only if steering was
commanded, the -m / -s test aids, and the v3 status fields. Closed loop:
"v", one keepalive slot shared by "m" and "v" (the last typed wins), -v,
and the v5 status (speed control, mfault). Status line:
d100 next to the motor state, never wider than the window (a fake width of
60), and the order in which parts are given up when it is too narrow.
"""
import io
import re
import sys

import pytest

import motor_console
import serial_logger
from serial_logger import BATTERY_FIELDS, MOTOR_FIELDS, STEERING_FIELDS, TELEMETRY_FIELDS_V1, TelemetryLogger


class FakeClock:
    def __init__(self):
        self.monotonic_seconds = 100.0

    def monotonic(self):
        return self.monotonic_seconds

    def time(self):
        return 1_760_000_000.0 + self.monotonic_seconds


class FakePort:
    """Each read() takes READ_TIMEOUT_SECONDS of fake time and returns the
    received bytes due by then. After stop_at it raises KeyboardInterrupt
    (Ctrl+C), unless the console has already quit."""

    def __init__(self, clock, received=(), stop_at=None, error_at=None, on_read=None):
        self.clock = clock
        self.received = sorted(received)     # (time, bytes)
        self.stop_at = stop_at
        self.error_at = error_at             # raise RuntimeError from read() from this time on
        self.on_read = on_read               # called on every read (to look at state mid-run)
        self.writes = []                     # (time, bytes)
        self.closed = False
        self.flushed = False

    @property
    def in_waiting(self):
        return 0

    def read(self, size):
        self.clock.monotonic_seconds += motor_console.READ_TIMEOUT_SECONDS
        if self.on_read is not None:
            self.on_read()
        if self.error_at is not None and self.clock.monotonic_seconds >= self.error_at:
            raise RuntimeError('serial port gone')
        if self.stop_at is not None and self.clock.monotonic_seconds >= self.stop_at:
            raise KeyboardInterrupt
        due = b''
        while self.received and self.received[0][0] <= self.clock.monotonic_seconds:
            due += self.received.pop(0)[1]
        return due

    def write(self, data):
        assert not self.closed
        self.writes.append((self.clock.monotonic_seconds, data))

    def flush(self):
        self.flushed = True

    def close(self):
        self.closed = True

    def sent(self):
        return [data for _time, data in self.writes]


class FakeKeyboard:
    """Keys typed at given times: [(time, 'm 20\\r'), (time, ESC), ...]."""

    def __init__(self, clock, typed):
        self.clock = clock
        self.keys = [(when, key) for when, text in sorted(typed) for key in text]

    def kbhit(self):
        return bool(self.keys) and self.keys[0][0] <= self.clock.monotonic_seconds

    def getwch(self):
        return self.keys.pop(0)[1]


class FakeKernel32:
    """The three console calls QuickEditOff uses, on one fake console input
    buffer. GetConsoleMode writes through the pointer like the real call."""

    HANDLE = 0x5C

    def __init__(self, mode=0x01F7, handle=HANDLE, get_ok=True, set_results=()):
        self.mode = mode                    # the console's input mode
        self.handle = handle
        self.get_ok = get_ok
        self.set_results = list(set_results)  # per SetConsoleMode call; True once these run out
        self.std_handles_asked = []
        self.set_calls = []

    def GetStdHandle(self, which):
        self.std_handles_asked.append(which)
        return self.handle

    def GetConsoleMode(self, handle, mode_pointer):
        assert handle == self.handle
        if not self.get_ok:
            return 0
        mode_pointer.contents.value = self.mode
        return 1

    def SetConsoleMode(self, handle, mode):
        assert handle == self.handle
        self.set_calls.append(mode)
        ok = self.set_results.pop(0) if self.set_results else True
        if ok:
            self.mode = mode
        return 1 if ok else 0


def run(typed=(), received=(), stop_after=3.0, kernel32=None, error_after=None, columns=200):
    """Runs the console loop on fakes. With kernel32, inside QuickEditOff (as
    main() does), and result['modes'] holds the console mode at every read.
    columns: the fake window width (wide by default, so nothing is given up)."""
    clock = FakeClock()
    start = clock.monotonic_seconds
    modes = []
    port = FakePort(clock, [(start + when, data) for when, data in received], stop_at=start + stop_after,
                    error_at=None if error_after is None else start + error_after,
                    on_read=None if kernel32 is None else lambda: modes.append(kernel32.mode))
    keyboard = FakeKeyboard(clock, [(start + when, text) for when, text in typed])
    raw_file, times_file, screen = io.BytesIO(), io.StringIO(), io.StringIO()
    status_line = motor_console.StatusLine(screen.write, columns=lambda: columns)
    logger = TelemetryLogger(raw_file, times_file, print_function=status_line.message)
    sleeps = []
    if kernel32 is None:
        console = motor_console.run_console(port, keyboard, clock, logger, status_line, sleep=sleeps.append)
    else:
        with motor_console.QuickEditOff(kernel32, status_line.message):
            console = motor_console.run_console(port, keyboard, clock, logger, status_line, sleep=sleeps.append)
    return {'port': port, 'console': console, 'logger': logger, 'raw': raw_file.getvalue(),
            'times': times_file.getvalue(), 'screen': screen.getvalue(), 'start': start, 'sleeps': sleeps,
            'modes': modes}


def telemetry_line(t_ms, duty=0, mstate=2):
    values = {name: 0 for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS}
    values.update(t_ms=t_ms, imu_ok=1, duty=duty, mstate=mstate)
    return (' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS) + '\r\n').encode()


EXIT_STOPS = [b'm 0\n'] * motor_console.EXIT_STOP_REPEATS


# --- Commands -----------------------------------------------------------------

def test_command_grammar_matches_the_firmware():
    for text in ('m 0', 'm 20', 'm -15', 'm 100', 'm -100', 'm -0', 'm 007'):
        assert motor_console.is_motor_command(text), text
    for text in ('m', 'm ', 'm -', 'M 20', 'm  20', 'm +20', 'm 101', 'm -101', 'm 1000', 'm 20 ', 'm20', 'x 20'):
        assert not motor_console.is_motor_command(text), text


def test_nothing_is_sent_before_the_first_command():
    result = run(stop_after=1.0)
    assert result['port'].sent() == EXIT_STOPS


def test_typed_command_is_sent_at_once_then_every_100_ms():
    result = run(typed=[(0.5, 'm 20\r')], stop_after=1.505)
    port, start = result['port'], result['start']
    keepalive_times = [when - start for when, data in port.writes if data == b'm 20\n']
    assert abs(keepalive_times[0] - 0.5) < 0.03            # sent on Enter
    gaps = [later - earlier for earlier, later in zip(keepalive_times, keepalive_times[1:])]
    # Reads come every 20 ms, so a keepalive can go out up to one read late,
    # but they are scheduled from the planned time, so they don't drift.
    assert all(0.08 <= gap <= 0.121 for gap in gaps), gaps
    assert len(keepalive_times) >= 9
    assert keepalive_times[-1] >= 1.38
    assert port.sent()[-3:] == EXIT_STOPS
    assert port.closed and port.flushed


def test_esc_sends_stop_at_once_and_keeps_sending_it():
    result = run(typed=[(0.2, 'm 25\r'), (0.71, motor_console.ESC)], stop_after=1.2)
    port, start = result['port'], result['start']
    stop_times = [when - start for when, data in port.writes if data == b'm 0\n']
    assert abs(stop_times[0] - 0.71) < 0.03                 # within one read of Esc
    assert not any(data == b'm 25\n' and when - start > 0.72 for when, data in port.writes)
    assert len(stop_times) >= 4 + motor_console.EXIT_STOP_REPEATS  # keepalive m 0, then the exit stops


def test_ctrl_c_key_sends_stop_and_quits():
    result = run(typed=[(0.2, 'm 15\r'), (0.6, motor_console.CTRL_C)], stop_after=10.0)
    port, start = result['port'], result['start']
    assert result['console'].quit_requested
    assert port.writes[-1][0] - start < 0.7                 # quit right away, not at stop_after
    assert port.sent()[-3:] == EXIT_STOPS
    assert result['sleeps'] == [motor_console.EXIT_STOP_SPACING_SECONDS] * motor_console.EXIT_STOP_REPEATS
    assert port.closed


def test_keyboard_interrupt_also_sends_stop():
    result = run(typed=[(0.2, 'm -10\r')], stop_after=0.9)  # KeyboardInterrupt from read()
    assert result['port'].sent()[-3:] == EXIT_STOPS
    assert result['port'].closed


def test_invalid_text_is_not_sent_and_raw_text_is_sent_once():
    result = run(typed=[(0.1, 'm 2x\r'), (0.3, '!garbage line\r'), (0.5, 'm 101\r')], stop_after=1.0)
    sent = result['port'].sent()
    assert sent == [b'garbage line\n'] + EXIT_STOPS        # only the raw line, once
    assert "not sent: 'm 2x'" in result['screen']
    assert "not sent: 'm 101'" in result['screen']
    assert "sent once, as-is: 'garbage line'" in result['screen']


def test_backspace_and_special_keys():
    # Typo fixed with backspace; an arrow key (prefix + code) is ignored.
    result = run(typed=[(0.1, 'm 29' + motor_console.BACKSPACE + '0\xe0H\r')], stop_after=0.5)
    assert result['port'].sent()[0] == b'm 20\n'


def test_board_reset_stops_sending():
    received = [(0.1, b'imu_init=ok who=0x47\r\n'), (0.2, telemetry_line(100)), (0.8, b'imu_init=ok who=0x47\r\n')]
    result = run(typed=[(0.3, 'm 20\r')], received=received, stop_after=1.5)
    port, start = result['port'], result['start']
    last_keepalive = max(when - start for when, data in port.writes if data == b'm 20\n')
    assert 0.7 <= last_keepalive <= 0.83                    # nothing after the reset at 0.8
    assert 'board reset: sending stopped' in result['screen']
    assert result['console'].motor_command is None


# --- Steering -----------------------------------------------------------------

EXIT_STOPS_WITH_STEERING = [b'm 0\n', b's 0\n'] * motor_console.EXIT_STOP_REPEATS


def write_times(result, data):
    return [when - result['start'] for when, written in result['port'].writes if written == data]


def test_steering_grammar_matches_the_firmware():
    for text in ('s 0', 's 100', 's -100', 's -0', 's 35', 's 007'):
        assert motor_console.is_steering_command(text) and motor_console.command_letter(text) == 's', text
        assert not motor_console.is_motor_command(text), text
    for text in ('s', 's ', 's -', 'S 20', 's  20', 's +20', 's 101', 's -101', 's 20 ', 's20', 'h 30', 'x 5'):
        assert motor_console.command_letter(text) is None, text


def test_motor_and_steering_keepalives_run_on_their_own_schedules():
    result = run(typed=[(0.2, 'm 20\r'), (0.55, 's 30\r')], stop_after=1.605)
    motor_times = write_times(result, b'm 20\n')
    steering_times = write_times(result, b's 30\n')
    assert abs(motor_times[0] - 0.2) < 0.03 and abs(steering_times[0] - 0.55) < 0.03
    # Typing "s" didn't shift the motor keepalives: same spacing all the way.
    for times in (motor_times, steering_times):
        gaps = [later - earlier for earlier, later in zip(times, times[1:])]
        assert all(0.08 <= gap <= 0.121 for gap in gaps), gaps
    assert len(motor_times) >= 13 and len(steering_times) >= 9
    assert result['port'].sent()[-6:] == EXIT_STOPS_WITH_STEERING


def test_keepalives_due_in_the_same_read_both_go_out_in_that_read():
    # Typed together, so both fall due on every same read: neither may wait for the next one.
    result = run(typed=[(0.2, 'm 20\rs 30\r')], stop_after=1.205)
    motor_times = write_times(result, b'm 20\n')
    steering_times = write_times(result, b's 30\n')
    assert len(motor_times) >= 9 and motor_times == steering_times
    # "m" first within each read.
    sent = result['port'].sent()
    for index, data in enumerate(sent):
        if data == b's 30\n':
            assert sent[index - 1] == b'm 20\n'


def test_no_steering_line_is_ever_sent_unless_typed():
    result = run(typed=[(0.2, 'm 20\r'), (0.5, motor_console.ESC), (0.8, motor_console.CTRL_C)], stop_after=5.0)
    assert not any(data.startswith(b's') for data in result['port'].sent())
    assert result['port'].sent()[-3:] == EXIT_STOPS


def test_esc_with_steering_stops_and_centres_and_keeps_sending_both():
    result = run(typed=[(0.2, 'm 25\r'), (0.3, 's -60\r'), (0.71, motor_console.ESC)], stop_after=1.2)
    assert abs(write_times(result, b'm 0\n')[0] - 0.71) < 0.03
    assert abs(write_times(result, b's 0\n')[0] - 0.71) < 0.03
    assert not any(when > 0.72 for when in write_times(result, b's -60\n') + write_times(result, b'm 25\n'))
    assert len(write_times(result, b's 0\n')) >= 4 + motor_console.EXIT_STOP_REPEATS
    assert 'Esc: m 0, s 0' in result['screen']


def test_ctrl_c_with_steering_sends_both_stops_then_quits():
    result = run(typed=[(0.2, 's 10\r'), (0.3, 'm 15\r'), (0.6, motor_console.CTRL_C)], stop_after=10.0)
    assert result['console'].quit_requested
    assert result['port'].sent()[-6:] == EXIT_STOPS_WITH_STEERING
    # One spacing per repeat, the pair written together.
    assert result['sleeps'] == [motor_console.EXIT_STOP_SPACING_SECONDS] * motor_console.EXIT_STOP_REPEATS


def test_keyboard_interrupt_with_only_steering_also_sends_both_stops():
    result = run(typed=[(0.2, 's -20\r')], stop_after=0.9)
    assert result['port'].sent()[-6:] == EXIT_STOPS_WITH_STEERING
    assert not any(data.startswith(b'm') for data in result['port'].sent()[:-6])  # no m before the exit stops


def test_stop_keepalive_test_aids():
    typed = [(0.1, 'm 20\r'), (0.15, 's 10\r'), (0.5, '-m\r'), (1.0, '-s\r'), (1.2, '-s\r')]
    result = run(typed=typed, stop_after=1.6)
    motor_times = write_times(result, b'm 20\n')
    steering_times = write_times(result, b's 10\n')
    assert max(motor_times) < 0.52 and max(steering_times) > 0.9   # -m stops only the motor keepalive
    assert max(steering_times) < 1.02                              # -s stops the steering one
    screen = result['screen']
    assert '-m: stopped sending m keepalives; the firmware motor watchdog acts within 500 ms' in screen
    assert '-s: stopped sending s keepalives; the firmware steering timeout (-> centre) acts within 500 ms' in screen
    assert '-s: no s keepalive was being sent' in screen
    # Nothing is being sent any more, so the exit is "m 0" only.
    assert result['port'].sent()[-3:] == EXIT_STOPS and b's 0\n' not in result['port'].sent()


def test_board_reset_stops_steering_too_and_no_centre_is_sent_after_it():
    received = [(0.6, b'imu_init=ok who=0x47\r\n')]
    result = run(typed=[(0.1, 'm 20\r'), (0.2, 's 30\r')], received=received, stop_after=1.2)
    assert max(write_times(result, b's 30\n')) <= 0.63
    assert result['console'].steering_command is None and result['console'].motor_command is None
    # The servo has no pulses after the reset: "s 0" on exit would start them.
    assert b's 0\n' not in result['port'].sent()
    assert 'the servo has no pulses until "s"' in result['screen']


def test_reset_cause_text():
    assert motor_console.reset_cause_text(36) == 'IWDG+PIN'
    assert motor_console.reset_cause_text(12) == 'BOR+PIN'
    assert motor_console.reset_cause_text(20) == 'SFT+PIN'
    assert motor_console.reset_cause_text(4) == 'PIN'
    assert motor_console.reset_cause_text(0) == 'none'
    assert motor_console.reset_cause_text(255) == 'IWDG+WWDG+SFT+LPWR+BOR+OBL+FW+PIN'


def v3_line(t_ms):
    values = {name: 0 for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS + STEERING_FIELDS}
    values.update(t_ms=t_ms, d100=-245, imu_ok=1, duty=320, mstate=1, rst=36, s_pct=-40, s_us=1380, sstate=1)
    names = TELEMETRY_FIELDS_V1 + MOTOR_FIELDS + STEERING_FIELDS
    return (' '.join(f'{name}={values[name]}' for name in names) + '\r\n').encode()


def test_status_line_shows_steering_and_reset_cause_from_v3_lines():
    received = [(0.05 + 0.1 * index, v3_line(100 * (index + 1))) for index in range(30)]
    result = run(typed=[(0.2, 'm 20\r'), (0.3, 's -40\r')], received=received, stop_after=3.0)
    shown = [line for line in result['screen'].split('\r') if line.startswith('good=')]
    assert any(re.match(r'good=\d+ bad=0 \(0\.00%\) boot=0 t_ms_gaps=0   \| sent: m 20 s -40 \| duty=320 mstate=1'
                        r' d100=-245 \| s=-40 us=1380 sst=1 \| rst=36 \(IWDG\+PIN\) \| > ', line) for line in shown), shown[-3:]
    assert result['logger'].counts['bad'] == 0


# --- Logging and screen -------------------------------------------------------

def test_log_files_are_what_the_logger_writes():
    received = [(0.05 + 0.1 * index, telemetry_line(100 * (index + 1), duty=2 * index, mstate=1))
                for index in range(20)]
    received.insert(5, (0.33, b'garbage\r\n'))
    result = run(typed=[(0.2, 'm 20\r')], received=received, stop_after=2.5)
    stream = b''.join(data for _when, data in sorted(received))
    assert result['raw'] == stream                          # raw bytes unchanged

    # The times file has one row per line, in the logger's format.
    rows = result['times'].splitlines()
    assert rows[0] == 'host_time_unix,line,byte_offset,result'
    assert len(rows) == 1 + 21
    assert all(re.fullmatch(r'\d+\.\d{6},\d+,\d+,(good|bad|first_partial)', row) for row in rows[1:])

    # Same results as feeding the same bytes to a TelemetryLogger directly.
    reference = TelemetryLogger(io.BytesIO(), io.StringIO(), print_function=lambda *args: None)
    reference.feed(stream, 0.0)
    assert result['logger'].counts == reference.counts == {'good': 20, 'bad': 1, 'boot': 0}
    # The console times its reads (each fake read takes 20 ms); the offline reference doesn't.
    assert result['logger'].summary_text() == reference.summary_text() + ' read_gap_max=20ms'


def test_status_line_starts_with_the_loggers_status_text():
    received = [(0.05 + 0.1 * index, telemetry_line(100 * (index + 1), duty=320, mstate=1)) for index in range(30)]
    result = run(typed=[(0.2, 'm 20\r'), (2.5, 'm 1')], received=received, stop_after=3.0)
    screen_lines = result['screen'].split('\r')
    shown = [line for line in screen_lines if line.startswith('good=')]
    # The logger's own status text, unchanged, then the console's part.
    assert any(re.match(r'good=\d+ bad=0 \(0\.00%\) boot=0 t_ms_gaps=0   \| sent: m 20 \| duty=320 mstate=1 d100=0 \| > ', line)
               for line in shown), shown[-3:]
    assert shown[-1].rstrip().endswith('> m 1')             # typed text visible, not yet sent


def test_status_line_shows_d100_from_v1_lines_too():
    values = {name: 0 for name in TELEMETRY_FIELDS_V1}
    values.update(t_ms=100, d100=37, imu_ok=1)
    v1_line = (' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V1) + '\r\n').encode()
    assert motor_console.motor_status_text(v1_line) == 'd100=37'
    assert motor_console.motor_status_text(v3_line(100)) == 'duty=320 mstate=1 d100=-245'
    assert motor_console.motor_status_text(None) == ''


def test_imu_missing_marker_is_shown_and_never_given_up():
    values = {name: 0 for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS + STEERING_FIELDS}
    values.update(t_ms=100, imu_ok=0, duty=0, mstate=2, rst=12, s_us=0, sstate=0)
    names = TELEMETRY_FIELDS_V1 + MOTOR_FIELDS + STEERING_FIELDS
    no_imu_line = (' '.join(f'{name}={values[name]}' for name in names) + '\r\n').encode()
    logger = logger_with(no_imu_line)
    console = motor_console.CommandConsole()
    full = motor_console.status_line_text(logger.status_text(), console, logger, 1000)
    assert ' | IMU FAIL | duty=0 mstate=2 d100=0 | ' in full
    essentials = 'IMU FAIL | duty=0 mstate=2 d100=0 | s=0 us=0 sst=0 | > '
    assert motor_console.status_line_text(logger.status_text(), console, logger, len(essentials)) == essentials
    # imu_ok=1 (and lines before v1's imu_ok can't occur): no marker.
    assert 'IMU FAIL' not in motor_console.status_line_text('', console, logger_with(v3_line(100)), 1000)
    assert not motor_console.imu_missing(None)


NARROW_COLUMNS = 60


def test_status_line_never_wraps_in_a_60_column_window():
    received = [(0.05 + 0.1 * index, v3_line(100 * (index + 1))) for index in range(30)]
    result = run(typed=[(0.2, 'm 20\r'), (0.3, 's -40\r'), (2.5, 'm 1')], received=received, stop_after=3.0,
                 columns=NARROW_COLUMNS)
    # Everything written between two '\r' (or a '\n') is what one row shows.
    pieces = [piece for row in result['screen'].split('\n') for piece in row.split('\r')]
    assert pieces and max(len(piece) for piece in pieces) <= NARROW_COLUMNS - 1, max(pieces, key=len)
    last = [piece for piece in pieces if piece.strip()][-1]
    for kept in ('duty=320 mstate=1 d100=-245', 's=-40 us=1380 sst=1'):
        assert kept in last, last
    assert last.rstrip().endswith('> m 1'), last
    assert 'IWDG' not in last                                # the decode went first


def logger_with(line):
    logger = TelemetryLogger(io.BytesIO(), io.StringIO(), print_function=lambda *args: None)
    logger.feed(line, 0.0)
    return logger


def test_narrow_window_gives_up_parts_in_priority_order():
    logger = logger_with(v3_line(100))
    console = motor_console.CommandConsole()
    console.handle_key('m', 0.0)                              # "m 20" sent, "m 1" being typed
    for key in ' 20\r' + 'm 1':
        console.handle_key(key, 0.0)
    status_text = logger.status_text()

    def text(max_length):
        return motor_console.status_line_text(status_text, console, logger, max_length)

    full = text(1000)
    assert full == (f'{status_text}   | sent: m 20 | duty=320 mstate=1 d100=-245 | s=-40 us=1380 sst=1'
                    ' | rst=36 (IWDG+PIN) | > m 1')
    no_decode = text(len(full) - 1)
    assert no_decode == full.replace(' (IWDG+PIN)', '')
    short_counts = text(len(no_decode) - 1)
    assert short_counts.startswith('good=1 bad=0   | sent: m 20 |') and 'rst=36 |' in short_counts
    no_reset = text(len(short_counts) - 1)
    assert no_reset == 'good=1 bad=0   | sent: m 20 | duty=320 mstate=1 d100=-245 | s=-40 us=1380 sst=1 | > m 1'
    no_counts = text(len(no_reset) - 1)
    assert no_counts == 'sent: m 20 | duty=320 mstate=1 d100=-245 | s=-40 us=1380 sst=1 | > m 1'
    essentials = text(len(no_counts) - 1)
    assert essentials == 'duty=320 mstate=1 d100=-245 | s=-40 us=1380 sst=1 | > m 1'
    # Narrower still: cut from the left, so the typed text stays visible.
    assert text(len(essentials) - 1) == essentials[1:]
    for max_length in range(1, len(full) + 5):
        shown = text(max_length)
        assert len(shown) <= max_length and shown.endswith('> m 1'[-max_length:])
        if max_length >= len(essentials):
            assert 'duty=320 mstate=1 d100=-245' in shown and 's=-40 us=1380 sst=1' in shown


def v4_line(t_ms, vbat=7921, bstate=1, vdda=3299):
    names = TELEMETRY_FIELDS_V1 + MOTOR_FIELDS + STEERING_FIELDS + BATTERY_FIELDS
    values = {name: 0 for name in names}
    values.update(t_ms=t_ms, d100=-245, imu_ok=1, duty=320, mstate=1, rst=36, s_pct=-40, s_us=1380, sstate=1,
                  vbat=vbat, bstate=bstate, vdda=vdda)
    return (' '.join(f'{name}={values[name]}' for name in names) + '\r\n').encode()


def test_battery_status_text():
    assert motor_console.battery_status_text(v4_line(100)) == 'vbat=7.92V bst=1'
    assert motor_console.battery_status_text(v4_line(100, vbat=12, bstate=0)) == 'NO BAT vbat=0.01V bst=0'
    assert motor_console.battery_status_text(v4_line(100, vbat=6950, bstate=2)) == 'LOW BAT vbat=6.95V bst=2'
    assert motor_console.battery_status_text(v4_line(100, vbat=6585, bstate=3)) == 'CUTOFF vbat=6.59V bst=3'
    assert motor_console.battery_status_text(v4_line(100, vbat=7900, bstate=4)) == 'ADC FAULT vbat=7.90V bst=4'
    assert motor_console.battery_status_text(v4_line(100, vbat=4200, bstate=5)) == 'REMOVED WHILE DRIVING vbat=4.20V bst=5'
    assert motor_console.battery_status_text(v4_line(100, vbat=8396, bstate=1)) == 'vbat=8.40V bst=1'   # rounded
    assert motor_console.battery_status_text(v4_line(100, vbat=9995, bstate=1)) == 'vbat=10.00V bst=1'  # carry
    assert motor_console.battery_status_text(v4_line(100, bstate=9)) == 'BSTATE 9? vbat=7.92V bst=9'      # unknown
    assert motor_console.battery_status_text(v3_line(100)) == ''                                       # no battery in v3
    assert motor_console.battery_status_text(None) == ''


def test_battery_is_shown_and_kept_in_a_narrow_window():
    logger = logger_with(v4_line(100, vbat=6585, bstate=3))
    console = motor_console.CommandConsole()
    full = motor_console.status_line_text(logger.status_text(), console, logger, 1000)
    assert ' | s=-40 us=1380 sst=1 | CUTOFF vbat=6.59V bst=3 | rst=36 (IWDG+PIN) | > ' in full
    essentials = 'duty=320 mstate=1 d100=-245 | s=-40 us=1380 sst=1 | CUTOFF vbat=6.59V bst=3 | > '
    assert motor_console.status_line_text(logger.status_text(), console, logger, len(essentials)) == essentials


def test_padding_and_messages_never_wider_than_the_window_even_after_a_resize():
    widths = [200]
    screen = io.StringIO()
    status_line = motor_console.StatusLine(screen.write, columns=lambda: widths[0])
    logger = logger_with(v3_line(100))
    console = motor_console.CommandConsole()
    status_line.show(logger.status_text(), console, logger)
    assert status_line.previous_length > NARROW_COLUMNS       # drawn wide
    wide_draw_length = len(screen.getvalue())
    widths[0] = NARROW_COLUMNS                                # window made narrower
    status_line.message('note')                               # blanks the wide line: only as wide as the window
    status_line.show(logger.status_text(), console, logger)
    status_line.show(logger.status_text(), console, logger)
    written_after_resize = screen.getvalue()[wide_draw_length:]
    pieces = [piece for row in written_after_resize.split('\n') for piece in row.split('\r')]
    assert max(len(piece) for piece in pieces) <= NARROW_COLUMNS - 1
    assert '\rnote\n' in written_after_resize

    # Same with a redraw (not a message) right after the window got narrower.
    widths[0] = 200
    status_line.show(logger.status_text(), console, logger)
    wide_draw_length = len(screen.getvalue())
    widths[0] = NARROW_COLUMNS
    status_line.show(logger.status_text(), console, logger)
    pieces = screen.getvalue()[wide_draw_length:].split('\r')
    assert max(len(piece) for piece in pieces) <= NARROW_COLUMNS - 1


def test_default_width_comes_from_the_terminal(monkeypatch):
    monkeypatch.setattr(motor_console.shutil, 'get_terminal_size', lambda fallback: type('Size', (), {'columns': 41})())
    status_line = motor_console.StatusLine(io.StringIO().write)
    assert status_line._max_length() == 40


# --- QuickEdit ----------------------------------------------------------------

QUICK_EDIT = motor_console.ENABLE_QUICK_EDIT_MODE
EXTENDED = motor_console.ENABLE_EXTENDED_FLAGS


def quick_edit_notes(screen):
    return [line for line in screen.splitlines() if 'QuickEdit' in line]


def test_quick_edit_cleared_with_extended_flags_and_other_bits_kept():
    # Original mode without EXTENDED set: it must be added both ways, or the
    # QuickEdit bit is ignored (Microsoft docs, SetConsoleMode).
    kernel32 = FakeKernel32(mode=0x0047)              # QuickEdit + processed/line/echo input
    notes = []
    with motor_console.QuickEditOff(kernel32, notes.append):
        assert kernel32.std_handles_asked == [-10]     # STD_INPUT_HANDLE
        assert kernel32.mode == (0x0047 | EXTENDED) & ~QUICK_EDIT == 0x0087
    assert kernel32.set_calls == [0x0087, 0x0047 | EXTENDED]
    assert notes == [motor_console.QUICK_EDIT_OFF_NOTE]


def test_quick_edit_off_while_running_and_restored_after_ctrl_c_key():
    kernel32 = FakeKernel32(mode=0x01F7)
    result = run(typed=[(0.2, 'm 15\r'), (0.6, motor_console.CTRL_C)], stop_after=10.0, kernel32=kernel32)
    assert result['console'].quit_requested
    assert result['modes'] and all(mode & QUICK_EDIT == 0 and mode & EXTENDED for mode in result['modes'])
    assert kernel32.mode == 0x01F7                      # original back
    assert kernel32.set_calls == [0x01B7, 0x01F7]
    assert result['port'].sent()[-3:] == EXIT_STOPS
    assert quick_edit_notes(result['screen']) == [motor_console.QUICK_EDIT_OFF_NOTE]


def test_quick_edit_restored_after_keyboard_interrupt():
    kernel32 = FakeKernel32(mode=0x01F7)
    result = run(typed=[(0.2, 'm -10\r')], stop_after=0.9, kernel32=kernel32)  # KeyboardInterrupt from read()
    assert result['modes'] and all(mode & QUICK_EDIT == 0 for mode in result['modes'])
    assert kernel32.mode == 0x01F7
    assert result['port'].sent()[-3:] == EXIT_STOPS


def test_quick_edit_restored_after_an_exception_which_still_propagates():
    kernel32 = FakeKernel32(mode=0x01F7)
    with pytest.raises(RuntimeError, match='serial port gone'):
        run(typed=[(0.2, 'm 20\r')], stop_after=10.0, kernel32=kernel32, error_after=0.5)
    assert kernel32.set_calls == [0x01B7, 0x01F7]
    assert kernel32.mode == 0x01F7


def test_console_runs_when_the_mode_cannot_be_read():
    kernel32 = FakeKernel32(get_ok=False)              # e.g. stdin redirected: not a console
    result = run(typed=[(0.2, 'm 20\r')], stop_after=0.6, kernel32=kernel32)
    assert kernel32.set_calls == []                     # nothing changed, nothing to restore
    notes = quick_edit_notes(result['screen'])
    assert len(notes) == 1 and 'could not turn QuickEdit off' in notes[0] and 'not a console' in notes[0]
    assert b'm 20\n' in result['port'].sent()          # the console itself works as usual


def test_console_runs_when_the_mode_cannot_be_set():
    kernel32 = FakeKernel32(mode=0x01F7, set_results=[False])
    result = run(typed=[(0.2, 'm 20\r')], stop_after=0.6, kernel32=kernel32)
    assert kernel32.set_calls == [0x01B7]               # tried once; no "restore" of a change that didn't happen
    assert kernel32.mode == 0x01F7
    notes = quick_edit_notes(result['screen'])
    assert len(notes) == 1 and 'SetConsoleMode failed' in notes[0]
    assert b'm 20\n' in result['port'].sent()


def test_console_runs_without_a_console_api_or_handle():
    if sys.platform != 'win32':
        assert motor_console.load_kernel32() is None    # no WinDLL off Windows: must not raise
    for kernel32, reason in ((None, 'no Windows console API'),
                             (FakeKernel32(handle=None), 'no console input handle'),
                             (FakeKernel32(handle=motor_console.INVALID_HANDLE_VALUE), 'no console input handle')):
        notes = []
        with motor_console.QuickEditOff(kernel32, notes.append):
            pass
        assert len(notes) == 1 and reason in notes[0], notes
        if kernel32 is not None:
            assert kernel32.set_calls == []
    result = run(typed=[(0.2, 'm 20\r')], stop_after=0.6, kernel32=FakeKernel32(handle=None))
    assert b'm 20\n' in result['port'].sent()


# --- Closed loop: "v" and telemetry v5 -----------------------------------------------

def test_v_grammar_matches_the_firmware():
    for text in ('v 0', 'v 50', 'v -30', 'v 100', 'v -100', 'v -0', 'v 007'):
        assert motor_console.command_letter(text) == 'v' and motor_console.is_motor_command(text), text
        assert not motor_console.is_steering_command(text), text
    for text in ('v', 'v ', 'v -', 'V 20', 'v  20', 'v +20', 'v 101', 'v -101', 'v 20 ', 'v20'):
        assert motor_console.command_letter(text) is None, text


def test_m_and_v_share_one_keepalive_slot_and_the_last_one_wins():
    result = run(typed=[(0.2, 'm 20\r'), (0.6, 'v 50\r'), (1.0, 'm 10\r')], stop_after=1.505)
    m20, v50, m10 = (write_times(result, data) for data in (b'm 20\n', b'v 50\n', b'm 10\n'))
    assert abs(v50[0] - 0.6) < 0.03 and max(m20) < 0.61      # v replaced m: no m 20 after it
    assert abs(m10[0] - 1.0) < 0.03 and max(v50) < 1.01      # and m replaced v
    gaps = [later - earlier for earlier, later in zip(v50, v50[1:])]
    assert len(v50) >= 4 and all(0.08 <= gap <= 0.121 for gap in gaps), gaps
    # Never two motor lines in the same read (one slot), the exit stops aside.
    driving = result['port'].writes[:-motor_console.EXIT_STOP_REPEATS]
    per_read = {}
    for when, data in driving:
        if data[:1] in (b'm', b'v'):
            per_read[when] = per_read.get(when, 0) + 1
    assert per_read and max(per_read.values()) == 1, per_read
    assert result['console'].motor_command == 'm 10'
    assert result['port'].sent()[-3:] == EXIT_STOPS


def test_v_and_steering_keepalives_are_independent():
    result = run(typed=[(0.2, 'v 40\r'), (0.55, 's 30\r')], stop_after=1.605)
    assert len(write_times(result, b'v 40\n')) >= 13 and len(write_times(result, b's 30\n')) >= 9
    assert result['port'].sent()[-6:] == EXIT_STOPS_WITH_STEERING


def test_esc_and_exit_after_v_send_m_0():
    result = run(typed=[(0.2, 'v 40\r'), (0.71, motor_console.ESC)], stop_after=1.2)
    assert abs(write_times(result, b'm 0\n')[0] - 0.71) < 0.03
    assert not any(when > 0.72 for when in write_times(result, b'v 40\n'))
    assert result['port'].sent()[-3:] == EXIT_STOPS


def test_stop_keepalive_test_aids_for_v():
    typed = [(0.1, 'v 30\r'), (0.5, '-v\r'), (0.6, 'm 20\r'), (0.9, '-v\r'), (1.2, '-m\r')]
    result = run(typed=typed, stop_after=1.6)
    assert max(write_times(result, b'v 30\n')) < 0.52
    assert max(write_times(result, b'm 20\n')) < 0.92                # -v stops an m too: one slot
    screen = result['screen']
    assert '-v: stopped sending v keepalives; the firmware motor watchdog acts within 500 ms' in screen
    assert '-v: stopped sending m keepalives (m and v share one slot); the firmware motor watchdog' in screen
    assert '-m: no m keepalive was being sent' in screen


def test_board_reset_stops_v_too():
    result = run(typed=[(0.2, 'v 50\r')], received=[(0.6, b'imu_init=ok who=0x47\r\n')], stop_after=1.2)
    assert not any(when > 0.62 for when in write_times(result, b'v 50\n'))
    assert result['console'].motor_command is None


def v5_line(t_ms, vset=50, vest=49, ctl=1, vok=1, vi=307, stall_n=0, enc_bad=0, mfault=0, mstate=1):
    names = serial_logger.TELEMETRY_FIELDS_V5
    values = {name: 0 for name in names}
    values.update(t_ms=t_ms, d100=250, imu_ok=1, duty=307, mstate=mstate, rst=36, s_pct=-40, s_us=1380, sstate=1,
                  vbat=7921, bstate=1, vdda=3299, vset=vset, vest=vest, ctl=ctl, vok=vok, vi=vi, stall_n=stall_n,
                  enc_bad=enc_bad, mfault=mfault)
    return (' '.join(f'{name}={values[name]}' for name in names) + '\r\n').encode()


def test_speed_status_text():
    assert motor_console.speed_status_text(v5_line(100)) == 'run vset=50 vest=49 vi=307'
    assert motor_console.speed_status_text(v5_line(100, vset=-50, vest=12, ctl=3, vi=0)) == 'GATE vset=-50 vest=12 vi=0'
    assert motor_console.speed_status_text(v5_line(100, vset=0, vest=0, ctl=2, vi=0)) == 'stop vset=0 vest=0 vi=0'
    assert motor_console.speed_status_text(v5_line(100, vset=0, vest=92, ctl=0, vi=0)) == 'open vest=92'
    assert motor_console.speed_status_text(v5_line(100, ctl=0, vest=0, vok=0)) == 'open vest=0 EST BAD'
    assert (motor_console.speed_status_text(v5_line(100, vok=0, stall_n=2, enc_bad=1))
            == 'run vset=50 vest=49 vi=307 EST BAD stall=2 enc_bad=1')
    assert motor_console.speed_status_text(v5_line(100, ctl=9)) == 'CTL 9? vset=50 vest=49 vi=307'
    assert motor_console.speed_status_text(v4_line(100)) == ''          # no speed part before v5
    assert motor_console.speed_status_text(None) == ''


def test_battery_status_is_read_from_v5_lines_too():
    assert motor_console.battery_status_text(v5_line(100)) == 'vbat=7.92V bst=1'


def test_motor_fault_text():
    assert motor_console.motor_fault_text(v5_line(100)) == ''
    assert motor_console.motor_fault_text(v5_line(100, mfault=7, mstate=3)) == 'MFAULT 7 REPEATED STALL'
    assert motor_console.motor_fault_text(v5_line(100, mfault=4, mstate=3)) == 'MFAULT 4 LOW BATTERY'
    assert motor_console.motor_fault_text(v5_line(100, mfault=1, mstate=3)) == 'MFAULT 1 PWM PERIOD'
    assert motor_console.motor_fault_text(v5_line(100, mfault=42, mstate=3)) == 'MFAULT 42 ?'
    assert motor_console.motor_fault_text(v4_line(100)) == ''


def test_speed_and_fault_are_shown_and_kept_in_a_narrow_window():
    logger = logger_with(v5_line(100, stall_n=3, mfault=7, mstate=3))
    console = motor_console.CommandConsole()
    full = motor_console.status_line_text(logger.status_text(), console, logger, 1000)
    assert (' | MFAULT 7 REPEATED STALL | duty=307 mstate=3 d100=250 | run vset=50 vest=49 vi=307 stall=3'
            ' | s=-40 us=1380 sst=1 | vbat=7.92V bst=1 | rst=36 (IWDG+PIN) | > ') in full
    essentials = ('MFAULT 7 REPEATED STALL | duty=307 mstate=3 d100=250 | run vset=50 vest=49 vi=307 stall=3'
                  ' | s=-40 us=1380 sst=1 | vbat=7.92V bst=1 | > ')
    assert motor_console.status_line_text(logger.status_text(), console, logger, len(essentials)) == essentials


# --- Text mode on a truck build (BENCH_TEXT_COMMANDS 0) -------------------------------

def counted_line(t_ms, cmd_ok, cmd_bad):
    values = {name: 0 for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS}
    values.update(t_ms=t_ms, imu_ok=1, mstate=2, cmd_ok=cmd_ok, cmd_bad=cmd_bad)
    return (' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS) + '\r\n').encode()


def lines_every_100_ms(count, cmd_ok, cmd_bad, start=0.05):
    """(time, line) pairs; cmd_ok(n) / cmd_bad(n) give the counters of the n-th line."""
    return [(start + 0.1 * n, counted_line(100 * n, cmd_ok(n), cmd_bad(n))) for n in range(count)]


def test_truck_build_text_mode_is_explained_once():
    # Every keepalive is counted as rejected and none as valid: one warning.
    received = lines_every_100_ms(30, cmd_ok=lambda n: 0, cmd_bad=lambda n: max(0, n - 3))
    result = run(typed=[(0.3, 'm 0\r')], received=received, stop_after=3.0)
    assert result['screen'].count('looks like a truck build (BENCH_TEXT_COMMANDS 0)') == 1
    assert 'motor_console.py --binary' in result['screen']


def test_no_warning_when_the_board_acts_on_commands():
    received = lines_every_100_ms(30, cmd_ok=lambda n: max(0, n - 3), cmd_bad=lambda n: 0)
    result = run(typed=[(0.3, 'm 0\r')], received=received, stop_after=3.0)
    assert 'truck build' not in result['screen']
    # Some lines rejected too (a typo, a raw test line), but valid ones counted: a bench build.
    received = lines_every_100_ms(30, cmd_ok=lambda n: max(0, n - 3), cmd_bad=lambda n: n)
    result = run(typed=[(0.3, 'm 0\r')], received=received, stop_after=3.0)
    assert 'truck build' not in result['screen']


def test_no_warning_for_steering_only_or_for_lines_sent_as_is():
    # "s" counts in s_ok, never in cmd_ok: on a bench build cmd_bad stays put, so no warning.
    received = lines_every_100_ms(30, cmd_ok=lambda n: 0, cmd_bad=lambda n: 0)
    assert 'truck build' not in run(typed=[(0.3, 's 20\r')], received=received, stop_after=3.0)['screen']
    # Testing rejection with !text (no valid command typed) is not a truck build.
    received = lines_every_100_ms(30, cmd_ok=lambda n: 0, cmd_bad=lambda n: n // 3)
    typed = [(0.3 + 0.3 * k, '!x 20\r') for k in range(8)]
    assert 'truck build' not in run(typed=typed, received=received, stop_after=3.0)['screen']


def test_text_gate_watch_unit_and_reset_on_boot():
    console = motor_console.CommandConsole()
    console.check_text_gate(counted_line(100, 0, 0))
    assert console.gate_baseline is None                      # nothing sent yet: not watching
    console.handle_key('m', 0.0), console.handle_key(' ', 0.0), console.handle_key('0', 0.0)
    console.handle_key(motor_console.ENTER, 0.0)
    console.check_text_gate(counted_line(200, 5, 7))            # baseline
    console.check_text_gate(counted_line(300, 5, 9))            # +2: not yet
    assert console.take_messages() == [] and not console.gate_settled
    console.check_text_gate(counted_line(400, 5, 10))           # +3: the warning
    assert 'cmd_bad +3' in console.take_messages()[-1] and console.gate_settled
    console.check_text_gate(counted_line(500, 5, 20))
    assert console.take_messages() == []                        # once
    console.handle_line_results(['boot'])                       # a reset starts the watch over
    assert not console.gate_settled and console.gate_baseline is None and not console.sent_valid_command


# --- Binary mode (--binary, the Pi link) ---------------------------------------------

import rover_link  # noqa: E402  (binary-mode tests only)

HELLO_ACK_FIELDS = {'build_flags': 0, 'reset_flags': 4, 'imu_ok': True, 'imu_who': 0x47, 'state_period_ms': 10,
                    'status_period_ms': 100}


def hello_ack_frame(build_flags=0):
    return rover_link.encode_frame(rover_link.HELLO_ACK, 0,
                                   rover_link.pack_hello_ack(dict(HELLO_ACK_FIELDS, build_flags=build_flags)))


def decoded(writes):
    """(type, seq, payload) of every frame written, text writes as ('text', bytes)."""
    decoder = rover_link.FrameDecoder()
    decoder.feed(b'\x00')
    out = []
    for data in writes:
        if not data.startswith(b'\x00'):
            out.append(('text', data))
            continue
        for status, frame, _ in decoder.feed(data):
            assert status == rover_link.OK, status
            out.append((frame.type, frame.seq, frame.payload))
    return out


def acknowledged_console(now=10.0):
    console = motor_console.BinaryConsole()
    assert decoded(console.keepalive_due(now))[0][0] == rover_link.HELLO
    console.handle_events([('frame', 'HELLO_ACK', 0, rover_link.unpack_hello_ack(rover_link.pack_hello_ack(
        HELLO_ACK_FIELDS)))], now)
    assert console.acknowledged
    console.take_messages()
    return console


def type_line(console, text, now):
    sent = []
    for key in text + motor_console.ENTER:
        sent += console.handle_key(key, now)
    return decoded(sent)


def drive_fields(frame):
    assert frame[0] == rover_link.DRIVE and rover_link.drive_check(frame[2]) == 'OK'
    return rover_link.unpack_drive(frame[2])


def test_binary_console_hello_then_a_50_hz_drive_keepalive_then_estop_on_exit():
    clock = FakeClock()
    start = clock.monotonic_seconds
    port = FakePort(clock, received=[(start + 0.3, hello_ack_frame())], stop_at=start + 1.5)
    keyboard = FakeKeyboard(clock, [(start + 0.1, 'v 2500\r')])     # typed before the board answered
    logger = serial_logger.BinaryLogger(io.BytesIO(), io.StringIO(), print_function=lambda text: None)
    status_line = motor_console.StatusLine(lambda text: None, columns=lambda: 60)
    motor_console.run_binary_console(port, keyboard, clock, logger, status_line, sleep=lambda seconds: None)
    frames = decoded(port.sent())
    types = [frame[0] for frame in frames]
    assert types[0] == rover_link.HELLO and types.count(rover_link.HELLO) == 1    # answered within 0.5 s
    assert types[-3:] == [rover_link.ESTOP] * 3 and port.closed
    drives = [(when, data) for when, data in port.writes if decoded([data])[0][0] == rover_link.DRIVE]
    assert all(drive_fields(decoded([data])[0]) == {'speed_cps': 2500, 'steer_percent': 0, 'steer_valid': False,
                                                    'arm': False} for _, data in drives)
    assert drives[0][0] >= start + 0.3                                   # none before the HELLO_ACK
    span = drives[-1][0] - drives[0][0]
    assert abs(len(drives) - (span / motor_console.DRIVE_PERIOD_SECONDS + 1)) <= 1    # 50 Hz
    assert [frame[1] for frame in frames] == list(range(len(frames)))    # one seq for every frame


def test_binary_console_commands():
    now = 10.0
    console = acknowledged_console(now)
    [frame] = type_line(console, 'v 1500', now)
    assert drive_fields(frame) == {'speed_cps': 1500, 'steer_percent': 0, 'steer_valid': False, 'arm': False}
    [frame] = type_line(console, 's -30', now)
    assert drive_fields(frame) == {'speed_cps': 1500, 'steer_percent': -30, 'steer_valid': True, 'arm': False}
    # arm: one DRIVE 0 with the arm bit, then the keepalive goes on at 0 without it.
    [frame] = type_line(console, 'arm', now)
    assert drive_fields(frame) == {'speed_cps': 0, 'steer_percent': -30, 'steer_valid': True, 'arm': True}
    [frame] = decoded(console.keepalive_due(now + 1.0))
    assert drive_fields(frame)['speed_cps'] == 0 and not drive_fields(frame)['arm']
    # Refused: m, out of range, nonsense; nothing sent.
    for text, why in (('m 20', 'no "m" in binary mode'), ('v 5001', 'beyond +-5000'), ('s 101', 'beyond +-100'),
                      ('drive', 'not sent')):
        assert type_line(console, text, now) == [] and why in console.take_messages()[-1], text
    # Esc: ESTOP, then DRIVE 0 with the steering centred.
    console.handle_key(motor_console.ESC, now)
    sent = decoded(console.handle_key(motor_console.ESC, now))
    assert sent[0][0] == rover_link.ESTOP and sent[0][2] == b''
    assert drive_fields(sent[1]) == {'speed_cps': 0, 'steer_percent': 0, 'steer_valid': True, 'arm': False}
    # -s: DRIVEs without steer_valid; -v: no DRIVEs at all.
    type_line(console, '-s', now)
    [frame] = decoded(console.keepalive_due(now + 2.0))
    assert not drive_fields(frame)['steer_valid']
    type_line(console, '-v', now)
    assert console.keepalive_due(now + 3.0) == []
    # !text goes out as text.
    assert type_line(console, '!m 20', now) == [('text', b'm 20\n')]
    assert console.take_messages()[-1].endswith('(binary mode ignores text, except e, c and h in a test build)')


@pytest.mark.parametrize('kind, outcome', [
    ('crc', 'CRC_ERROR'), ('cobs', 'COBS_ERROR'), ('length', 'BAD_LENGTH'), ('type', 'UNKNOWN_TYPE'),
    ('reserved', 'BAD_FLAGS'), ('range', 'OUT_OF_RANGE'), ('arm', 'ARM_WITH_SPEED')])
def test_injected_frames_are_each_dropped_for_their_own_reason(kind, outcome):
    console = acknowledged_console()
    frames = []
    for key in f'inject {kind} 4' + motor_console.ENTER:
        frames += console.handle_key(key, 10.0)
    assert len(frames) == 4
    assert motor_console.INJECT_EXPECTED[kind].replace('N', '4') in console.take_messages()[-1]
    for wire in frames:
        assert wire[0] == 0 and wire[-1] == 0 and 0 not in wire[1:-1]
        try:
            frame = rover_link.decode_frame(wire[1:-1])
        except rover_link.FrameError as error:
            assert error.status == outcome
            continue
        result = 'UNKNOWN_TYPE' if frame.type != rover_link.DRIVE else rover_link.drive_check(frame.payload)
        assert result == outcome


def test_binary_console_before_hello_ack_and_after_a_reset():
    console = motor_console.BinaryConsole()
    assert type_line(console, 'v 2000', 10.0) == []
    assert 'no HELLO_ACK yet' in console.take_messages()[-1]
    assert [frame[0] for frame in decoded(console.keepalive_due(10.0))] == [rover_link.HELLO]
    console.handle_events([('frame', 'HELLO_ACK', 0, rover_link.unpack_hello_ack(rover_link.pack_hello_ack(
        HELLO_ACK_FIELDS)))], 10.2)
    assert [frame[0] for frame in decoded(console.keepalive_due(10.2))] == [rover_link.DRIVE]   # typed earlier
    console.handle_events([('boot',)], 11.0)
    assert not console.acknowledged and not console.sending
    assert console.keepalive_due(11.05) == []
    assert [frame[0] for frame in decoded(console.keepalive_due(11.1))] == [rover_link.HELLO]
    assert console.exit_frames() and decoded(console.exit_frames())[0][0] == rover_link.ESTOP


def binary_logger_with(state=None, status=None, build_flags=0):
    logger = serial_logger.BinaryLogger(None, None, print_function=lambda text: None)
    logger.hello_ack = rover_link.unpack_hello_ack(rover_link.pack_hello_ack(dict(HELLO_ACK_FIELDS,
                                                                                  build_flags=build_flags)))
    if state is not None:
        logger.last_state = {name: 0 for name in rover_link.STATE_FIELDS}
        logger.last_state.update(enc_flags=rover_link.ENC_VALID_NOW, imu_flags=rover_link.IMU_OK)
        logger.last_state.update(state)
    if status is not None:
        logger.last_status = {name: 0 for name in rover_link.STATUS_FIELDS}
        logger.last_status.update(battery_state=1, vbat_mv=7921)
        logger.last_status.update(status)
    return logger


def test_binary_status_line_decodes_state_and_status_and_fits_the_window():
    console = acknowledged_console()
    type_line(console, 'v 2500', 10.0)
    logger = binary_logger_with(state={'motor_state': 2, 'speed_cps': 2450, 'count': 12345, 'duty': 0},
                                status={'latch_reason': 6, 'setpoint_cps': 2500, 'rx_ok': 812, 'loop_us_max': 702,
                                        'locked_stalls': 1})
    wide = motor_console.binary_status_line_text(console, logger, 1000)
    for part in ('bin', 'sent: v 2500', 'LATCHED (ESTOP) duty=0 vest=2450 cnt=12345 ep=0', 'vset=2500',
                 'vbat=7.92V bst=1', 'stalls lock=1 sat=0', 'rx ok=812', 'loop=702us', '> '):
        assert part in wide, part
    for width in (40, 59, 80, 120):
        assert len(motor_console.binary_status_line_text(console, logger, width)) <= width
    fault = binary_logger_with(state={'motor_state': 3, 'enc_flags': 0, 'imu_flags': 0},
                               status={'fault_reason': 7}, build_flags=0x02)
    text = motor_console.binary_status_line_text(console, fault, 1000)
    assert 'FAULT (REPEATED STALL)' in text and 'EST BAD' in text and 'IMU FAIL' in text
    assert 'TEST BUILD SPEED_CAPTURE' in text
    assert 'HELLO..' in motor_console.binary_status_line_text(motor_console.BinaryConsole(), logger, 1000)


def test_binary_console_logs_the_raw_bytes_unchanged():
    clock = FakeClock()
    start = clock.monotonic_seconds
    received = hello_ack_frame() + rover_link.encode_frame(
        rover_link.STATE, 0, rover_link.pack_state(dict({name: 0 for name in rover_link.STATE_FIELDS}, t_ms=10)))
    port = FakePort(clock, received=[(start + 0.1, received[:9]), (start + 0.2, received[9:])], stop_at=start + 0.5)
    raw = io.BytesIO()
    logger = serial_logger.BinaryLogger(raw, io.StringIO(), print_function=lambda text: None)
    motor_console.run_binary_console(port, FakeKeyboard(clock, []), clock, logger,
                                     motor_console.StatusLine(lambda text: None, columns=lambda: 80),
                                     sleep=lambda seconds: None)
    assert raw.getvalue() == received and logger.type_counts == {'HELLO_ACK': 1, 'STATE': 1}


# --- Linux keyboard: decode_keys, PosixKeyboard (a real pty), ExitSignals ---------------

ESC, CTRL_C, ENTER, BACKSPACE = motor_console.ESC, motor_console.CTRL_C, motor_console.ENTER, motor_console.BACKSPACE


def test_decode_keys_lone_esc_letters_enter_backspace_ctrl_c():
    assert motor_console.decode_keys('\x1b') == ([ESC], '\x1b')          # pending: the rest may follow
    assert motor_console.decode_keys('v 50\r') == (['v', ' ', '5', '0', ENTER], '')
    assert motor_console.decode_keys('v 50\n') == (['v', ' ', '5', '0', ENTER], '')
    assert motor_console.decode_keys('m 29\x7f0\x08') == (['m', ' ', '2', '9', BACKSPACE, '0', BACKSPACE], '')
    assert motor_console.decode_keys('\x03') == ([CTRL_C], '')
    assert motor_console.decode_keys('') == ([], '')


def test_decode_keys_every_esc_sequence_is_one_esc():
    for sequence in ('\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D',      # arrows
                     '\x1b[H', '\x1b[F', '\x1b[1~', '\x1b[4~',    # Home/End
                     '\x1b[3~', '\x1b[15~', '\x1b[24~',           # Delete, F5, F12
                     '\x1bOP', '\x1bOA', '\x1b[1;5C',              # F1 (SS3), arrow (SS3), Ctrl+Right
                     '\x1bx', '\x1bv'):                            # Alt+x, Alt+v
        assert motor_console.decode_keys(sequence) == ([ESC], ''), repr(sequence)
        assert motor_console.decode_keys('v 5' + sequence + '0\r') == (['v', ' ', '5', ESC, '0', ENTER], ''), \
            repr(sequence)


def test_decode_keys_esc_never_swallows_esc_or_ctrl_c():
    assert motor_console.decode_keys('\x1b\x1b') == ([ESC, ESC], '\x1b')
    assert motor_console.decode_keys('\x1b[A\x1b[B') == ([ESC, ESC], '')
    assert motor_console.decode_keys('\x1b\x03') == ([ESC, CTRL_C], '')
    assert motor_console.decode_keys('\x1b\r') == ([ESC, ENTER], '')
    assert motor_console.decode_keys('\x1b[\x1b') == ([ESC, ESC], '\x1b')   # broken CSI: the next ESC still counts


def test_decode_keys_sequence_split_across_reads():
    keys, pending = motor_console.decode_keys('\x1b')
    assert (keys, pending) == ([ESC], '\x1b')
    assert motor_console.decode_keys('[A', pending) == ([], '')            # the ESC isn't given twice
    keys, pending = motor_console.decode_keys('\x1b[1')
    assert (keys, pending) == ([ESC], '\x1b[1')
    assert motor_console.decode_keys('5~v', pending) == (['v'], '')
    assert motor_console.decode_keys('x', '\x1b') == ([], '')              # Alt+x split
    assert motor_console.decode_keys('\x03', '\x1b') == ([CTRL_C], '')


def test_decode_keys_drops_the_msvcrt_prefixes_and_keeps_other_characters():
    assert motor_console.decode_keys('\xe0\x1b') == ([ESC], '\x1b')        # a prefix can't eat the Esc
    assert motor_console.decode_keys('\x00H') == (['H'], '')
    assert motor_console.decode_keys('é\t') == (['é', '\t'], '')  # handle_key ignores the tab


def test_decoded_keys_drive_the_binary_console_like_windows_keys():
    console = acknowledged_console()
    keys, _ = motor_console.decode_keys('v 25\x7f500\r')
    sent = decoded([data for key in keys for data in console.handle_key(key, 10.0)])
    assert drive_fields(sent[0])['speed_cps'] == 2500
    keys, _ = motor_console.decode_keys('\x1b[A')                          # an arrow: ESTOP
    assert decoded(console.handle_key(keys[0], 10.0))[0][0] == rover_link.ESTOP


posix_only = pytest.mark.skipif(sys.platform == 'win32', reason='termios and pty are POSIX only')


class PtyClock:
    def __init__(self):
        self.now = 50.0

    def monotonic(self):
        return self.now


@pytest.fixture
def pty_pair():
    import os
    import pty
    master, slave = pty.openpty()
    yield master, slave
    for fd in (master, slave):
        try:
            os.close(fd)
        except OSError:
            pass


def keys_from(keyboard):
    keys = []
    while keyboard.kbhit():
        keys.append(keyboard.getwch())
    return keys


@posix_only
def test_posix_keyboard_sets_and_restores_the_terminal(pty_pair):
    import termios
    master, slave = pty_pair
    before = termios.tcgetattr(slave)
    assert before[3] & termios.ECHO and before[3] & termios.ICANON and before[3] & termios.ISIG
    with motor_console.PosixKeyboard(slave) as keyboard:
        inside = termios.tcgetattr(slave)
        assert not inside[3] & (termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)
        assert not inside[0] & termios.IXON
        assert inside[1] == before[1]                                      # output processing unchanged
        assert inside[6][termios.VMIN] == 1 and inside[6][termios.VTIME] == 0
        assert keyboard.kbhit() is False
    assert termios.tcgetattr(slave) == before


@posix_only
def test_posix_keyboard_restores_the_terminal_after_an_exception(pty_pair):
    import termios
    master, slave = pty_pair
    before = termios.tcgetattr(slave)
    with pytest.raises(RuntimeError, match='serial port gone'):
        with motor_console.PosixKeyboard(slave):
            raise RuntimeError('serial port gone')
    assert termios.tcgetattr(slave) == before


@posix_only
def test_posix_keyboard_keys_through_a_real_pty(pty_pair):
    import os
    master, slave = pty_pair
    clock = PtyClock()
    with motor_console.PosixKeyboard(slave, clock=clock) as keyboard:
        os.write(master, b'v 50\r')
        assert keys_from(keyboard) == ['v', ' ', '5', '0', ENTER]       # CR, or LF after ICRNL: ENTER
        os.write(master, b'\x03')
        assert keys_from(keyboard) == [CTRL_C]                           # ISIG off: a key, not SIGINT
        os.write(master, b'\x1b[A')
        assert keys_from(keyboard) == [ESC]
        os.write(master, 'é'.encode()[:1])                          # half a UTF-8 character
        assert keys_from(keyboard) == []
        os.write(master, 'é'.encode()[1:])
        assert keys_from(keyboard) == ['é']
        # A split sequence: the tail within ESCAPE_TAIL_SECONDS is dropped...
        os.write(master, b'\x1b')
        assert keys_from(keyboard) == [ESC]
        clock.now += 0.01
        os.write(master, b'[B')
        assert keys_from(keyboard) == []
        # ...but after a pause, what follows a lone Esc is typed text.
        os.write(master, b'\x1b')
        assert keys_from(keyboard) == [ESC]
        clock.now += 1.0
        os.write(master, b'[s')
        assert keys_from(keyboard) == ['[', 's']


@posix_only
def test_posix_keyboard_hangup_is_one_ctrl_c(pty_pair):
    import os
    master, slave = pty_pair
    with motor_console.PosixKeyboard(slave) as keyboard:
        os.close(master)                                                  # the SSH session's end of the pty
        assert keys_from(keyboard) == [CTRL_C]
        assert keyboard.kbhit() is False                                  # once
    # and leaving the with block on a hung-up terminal didn't raise


def binary_run_on_pty(slave, on_read, stop_after=5.0):
    """run_binary_console with the real PosixKeyboard and ExitSignals, as main() sets them up."""
    clock = FakeClock()
    start = clock.monotonic_seconds
    # The HELLO goes out HELLO_START_DELAY_SECONDS (0.1 s) after the start; the board answers after it.
    port = FakePort(clock, received=[(start + 0.15, hello_ack_frame())], stop_at=start + stop_after,
                    on_read=lambda: on_read(clock.monotonic_seconds - start))
    logger = serial_logger.BinaryLogger(io.BytesIO(), io.StringIO(), print_function=lambda text: None)
    keyboard = motor_console.PosixKeyboard(slave)
    with motor_console.ExitSignals(keyboard.request_quit) as signals, keyboard:
        console = motor_console.run_binary_console(port, keyboard, clock, logger,
                                                   motor_console.StatusLine(lambda text: None, columns=lambda: 80),
                                                   sleep=lambda seconds: None)
    return port, console, signals, start


@posix_only
@pytest.mark.parametrize('name', motor_console.EXIT_SIGNAL_NAMES)
def test_sighup_and_sigterm_send_estop_x3_and_close(pty_pair, name):
    import os
    import signal
    import termios
    master, slave = pty_pair
    before = termios.tcgetattr(slave)
    handlers_before = {number: signal.getsignal(number) for number in (signal.SIGHUP, signal.SIGTERM)}
    os.write(master, b'arm\rv 2500\r')
    kills = []

    def on_read(elapsed):
        if elapsed >= 0.5 and not kills:
            kills.append(elapsed)
            os.kill(os.getpid(), getattr(signal, name))
            os.kill(os.getpid(), getattr(signal, name))                   # a second one is ignored

    port, console, signals, start = binary_run_on_pty(slave, on_read)
    assert signals.received == name and console.quit_requested
    frames = decoded(port.sent())
    assert rover_link.DRIVE in [frame[0] for frame in frames[:-3]]       # it was driving
    assert [frame[0] for frame in frames[-3:]] == [rover_link.ESTOP] * 3 and port.closed
    assert port.writes[-1][0] - start < 0.6                               # out within a loop pass, not at stop_at
    assert termios.tcgetattr(slave) == before
    assert {number: signal.getsignal(number) for number in handlers_before} == handlers_before


@posix_only
def test_terminal_hangup_sends_estop_x3_and_close(pty_pair):
    import os
    master, slave = pty_pair
    os.write(master, b'arm\rv 2500\r')
    closed = []

    def on_read(elapsed):
        if elapsed >= 0.5 and not closed:
            closed.append(elapsed)
            os.close(master)

    port, console, signals, start = binary_run_on_pty(slave, on_read)
    assert console.quit_requested and signals.received is None
    assert [frame[0] for frame in decoded(port.sent())[-3:]] == [rover_link.ESTOP] * 3 and port.closed
    assert port.writes[-1][0] - start < 0.6


@posix_only
def test_text_console_sigterm_sends_m0_x3(pty_pair):
    import os
    import signal
    master, slave = pty_pair
    clock = FakeClock()
    start = clock.monotonic_seconds
    os.write(master, b'm 20\r')
    sent_kill = []

    def on_read():
        if clock.monotonic_seconds - start >= 0.5 and not sent_kill:
            sent_kill.append(True)
            os.kill(os.getpid(), signal.SIGTERM)

    port = FakePort(clock, stop_at=start + 5.0, on_read=on_read)
    logger = TelemetryLogger(io.BytesIO(), io.StringIO(), print_function=lambda text: None)
    keyboard = motor_console.PosixKeyboard(slave)
    with motor_console.ExitSignals(keyboard.request_quit), keyboard:
        motor_console.run_console(port, keyboard, clock, logger,
                                  motor_console.StatusLine(lambda text: None, columns=lambda: 80),
                                  sleep=lambda seconds: None)
    assert port.sent()[0] == b'm 20\n' and port.sent()[-3:] == EXIT_STOPS and port.closed
    assert port.writes[-1][0] - start < 0.6


# --- Exit drain, and stale data before the HELLO_ACK ---------------------------------------

def state_bytes(seq, t_ms, motor_state, speed_cps=0):
    fields = dict({name: 0 for name in rover_link.STATE_FIELDS}, t_ms=t_ms, motor_state=motor_state,
                  speed_cps=speed_cps)
    return rover_link.encode_frame(rover_link.STATE, seq & 0xFF, rover_link.pack_state(fields))


def binary_run(received, typed, stop_after=5.0, error_after=None, screen=None):
    clock = FakeClock()
    start = clock.monotonic_seconds
    port = FakePort(clock, [(start + when, data) for when, data in received], stop_at=start + stop_after,
                    error_at=None if error_after is None else start + error_after)
    keyboard = FakeKeyboard(clock, [(start + when, text) for when, text in typed])
    raw = io.BytesIO()
    logger = serial_logger.BinaryLogger(raw, io.StringIO(), print_function=lambda text: None)
    status_line = motor_console.StatusLine((screen or io.StringIO()).write, columns=lambda: 400)
    console = motor_console.run_binary_console(port, keyboard, clock, logger, status_line, sleep=lambda seconds: None)
    return port, console, logger, start, clock, raw


def board_stream(latch_at, until=1.5):
    """HELLO_ACK at 0.15 s, then STATE every 10 ms: RUNNING until latch_at, LATCHED after."""
    received = [(0.15, hello_ack_frame())]
    for n in range(int(0.16 * 100), int(until * 100)):
        received.append((n / 100, state_bytes(n, 10 * n, 2 if n / 100 >= latch_at else 1, 2500)))
    return received


def test_binary_exit_keeps_logging_after_the_estops_so_the_log_shows_the_latch():
    received = board_stream(latch_at=0.61)
    port, console, logger, start, clock, raw = binary_run(received, [(0.3, 'arm\rv 2500\r'), (0.6, CTRL_C)])
    frames = decoded(port.sent())
    assert [frame[0] for frame in frames[-3:]] == [rover_link.ESTOP] * 3 and port.closed
    estop_time = port.writes[-1][0]
    assert all(when == estop_time for when, _ in port.writes[-3:])        # the stops aren't held up
    assert logger.last_state['motor_state'] == 2                            # the latch is in the log
    logged_until = logger.last_state['t_ms'] / 1000
    assert estop_time - start + 0.15 <= logged_until <= estop_time - start + 0.23   # 0.2 s + one read
    assert clock.monotonic_seconds - estop_time <= motor_console.EXIT_DRAIN_SECONDS + 0.021   # bounded


def test_exit_drain_survives_a_failing_port_and_still_closes():
    received = board_stream(latch_at=0.61)
    port, console, logger, start, clock, raw = binary_run(received, [(0.3, 'v 2500\r'), (0.6, CTRL_C)],
                                                           error_after=0.65)
    assert [frame[0] for frame in decoded(port.sent())[-3:]] == [rover_link.ESTOP] * 3 and port.closed
    assert console.quit_requested


def test_exit_drain_ends_at_a_second_ctrl_c_and_prints_nothing():
    clock = FakeClock()
    port = FakePort(clock, [(clock.monotonic_seconds + 0.05, b'garbage line\r\n')], stop_at=clock.monotonic_seconds)
    printed = []
    logger = TelemetryLogger(io.BytesIO(), io.StringIO(), print_function=printed.append)
    motor_console.drain_after_exit(port, clock, logger)              # KeyboardInterrupt at the first read
    assert logger.print_function == printed.append and printed == []
    port = FakePort(clock, [(clock.monotonic_seconds + 0.05, b'first line, discarded\r\ngarbage line\r\n')])
    motor_console.drain_after_exit(port, clock, logger)
    assert logger.counts['bad'] == 1 and printed == []               # logged, not printed
    assert logger.print_function == printed.append


def test_text_exit_keeps_logging_after_m0():
    received = [(0.1 + 0.05 * n, telemetry_line(100 + 50 * n, duty=300 if n < 10 else 0, mstate=1 if n < 10 else 2))
                for n in range(30)]
    result = run(typed=[(0.2, 'm 20\r'), (0.6, motor_console.CTRL_C)], received=received, stop_after=10.0)
    assert result['port'].sent()[-3:] == EXIT_STOPS
    assert b' mstate=2 ' in result['logger'].last_good_line                # the board's answer is logged
    # Stops at 0.62 s; the line at 0.80 s is the last one read (the next comes at 0.85 s).
    assert result['raw'].endswith(telemetry_line(100 + 50 * 14, mstate=2))


def test_stale_frames_never_reach_the_status_line_and_a_leftover_ack_is_not_the_answer():
    leftover = b''.join(state_bytes(n, 900 + 10 * n, 1, speed_cps=4321) for n in range(5)) + hello_ack_frame()
    received = [(0.01, leftover), (0.2, hello_ack_frame())] + [
        (n / 100, state_bytes(n, 5000 + 10 * n, 2, speed_cps=0)) for n in range(21, 60)]
    screen = io.StringIO()
    port, console, logger, start, clock, raw = binary_run(received, [], stop_after=0.6, screen=screen)
    hellos = [when for when, data in port.writes if decoded([data])[0][0] == rover_link.HELLO]
    assert len(hellos) == 1 and hellos[0] - start >= serial_logger.HELLO_START_DELAY_SECONDS - 1e-9
    assert logger.stale_counts == {'frames': 6, 'bad': 0}
    assert '4321' not in screen.getvalue() and 'RUNNING' not in screen.getvalue()
    assert logger.last_state['t_ms'] >= 5210 and console.acknowledged
    assert raw.getvalue().startswith(leftover)                                # still in the raw file
