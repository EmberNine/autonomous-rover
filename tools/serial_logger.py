#!/usr/bin/env python3
"""Log the rover firmware's telemetry from a serial port, byte-for-byte.

Writes two files:
  <output>            the raw bytes exactly as received (nothing added, nothing removed)
  <output>.times.csv  one row per received line: host time, line number, byte offset, ok/bad

and keeps a running count of lines that pass a strict format check. Close any
other program that has the port open (e.g. the VS Code Serial Monitor) first:
only one program can hold a COM port at a time.

--binary (Pi link): sends HELLO (again every 0.5 s until a HELLO_ACK comes
back, and again after a board reset), then logs the binary frames: the raw
bytes as above, and <output>.times.csv gets one row per frame (host time,
piece number, OK or the reason it is bad, type, seq). HELLO latches the motor
and this never sends anything that re-arms it, so logging can't move the
truck. tools/link_dump.py turns the raw file into one CSV per frame type.
Frames received before the first HELLO_ACK are stale (BinaryLogger): left
over in the ST-LINK bridge from before the port was opened. They are kept in
the raw file and counted on their own, not as frames, bad pieces or gaps.
The first HELLO goes out HELLO_START_DELAY_SECONDS after opening, so that
leftover arrives before it.

The read loop never writes to the files itself: a writer thread does
(QueuedWriter), so a slow disk can't make it miss bytes. The status and the
summary show read_gap_max, the longest time between two reads (the buffers
before the port hold ~0.9 s). SIGHUP (the SSH session went away) and SIGTERM
(e.g. the Pi shutting down) stop it like Ctrl+C on Linux: the files are
written to the end and closed, and the summary is printed.

Usage:  py tools\\serial_logger.py COM5 logs\\gyro_static.log [--baud 115200] [--binary]
"""
import argparse
import collections
import contextlib
import re
import signal
import sys
import threading
import time

import rover_link

# Every field of a telemetry line, in the order the firmware prints them.
# Version 1: encoder + IMU (firmware before the motor driver).
TELEMETRY_FIELDS_V1 = (
    't_ms', 'raw', 'count', 'd100', 'MD', 'ML', 'MH', 'AGC', 'mag_min', 'mag_max',
    'i2c_err', 'overrun', 'resync', 'skipped', 'i2c_us_max', 'loop_us_max',
    'imu_ok', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp',
    'gsum_x', 'gsum_y', 'gsum_z', 'n_imu', 'imu_err', 'spi_us_max',
)
# Version 2 appends the motor fields; the v1 fields keep their positions.
MOTOR_FIELDS = (
    'cmd_pct', 'duty', 'mstate', 'wd_trip', 'cmd_ok', 'cmd_bad', 'cmd_clamp', 'rx_err',
)
TELEMETRY_FIELDS_V2 = TELEMETRY_FIELDS_V1 + MOTOR_FIELDS
# Version 3 appends the reset cause and the steering fields to a v2 line; the
# v1 and v2 fields keep their positions.
STEERING_FIELDS = ('rst', 's_pct', 's_us', 'sstate', 's_ok', 's_wd')
TELEMETRY_FIELDS_V3 = TELEMETRY_FIELDS_V2 + STEERING_FIELDS
# Version 4 appends the battery to a v3 line: filtered pack voltage (mV), the
# battery state (0 no battery, 1 OK, 2 low, 3 cutoff, 4 ADC fault, 5 removed
# while driving) and VDDA (mV).
BATTERY_FIELDS = ('vbat', 'bstate', 'vdda')
TELEMETRY_FIELDS_V4 = TELEMETRY_FIELDS_V3 + BATTERY_FIELDS
# Version 5 appends the closed-loop speed control to a v4 line: setpoint and
# 20 ms estimate (speed units, 1 = 50 counts/s), the controller state (0 open
# loop, 1 v running, 2 v stopped, 3 zero-speed gate), estimate valid (0/1),
# the integrator (ticks), stalls and bad-estimate latches since boot, and the
# motor's fault reason (0 = none). From v5 on, count and d100 are the
# corrected angle's count (the 4-per-revolution sensor error removed); raw
# stays the raw angle.
SPEED_FIELDS = ('vset', 'vest', 'ctl', 'vok', 'vi', 'stall_n', 'enc_bad', 'mfault')
TELEMETRY_FIELDS_V5 = TELEMETRY_FIELDS_V4 + SPEED_FIELDS


def _fields_pattern(names):
    return rb' '.join(name.encode() + rb'=-?\d+' for name in names)


# Exactly a v1, v2, v3, v4 or v5 line:
#   "t_ms=<int> ... spi_us_max=<int>[ cmd_pct=<int> ... rx_err=<int>[ rst=<int> ... s_wd=<int>
#    [ vbat=<int> bstate=<int> vdda=<int>[ vset=<int> ... mfault=<int>]]]]\r\n",
# single spaces, nothing else. Each appended part is all or nothing, and each
# only ever follows the one before it, so a line cut short inside any part
# is bad.
STRICT_LINE = re.compile(
    rb'^' + _fields_pattern(TELEMETRY_FIELDS_V1)
    + rb'(?: ' + _fields_pattern(MOTOR_FIELDS)
    + rb'(?: ' + _fields_pattern(STEERING_FIELDS)
    + rb'(?: ' + _fields_pattern(BATTERY_FIELDS)
    + rb'(?: ' + _fields_pattern(SPEED_FIELDS) + rb')?)?)?)?\r\n$'
)
BOOT_LINE = re.compile(rb'^imu_init=\w+ who=0x[0-9A-F]{2}\r\n$')
REPORT_PERIOD_MS = 100      # firmware sends one line every 100 ms
STATUS_EVERY_SECONDS = 2.0  # how often the running totals are printed
MAX_BAD_LINES_SHOWN = 5     # print the first few bad lines in full, to see what's wrong


def classify(line):
    """Return 'good', 'boot' or 'bad' for one complete line (ending in b'\\n')."""
    if STRICT_LINE.match(line):
        return 'good'
    if BOOT_LINE.match(line):
        return 'boot'
    return 'bad'


# --- Keeping the read loop free of disk I/O ----------------------------------------------
#
# 2026-10-16, b5_session_vbat_1 (serial_logger --binary on the Pi during a docker
# pull onto the same SD card): the 2 s flush blocked the read loop for 2.5 s and
# 1.0 s. The buffers before the port hold ~0.93 s (~5.8 KB at ~6.1 KB/s), so
# 1.6 s and 0.1 s of frames were lost (a cut frame, seq and t_ms gaps). So the
# loops never touch a file: QueuedWriter takes the writes and a thread of its
# own does them.

WRITER_QUEUE_LIMIT = 32 * 1024 * 1024   # bytes (or characters) waiting: ~90 min of the binary stream
_FLUSH = object()


class QueuedWriter:
    """A file whose write() and flush() never block the caller: they queue, and
    a writer thread does the real writes and flushes, in order, so the file
    ends up exactly as with direct writes. If more than limit is waiting (the
    disk stalled for a very long time) a write is dropped and counted
    (dropped, in bytes or characters), never waited for. A failing write (disk
    full) is kept in error and everything after it is dropped and counted.
    finish() waits until everything queued is written and flushed (the file
    itself is closed by its owner). Works for binary and text files alike."""

    def __init__(self, file, limit=WRITER_QUEUE_LIMIT):
        self.file = file
        self.limit = limit
        self.queue = collections.deque()
        self.queued = 0
        self.dropped = 0
        self.error = None
        self.finishing = False
        self.condition = threading.Condition()
        self.thread = threading.Thread(target=self._run, name='log-writer', daemon=True)
        self.thread.start()

    def write(self, data):
        with self.condition:
            if self.error is not None or self.finishing or self.queued + len(data) > self.limit:
                self.dropped += len(data)
            else:
                self.queue.append(data)
                self.queued += len(data)
                self.condition.notify()
        return len(data)

    def flush(self):
        with self.condition:
            self.queue.append(_FLUSH)
            self.condition.notify()

    def _run(self):
        while True:
            with self.condition:
                while not self.queue and not self.finishing:
                    self.condition.wait()
                if not self.queue:
                    return
                items = list(self.queue)
                self.queue.clear()
            for item in items:          # the file I/O, outside the lock: write() never waits for it
                try:
                    if item is _FLUSH:
                        self.file.flush()
                    elif self.error is None:
                        self.file.write(item)
                    else:
                        with self.condition:
                            self.dropped += len(item)
                except Exception as error:
                    with self.condition:
                        self.error = error
                        if item is not _FLUSH:
                            self.dropped += len(item)
                if item is not _FLUSH:
                    with self.condition:
                        self.queued -= len(item)

    def finish(self):
        with self.condition:
            self.queue.append(_FLUSH)
            self.finishing = True
            self.condition.notify()
        self.thread.join()

    def problem_text(self):
        """'' when every write made it to the file, else what went wrong."""
        if self.error is None and not self.dropped:
            return ''
        what = f'failed ({self.error})' if self.error is not None else 'fell behind'
        return f'{what}: {self.dropped} bytes not written'


@contextlib.contextmanager
def queued_writer(file):
    writer = QueuedWriter(file)
    try:
        yield writer
    finally:
        writer.finish()


def print_writer_problems(writers):
    for name, writer in writers:
        if writer.problem_text():
            print(f'WARNING: writing {name} {writer.problem_text()}; that file is incomplete.')


class ReadGapMeter:
    """The longest time between two reads (monotonic times), shown as ' read_gap_max=..ms'
    once at least two reads were timed; '' before (and in offline use, e.g. link_dump)."""

    def __init__(self):
        self.last = None
        self.longest = None

    def note(self, read_at):
        if read_at is None:
            return
        if self.last is not None:
            gap = read_at - self.last
            self.longest = gap if self.longest is None else max(self.longest, gap)
        self.last = read_at

    def text(self):
        return '' if self.longest is None else f' read_gap_max={self.longest * 1000:.0f}ms'


# --- Stopping cleanly on SIGHUP / SIGTERM (POSIX) ----------------------------------------

EXIT_SIGNAL_NAMES = ('SIGHUP', 'SIGTERM')    # SIGHUP: the SSH session (the terminal) went away


class ExitSignals:
    """Context manager (POSIX): SIGHUP and SIGTERM call on_signal instead of
    killing the process, so the loop leaves through its own way out (the files
    are written to the end and closed, motor_console sends its stops), as for
    Ctrl+C. The handler raises nothing, so a signal can't cut that short; the
    first one also ignores the rest. The previous handlers come back on the way
    out. received: the first signal's name. Used by serial_logger.py,
    tools/motor_console.py and the ROS bridge (with SIGINT too: names).
    A name the platform doesn't have is skipped."""

    def __init__(self, on_signal, names=EXIT_SIGNAL_NAMES):
        self.on_signal = on_signal
        self.names = names
        self.previous = {}
        self.received = None

    def __enter__(self):
        for name in self.names:
            number = getattr(signal, name, None)
            if number is not None:
                self.previous[number] = signal.signal(number, self._handle)
        return self

    def _handle(self, number, frame):
        for other in self.previous:
            signal.signal(other, signal.SIG_IGN)
        self.received = signal.Signals(number).name
        self.on_signal()

    def __exit__(self, *exception):
        for number, handler in self.previous.items():
            signal.signal(number, handler)
        self.previous = {}
        return False


def exit_signals(on_signal):
    """ExitSignals on POSIX; nothing on Windows (no SIGHUP there; Ctrl+C as before)."""
    return contextlib.nullcontext() if sys.platform == 'win32' else ExitSignals(on_signal)


class TelemetryLogger:
    """Everything the logger does with received bytes, without the serial port.

    feed() takes each chunk exactly as read from the port, writes it unchanged
    to raw_file, splits it into lines, classifies them, writes one row per line
    to times_file, and keeps the counts. tools/motor_console.py uses the same
    class, so its log files and status line are produced by the same code.
    """

    def __init__(self, raw_file, times_file, print_function=print):
        self.raw_file = raw_file
        self.times_file = times_file
        self.print_function = print_function
        self.counts = {'good': 0, 'bad': 0, 'boot': 0}
        self.gaps = 0              # good lines whose t_ms didn't follow the previous one by exactly 100
        self.previous_t_ms = None
        self.partial_line = b''
        self.byte_offset = 0       # where in the raw file the current line starts
        self.line_number = 0
        self.discarded_first = False
        self.last_good_line = None  # the most recent good line (bytes), for live displays
        self.read_gap = ReadGapMeter()
        self.times_file.write('host_time_unix,line,byte_offset,result\n')

    def feed(self, chunk, received_at, read_at=None):
        """Process one chunk. received_at: host time (time.time()) shared by all lines finished in it.
        read_at: monotonic time of the read (for read_gap_max); None in offline use.

        Returns the classification of each line completed in this chunk, in order.
        """
        self.read_gap.note(read_at)
        if chunk:
            self.raw_file.write(chunk)  # unchanged, before any parsing
        self.partial_line += chunk

        results = []
        while b'\n' in self.partial_line:
            line, self.partial_line = self.partial_line.split(b'\n', 1)
            line += b'\n'
            self.line_number += 1
            if not self.discarded_first:
                # Opening the port mid-line gives a truncated first line; don't count it.
                self.discarded_first = True
                result = 'first_partial' if not line.startswith((b't_ms=', b'imu_init=')) else classify(line)
            else:
                result = classify(line)

            if result in self.counts:
                self.counts[result] += 1
            if result == 'bad' and self.counts['bad'] <= MAX_BAD_LINES_SHOWN:
                self.print_function(f'\nBAD line {self.line_number}: {line!r}')
            if result == 'good':
                t_ms = int(line[5:line.index(b' ')])
                if self.previous_t_ms is not None and t_ms - self.previous_t_ms != REPORT_PERIOD_MS:
                    self.gaps += 1
                self.previous_t_ms = t_ms
                self.last_good_line = line
            if result == 'boot':
                self.previous_t_ms = None  # the board restarted; t_ms starts over

            self.times_file.write(f'{received_at:.6f},{self.line_number},{self.byte_offset},{result}\n')
            self.byte_offset += len(line)
            results.append(result)
        return results

    def flush(self):
        self.raw_file.flush()
        self.times_file.flush()

    def status_text(self):
        """The running totals, as printed every STATUS_EVERY_SECONDS (without the leading '\\r')."""
        total = self.counts['good'] + self.counts['bad']
        bad_percent = 100.0 * self.counts['bad'] / total if total else 0.0
        return (f"good={self.counts['good']} bad={self.counts['bad']} ({bad_percent:.2f}%) "
                f"boot={self.counts['boot']} t_ms_gaps={self.gaps}{self.read_gap.text()}")

    def summary_text(self):
        """The final line printed when logging stops."""
        total = self.counts['good'] + self.counts['bad']
        return (f"Stopped. {total} lines: good={self.counts['good']} bad={self.counts['bad']} "
                f"boot={self.counts['boot']} t_ms_gaps={self.gaps}{self.read_gap.text()}")

    def exit_code(self):
        return 0 if self.counts['bad'] == 0 else 1


# --- Binary mode (the Pi link, tools/rover_link.py) ------------------------------------
#
# The board boots in text mode. A HELLO frame switches it to binary mode for
# good (until a reset): from then on it sends STATE (100 Hz) and STATUS
# (10 Hz) frames instead of the text line, and ignores text. HELLO also
# latches the motor; only an arming DRIVE re-arms it, and the logger never
# sends one, so logging in binary mode can't move anything.

HELLO_RETRY_SECONDS = 0.5      # resend HELLO until a HELLO_ACK comes back
HELLO_AFTER_BOOT_SECONDS = 0.1  # after a boot line: give the firmware time to start receiving
# After opening the port, wait this long before the first HELLO while
# reading (and logging, as stale) what arrives: the ST-LINK bridge delivers
# the bytes it held while the port was closed (~1-2 KB, possibly minutes
# old, valid CRCs) at once, so a stale HELLO_ACK among them arrives before
# this session's HELLO goes out and can't be taken for its answer.
HELLO_START_DELAY_SECONDS = 0.1
# A boot line can follow the tail of a frame without a newline in between,
# so it is searched for at the end of a received line, not matched from its start.
BOOT_LINE_AT_END = re.compile(rb'imu_init=\w+ who=0x[0-9A-F]{2}\r\n$')
MAX_TEXT_LINE = 1100           # longest text line kept while looking for boot / v5 lines
STATE_PERIOD_MS = 10


class Sequence:
    """The Pi -> Nucleo seq: one counter for every frame sent (the firmware
    counts gaps across all of them)."""

    def __init__(self):
        self.value = 0

    def next(self):
        value = self.value
        self.value = (self.value + 1) & 0xFF
        return value


class HelloSender:
    """Sends HELLO until a HELLO_ACK has come back: at once, then every
    HELLO_RETRY_SECONDS; again after a boot line or a v5 text line (the board
    restarted, or never switched). No I/O: due() returns the frames to send.
    first_delay: the first HELLO goes out this long after the first due()
    (HELLO_START_DELAY_SECONDS in the tools that open the port)."""

    def __init__(self, sequence, first_delay=0.0):
        self.sequence = sequence
        self.first_delay = first_delay
        self.pending = True
        self.next_time = None          # None: the first due() (sends at once without a first_delay)
        self.sent_count = 0

    def due(self, now):
        if self.next_time is None and self.first_delay > 0:
            self.next_time = now + self.first_delay
            self.first_delay = 0.0     # once: a later HELLO asked for at once goes at once
        if not self.pending or (self.next_time is not None and now < self.next_time):
            return []
        self.next_time = now + HELLO_RETRY_SECONDS
        self.sent_count += 1
        return [rover_link.encode_frame(rover_link.HELLO, self.sequence.next(), rover_link.pack_hello())]

    def acknowledged(self):
        self.pending = False

    def board_in_text_mode(self, now, delay=0.0):
        if not self.pending:
            self.pending = True
            self.next_time = now + delay


class BinaryLogger:
    """Everything the logger does with received bytes in binary mode, without
    the serial port: raw bytes unchanged to raw_file (None: not kept), one row
    per finished piece to times_file (None: not kept), counts, sequence and
    t_ms gaps, and the last HELLO_ACK / STATE / STATUS fields for live
    displays. Text lines between frames (a boot line, v5 lines from a board
    in text mode) are recognised too.

    feed() returns events in order: ('frame', name, seq, fields, piece)
    (piece: this frame's own number, as in times_file),
    ('bad', status), ('boot',), ('text',). A piece that held a recognised
    text line is logged as TEXT, not counted as bad (after a reset the board
    sends its boot line and v5 lines between the frames' delimiters).

    Stale data (stale_until_hello_ack, on except in unit tests that feed
    bare frames): every frame or bad piece before the first HELLO_ACK is
    stale. When the port is reopened while the board is in binary mode, the
    ST-LINK bridge first delivers the bytes it held while the port was
    closed (valid CRCs, possibly minutes old), spliced onto the live stream
    (one bad piece, a seq and a t_ms gap). Stale pieces are counted on their
    own (stale_counts, stale_bytes; result STALE in times_file, with the
    frame type or the reason it is bad), not as frames, bad pieces or gaps;
    they give no event and don't set last_state / last_status (nor the seq
    and t_ms checks, so the splice is no gap). Raw bytes are kept as always;
    text and boot lines are recognised as always.
    The loops that send HELLO call expect_hello() first and
    mark_hello_sent() as the first HELLO goes out: a HELLO_ACK received
    before that is stale too. Remaining risk: a stale HELLO_ACK that arrives
    after the first HELLO is taken for the answer."""

    def __init__(self, raw_file, times_file, print_function=print, stale_until_hello_ack=True):
        self.raw_file = raw_file
        self.times_file = times_file
        self.print_function = print_function
        self.decoder = rover_link.FrameDecoder()
        self.text_line = bytearray()
        self.counts = {'frames': 0, 'bad': 0, 'boot': 0, 'text': 0}
        self.type_counts = {}
        self.bad_counts = {}
        self.seq_gaps = {}              # per frame type: seq != previous + 1 (mod 256)
        self.previous_seq = {}
        self.t_ms_gaps = 0              # STATE t_ms steps other than STATE_PERIOD_MS
        self.previous_state_t_ms = None
        self.frame_number = 0
        self.piece_had_text = False   # a boot or v5 text line was recognised in the current piece
        self.hello_ack = None
        self.last_state = None
        self.last_status = None
        self.stale_mode = stale_until_hello_ack
        self.awaiting_hello_ack = stale_until_hello_ack
        self.hello_sent = True          # False between expect_hello() and mark_hello_sent()
        self.stale_counts = {'frames': 0, 'bad': 0}
        self.stale_bytes = 0            # the stale pieces' bytes, delimiters included
        self.read_gap = ReadGapMeter()
        if self.times_file is not None:
            self.times_file.write('host_time_unix,piece,result,type,seq\n')

    def expect_hello(self):
        """This session will send the HELLO: until mark_hello_sent(), a HELLO_ACK is stale too."""
        self.hello_sent = False

    def mark_hello_sent(self):
        """This session's first HELLO is about to go out: HELLO_ACKs from now on are answers."""
        self.hello_sent = True

    def feed(self, chunk, received_at, read_at=None):
        """read_at: monotonic time of the read (for read_gap_max); None in offline use (link_dump)."""
        self.read_gap.note(read_at)
        if chunk and self.raw_file is not None:
            self.raw_file.write(chunk)
        # Piece by piece (split after each 0x00), so text recognised inside a
        # piece (a boot line, v5 lines) marks that piece as text, not as a bad frame.
        events = []
        start = 0
        for position, byte in enumerate(chunk):
            if byte == 0:
                events += self._feed_segment(chunk[start:position + 1], received_at)
                start = position + 1
        if start < len(chunk):
            events += self._feed_segment(chunk[start:], received_at)
        return events

    def _feed_segment(self, segment, received_at):
        events = self._text_events(segment)
        for status, frame, raw in self.decoder.feed(segment):
            if status == rover_link.LEADING:
                continue
            self.frame_number += 1
            if status != rover_link.OK:
                if self.piece_had_text:
                    self._row(received_at, 'TEXT', '', '')
                    continue
                if self.awaiting_hello_ack:
                    self._stale(received_at, 'bad', status, '', raw)
                    continue
                self.counts['bad'] += 1
                self.bad_counts[status] = self.bad_counts.get(status, 0) + 1
                self._row(received_at, status, '', '')
                events.append(('bad', status))
                continue
            name, fields = rover_link.decode_message(frame)
            if self.awaiting_hello_ack:
                if name != 'HELLO_ACK' or not self.hello_sent:
                    self._stale(received_at, 'frames', name, frame.seq, raw)
                    continue
                self.awaiting_hello_ack = False   # the answer: live from here
            self.counts['frames'] += 1
            self.type_counts[name] = self.type_counts.get(name, 0) + 1
            previous = self.previous_seq.get(name)
            if previous is not None and frame.seq != (previous + 1) & 0xFF:
                self.seq_gaps[name] = self.seq_gaps.get(name, 0) + 1
            self.previous_seq[name] = frame.seq
            if name == 'HELLO_ACK':
                self.hello_ack = fields
            elif name == 'STATE' and fields is not None:
                step = (fields['t_ms'] - (self.previous_state_t_ms or 0)) & 0xFFFFFFFF
                if self.previous_state_t_ms is not None and step != STATE_PERIOD_MS:
                    self.t_ms_gaps += 1
                self.previous_state_t_ms = fields['t_ms']
                self.last_state = fields
            elif name == 'STATUS' and fields is not None:
                self.last_status = fields
            self._row(received_at, 'OK', name, frame.seq)
            events.append(('frame', name, frame.seq, fields, self.frame_number))
        if segment.endswith(b'\x00'):
            self.piece_had_text = False
        return events

    def _text_events(self, segment):
        events = []
        for byte in segment:
            if byte == 0:
                self.text_line.clear()      # a frame boundary: no text line runs across it
                continue
            self.text_line.append(byte)
            if byte == 0x0A:
                line = bytes(self.text_line)
                self.text_line.clear()
                if BOOT_LINE_AT_END.search(line):
                    self.counts['boot'] += 1
                    self.piece_had_text = True
                    self.previous_seq.clear()
                    self.previous_state_t_ms = None
                    events.append(('boot',))
                elif STRICT_LINE.match(line):
                    self.counts['text'] += 1
                    self.piece_had_text = True
                    events.append(('text',))
            elif len(self.text_line) > MAX_TEXT_LINE:
                self.text_line.clear()
        return events

    def _stale(self, received_at, kind, name, seq, raw):
        self.stale_counts[kind] += 1
        self.stale_bytes += len(raw) + 1
        self._row(received_at, 'STALE', name, seq)

    def stale_text(self):
        """'none', or the stale counts; says so when no HELLO_ACK has been accepted at all."""
        missing = ' (no HELLO_ACK in this log)' if self.stale_mode and self.awaiting_hello_ack else ''
        if not any(self.stale_counts.values()):
            return 'none' + missing
        return (f"frames={self.stale_counts['frames']} bad={self.stale_counts['bad']} "
                f'({self.stale_bytes} bytes){missing}')

    def _row(self, received_at, result, name, seq):
        if self.times_file is not None:
            self.times_file.write(f'{received_at:.6f},{self.frame_number},{result},{name},{seq}\n')

    def flush(self):
        for file in (self.raw_file, self.times_file):
            if file is not None:
                file.flush()

    def status_text(self):
        gaps = sum(self.seq_gaps.values())
        return (f"frames={self.counts['frames']} bad={self.counts['bad']} seq_gaps={gaps} "
                f"t_ms_gaps={self.t_ms_gaps} boot={self.counts['boot']} text={self.counts['text']}"
                f'{self.read_gap.text()}')

    def summary_text(self):
        types = ' '.join(f'{name}={count}' for name, count in sorted(self.type_counts.items())) or 'none'
        bad = ' '.join(f'{name}={count}' for name, count in sorted(self.bad_counts.items())) or 'none'
        gaps = ' '.join(f'{name}={count}' for name, count in sorted(self.seq_gaps.items())) or 'none'
        return (f'Stopped. frames: {types}; bad: {bad}; seq gaps: {gaps}; STATE t_ms gaps={self.t_ms_gaps}; '
                f"boot={self.counts['boot']} text lines={self.counts['text']}; "
                f'stale before HELLO_ACK: {self.stale_text()}'
                + (f';{self.read_gap.text()}' if self.read_gap.text() else ''))

    def exit_code(self):
        return 0 if self.counts['bad'] == 0 else 1


def handle_hello_events(events, hello, now, show):
    """What both binary tools do with the logger's events: a HELLO_ACK stops
    the HELLO retries (and is shown, loudly for a test build); a boot line or
    a v5 text line means the board is in text mode again: HELLO again."""
    for event in events:
        if event[0] == 'frame' and event[1] == 'HELLO_ACK' and event[3] is not None:
            hello.acknowledged()
            show(hello_ack_text(event[3]))
        elif event[0] == 'boot':
            hello.board_in_text_mode(now, HELLO_AFTER_BOOT_SECONDS)
            show('board reset: back in text mode, motor latched; sending HELLO again')
        elif event[0] == 'text':
            hello.board_in_text_mode(now)


def hello_ack_text(fields):
    flags = rover_link.build_flag_text(fields['build_flags'])
    build = f'TEST BUILD ({flags}): never drive the truck with it' if flags else 'normal build'
    return (f"HELLO_ACK: protocol {fields['version']}, {build}, rst={fields['reset_flags']}, "
            f"imu_ok={fields['imu_ok']} who=0x{fields['imu_who']:02X}, speed {fields['min_cps']}..{fields['max_cps']} "
            f"cps ({fields['cps_per_unit']} cps/unit), STATE every {fields['state_period_ms']} ms, "
            f"STATUS every {fields['status_period_ms']} ms")


def run_binary_logger(port, clock, logger, hello, show, stop=None):
    """The --binary loop: HELLO until acknowledged (again after a reset), log
    every frame, never send anything else. Returns on KeyboardInterrupt, or
    once stop (a threading.Event, set by ExitSignals) is set.
    port: pyserial-like (read, in_waiting, write, close); clock: time-like."""
    next_status_time = clock.monotonic() + STATUS_EVERY_SECONDS
    logger.expect_hello()
    try:
        while stop is None or not stop.is_set():
            now = clock.monotonic()
            for frame in hello.due(now):
                logger.mark_hello_sent()
                port.write(frame)
            chunk = port.read(port.in_waiting or 1)
            events = logger.feed(chunk, clock.time(), clock.monotonic())
            handle_hello_events(events, hello, clock.monotonic(), show)
            if clock.monotonic() >= next_status_time:
                logger.flush()
                show(logger.status_text())
                next_status_time += STATUS_EVERY_SECONDS
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='serial port, e.g. COM5')
    parser.add_argument('output', help='file for the raw bytes; timestamps go to <output>.times.csv')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--binary', action='store_true',
                        help='Pi link binary mode: send HELLO (motor stays latched), log frames')
    args = parser.parse_args()

    import serial  # imported here so the line checker above can be used without pyserial

    if args.binary:
        return main_binary(serial.Serial(args.port, args.baud, timeout=0.1), args.output)

    # timeout=0.1 makes read() return at least every 100 ms, so Ctrl+C is noticed promptly.
    port = serial.Serial(args.port, args.baud, timeout=0.1)
    next_status_time = time.monotonic() + STATUS_EVERY_SECONDS
    stop = threading.Event()

    with exit_signals(stop.set) as signals, \
         open(args.output, 'wb') as raw_file, \
         open(args.output + '.times.csv', 'w', encoding='ascii', newline='') as times_file, \
         queued_writer(raw_file) as raw, queued_writer(times_file) as times:
        logger = TelemetryLogger(raw, times)
        try:
            while not stop.is_set():
                chunk = port.read(port.in_waiting or 1)
                received_at = time.time()  # all lines finished in this chunk get the same host time
                read_at = time.monotonic()
                logger.feed(chunk, received_at, read_at)

                if read_at >= next_status_time:
                    logger.flush()
                    print(f'\r{logger.status_text()}   ', end='', flush=True)
                    next_status_time += STATUS_EVERY_SECONDS
        except KeyboardInterrupt:
            pass
        finally:
            port.close()

    return print_end(logger, signals, (('the raw log', raw), ('the times file', times)))


def main_binary(port, output):
    stop = threading.Event()
    with exit_signals(stop.set) as signals, \
         open(output, 'wb') as raw_file, open(output + '.times.csv', 'w', encoding='ascii', newline='') as times_file, \
         queued_writer(raw_file) as raw, queued_writer(times_file) as times:
        logger = BinaryLogger(raw, times)
        hello = HelloSender(Sequence(), first_delay=HELLO_START_DELAY_SECONDS)
        run_binary_logger(port, time, logger, hello, lambda text: print(f'\r{text}   ', flush=True), stop)
    return print_end(logger, signals, (('the raw log', raw), ('the times file', times)))


def print_end(logger, signals, writers):
    """After the files are written to the end and closed: the signal (if one
    stopped it), the summary, any write problem. A terminal that is gone (the
    SSH session dropped) is ignored: the files are complete anyway. Exit code:
    the logger's, or 1 if a file is incomplete."""
    try:
        received = getattr(signals, 'received', None)
        if received:
            print(f'\n{received}: stopped as for Ctrl+C.', end='')
        print(f'\n{logger.summary_text()}')
        print_writer_problems(writers)
    except OSError:
        pass
    return 1 if any(writer.problem_text() for _, writer in writers) else logger.exit_code()


if __name__ == '__main__':
    sys.exit(main())
