"""The bridge's serial link: a reader thread and a TX thread. No ROS.

The TX thread is the only writer to the port. Every TX_PERIOD_SECONDS (50 Hz)
it sends any HELLO due (serial_logger.HelloSender: 0.1 s after the start,
again every 0.5 s until a HELLO_ACK, again after a board reset), then the
frames tx_source(now) asks for: safety.Safety.tick, i.e. DRIVE while armed,
ESTOP x3 when it says so, nothing otherwise. One Sequence for all of them
(the firmware counts seq gaps across every frame). Without a tx_source only
HELLO goes out (the read-only bridge of groups 1-2).

Received bytes go through serial_logger.BinaryLogger with its stale rule:
nothing before this session's first HELLO_ACK is published, and a
HELLO_ACK received before the first HELLO went out is stale too.

The reader thread only reads, decodes and queues: no file, no print, no
ROS call, so a slow disk or a busy publisher can't hold it up (the
b5_session_vbat_1 log lost 1.6 s of frames when serial_logger's own flush
blocked its read loop for 2.5 s; ~0.93 s of data fits in the buffers before
the port). Decoded frames wait in a bounded queue for the ROS side; if that
falls 10 s behind, the oldest are dropped and counted.

A port that fails (unplugged: pyserial raises) sets failed; the node then
exits (once the board is back, if it was told its USB serial number:
usb_serial_tty), and the container's restart policy brings it back.
"""
import collections
import os
import threading
import time

from rover_bridge import _tools  # noqa: F401  (tools/ on sys.path)
import rover_link  # noqa: E402
from serial_logger import HELLO_START_DELAY_SECONDS, BinaryLogger, HelloSender, Sequence, handle_hello_events  # noqa: E402

EVENT_QUEUE_LENGTH = 1100        # 10 s of STATE + STATUS
TX_PERIOD_SECONDS = 0.02         # 50 Hz: 25 DRIVEs per firmware watchdog timeout (500 ms)
SYSFS_TTY = '/sys/class/tty'


def usb_serial_tty(serial, sysfs_tty=SYSFS_TTY):
    """The tty (e.g. 'ttyACM0') of a USB device with this serial number, or None.
    Reads sysfs: /sys/class/tty/<tty>/device is the USB interface, whose parent
    directory (the USB device) has the 'serial' file. Containers see the host's
    sysfs (read-only), so this tells from inside one whether the board is back."""
    try:
        names = sorted(os.listdir(sysfs_tty))
    except OSError:
        return None
    for name in names:
        interface = os.path.realpath(os.path.join(sysfs_tty, name, 'device'))
        try:
            with open(os.path.join(os.path.dirname(interface), 'serial')) as file:
                if file.read().strip() == serial:
                    return name
        except OSError:
            continue
    return None


class Link:
    """port: pyserial-like (read, in_waiting, write, close), opened with a read
    timeout (READ_TIMEOUT in node.py). clock: time-like (monotonic, time).
    events: (received_at_unix, name, seq, fields, read_at_monotonic) of every
    live frame (and ('boot', ...) for a board reset), oldest first; notes:
    lines for the log (HELLO_ACK, board reset, ...)."""

    def __init__(self, port, clock=time, hello_delay=HELLO_START_DELAY_SECONDS, raw_file=None, times_file=None,
                 tx_source=None):
        """raw_file / times_file: where BinaryLogger writes the raw bytes and the times rows, or None.
        They must not block the reader: pass serial_logger.QueuedWriter objects.
        tx_source(now): the frames to send besides HELLO, as ('DRIVE', speed_cps, steer_percent,
        arm) / ('ESTOP',) tuples (safety.Safety.tick), or None (HELLO only)."""
        self.port = port
        self.clock = clock
        self.tx_source = tx_source
        self.notes = collections.deque(maxlen=200)
        self.logger = BinaryLogger(raw_file, times_file, print_function=self.notes.append)
        self.logger.expect_hello()
        self.sequence = Sequence()
        self.hello = HelloSender(self.sequence, first_delay=hello_delay)
        self.sent = collections.Counter()      # frames written, by type name
        self.hello_lock = threading.Lock()
        self.events = collections.deque(maxlen=EVENT_QUEUE_LENGTH)
        self.dropped_events = 0
        self.read_gap_max = 0.0          # longest time between two reads since take_read_gap_max()
        self._last_read = None
        self.failure = None
        self.failed = threading.Event()
        self.stopping = threading.Event()
        self._threads = []

    # --- one step of each loop (the threads repeat these; tests call them directly) ---

    def read_once(self):
        chunk = self.port.read(self.port.in_waiting or 1)
        now = self.clock.monotonic()
        if self._last_read is not None:
            self.read_gap_max = max(self.read_gap_max, now - self._last_read)
        self._last_read = now
        if not chunk:
            return
        received_at = self.clock.time()
        events = self.logger.feed(chunk, received_at, now)
        with self.hello_lock:
            handle_hello_events(events, self.hello, now, self.notes.append)
        for event in events:
            if event[0] == 'frame' and event[3] is not None:
                self._queue((received_at, event[1], event[2], event[3], now))
            elif event[0] == 'boot':                 # the board restarted: its clock starts over
                self._queue((received_at, 'boot', None, None, now))

    def _queue(self, item):
        if len(self.events) == self.events.maxlen:
            self.dropped_events += 1
        self.events.append(item)

    def tx_once(self):
        """One TX step: HELLO if due, then what tx_source asks for. The only place that writes."""
        now = self.clock.monotonic()
        with self.hello_lock:
            frames = [('HELLO', frame) for frame in self.hello.due(now)]
        if frames:
            self.logger.mark_hello_sent()
        if self.tx_source is not None:
            frames += [(action[0], self._encode(action)) for action in self.tx_source(now)]
        for name, frame in frames:
            self.port.write(frame)
            self.sent[name] += 1

    hello_once = tx_once                   # (the name of groups 1-2)

    def _encode(self, action):
        if action[0] == 'ESTOP':
            return rover_link.encode_frame(rover_link.ESTOP, self.sequence.next())
        _, speed, steer, arm = action
        return rover_link.encode_frame(rover_link.DRIVE, self.sequence.next(),
                                       rover_link.pack_drive(speed, steer, steer_valid=True, arm=arm))

    # --- threads ---

    def start(self):
        self._threads = [threading.Thread(target=self._loop, args=(self.read_once, None), name='link-reader',
                                          daemon=True),
                         threading.Thread(target=self._loop, args=(self.tx_once, TX_PERIOD_SECONDS),
                                          name='link-tx', daemon=True)]
        for thread in self._threads:
            thread.start()

    def _loop(self, step, pause):
        try:
            while not self.stopping.is_set():
                step()
                if pause is not None and self.stopping.wait(pause):
                    break
        except Exception as error:       # the port is gone (or broken): the node exits
            if not self.stopping.is_set():
                self.failure = error
                self.failed.set()

    def stop(self):
        """Stops both threads (each within a read timeout or a TX period), then closes the port."""
        self.stopping.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        try:
            self.port.close()
        except Exception:
            pass

    # --- for the node's log ---

    def take_events(self):
        events = []
        while self.events:
            events.append(self.events.popleft())
        return events

    def take_notes(self):
        notes = []
        while self.notes:
            notes.append(self.notes.popleft())
        return notes

    def take_read_gap_max(self):
        gap, self.read_gap_max = self.read_gap_max, 0.0
        return gap

    def counts(self):
        logger = self.logger
        return {'frames': logger.counts['frames'], 'bad': logger.counts['bad'],
                'seq_gaps': sum(logger.seq_gaps.values()), 't_ms_gaps': logger.t_ms_gaps,
                'dropped': self.dropped_events, 'hello_sent': self.hello.sent_count,
                'drive_sent': self.sent['DRIVE'], 'estop_sent': self.sent['ESTOP']}

    def stats_text(self):
        logger = self.logger
        types = ' '.join(f'{name}={count}' for name, count in sorted(logger.type_counts.items())) or 'none'
        gaps = sum(logger.seq_gaps.values())
        return (f"frames: {types}; bad={logger.counts['bad']} seq_gaps={gaps} t_ms_gaps={logger.t_ms_gaps} "
                f"boot={logger.counts['boot']}; stale: {logger.stale_text()}; hello_sent={self.hello.sent_count} "
                f"dropped={self.dropped_events}")
