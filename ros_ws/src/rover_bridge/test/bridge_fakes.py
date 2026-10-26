"""Fakes for the bridge tests: a clock, a step-by-step port, and a threaded fake board."""
import collections
import math
import threading
import time

from rover_bridge import _tools  # noqa: F401
import rover_link  # noqa: E402

HELLO_ACK_FIELDS = {'build_flags': 0, 'reset_flags': 4, 'imu_ok': True, 'imu_who': 0x47, 'state_period_ms': 10,
                    'status_period_ms': 100}
BOOT_LINE = b'imu_init=OK who=0x47\r\n'


def state_fields(t_ms, **overrides):
    """A STATE as the board sends it at rest on the lift, 10 IMU samples per period:
    gyro z +10 deg/s, accel z +1 g, encoder at one revolution and +0.5 rev/s."""
    fields = {name: 0 for name in rover_link.STATE_FIELDS}
    fields.update(t_ms=t_ms, count=4096, speed_cps=2048, enc_flags=rover_link.ENC_VALID_NOW,
                  gyro_z=655 * 10, accel_z=8192 * 10, imu_samples=10, imu_flags=rover_link.IMU_OK, motor_state=2)
    fields.update(overrides)
    return fields


def status_fields(**overrides):
    fields = {name: 0 for name in rover_link.STATUS_FIELDS}
    fields.update(motor_state=2, latch_reason=7, battery_state=1, vbat_mv=7700, imu_ok=1)
    fields.update(overrides)
    return fields


def hello_ack_frame(seq=0, **overrides):
    return rover_link.encode_frame(rover_link.HELLO_ACK, seq, rover_link.pack_hello_ack(dict(HELLO_ACK_FIELDS,
                                                                                               **overrides)))


def state_frame(seq, t_ms, **overrides):
    return rover_link.encode_frame(rover_link.STATE, seq & 0xFF, rover_link.pack_state(state_fields(t_ms, **overrides)))


def status_frame(seq, **overrides):
    return rover_link.encode_frame(rover_link.STATUS, seq & 0xFF, rover_link.pack_status(status_fields(**overrides)))


def written_types(writes):
    decoder = rover_link.FrameDecoder()
    decoder.feed(b'\x00')
    types = []
    for data in writes:
        for status, frame, _ in decoder.feed(data):
            assert status == rover_link.OK, status
            types.append(frame.type)
    return types


class StepClock:
    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1_792_000_000.0 + self.now


class StepPort:
    """Bytes put with give() come out of the next read(); every read() takes 20 ms of clock time."""

    def __init__(self, clock):
        self.clock = clock
        self.pending = bytearray()
        self.writes = []
        self.closed = False
        self.error = None

    @property
    def in_waiting(self):
        return len(self.pending)

    def give(self, data):
        self.pending += data

    def read(self, size):
        if self.error is not None:
            raise self.error
        self.clock.now += 0.02
        data = bytes(self.pending[:max(size, 1)])
        del self.pending[:len(data)]
        return data

    def write(self, data):
        if self.error is not None:
            raise self.error
        self.writes.append(data)

    def close(self):
        self.closed = True


class FakeMotor:
    """The firmware's motor rules as far as the bridge sees them (motor_control.c, link_rx.c):
    LATCHED (BOOT) at start; a DRIVE with the arm bit takes LATCHED to OFF whatever the reason; a
    DRIVE's speed (in 50-cps units, LinkProtocol_CpsToUnits) sets the setpoint, RUNNING when it
    isn't 0; ESTOP latches (6); HELLO latches (7) unless already latched; no DRIVE for 500 ms while
    RUNNING latches (WATCHDOG, 2: the watchdog acts while the driver is enabled). received: frames
    by type. setpoint_cps is what STATUS reports: every valid DRIVE's speed as received, not
    quantized and not cleared by a latch (link_rx.c drive_speed_cps); units is what the motor runs on.
    The steering (steering_control.c): a DRIVE with steer_valid sets steer_percent, whatever the
    motor's state (an ESTOP or a latch doesn't touch it); no such DRIVE for 500 ms -> centre (0)."""

    def __init__(self, latch_reason=1):
        self.state, self.latch_reason, self.setpoint_cps, self.units = 2, latch_reason, 0, 0
        self.last_drive = None
        self.steer_percent, self.last_steer = 0, None
        self.received = collections.Counter()
        self.decoder = rover_link.FrameDecoder()
        self.decoder.feed(b'\x00')
        self.lock = threading.Lock()

    def receive(self, data, now):
        with self.lock:
            for status, frame, _ in self.decoder.feed(data):
                if status != rover_link.OK:
                    continue
                name = rover_link.MESSAGE_NAMES.get(frame.type, frame.type)
                self.received[name] += 1
                if frame.type == rover_link.DRIVE and rover_link.drive_check(frame.payload) == 'OK':
                    drive = rover_link.unpack_drive(frame.payload)
                    self.last_drive = now
                    self.setpoint_cps = drive['speed_cps']
                    if drive['steer_valid']:
                        self.steer_percent, self.last_steer = drive['steer_percent'], now
                    if drive['arm'] and self.state == 2:
                        self.state, self.latch_reason = 0, 0
                    if self.state in (0, 1):
                        self.units = rover_link.cps_to_units(drive['speed_cps'])
                        self.state = 1 if self.units else 0
                elif frame.type == rover_link.ESTOP:
                    self.state, self.latch_reason, self.units = 2, 6, 0
                elif frame.type == rover_link.HELLO and self.state != 2:
                    self.state, self.latch_reason, self.units = 2, 7, 0

    def tick(self, now):
        with self.lock:
            if self.state == 1 and self.last_drive is not None and now - self.last_drive >= 0.5:
                self.state, self.latch_reason, self.units = 2, 2, 0
            if self.last_steer is not None and now - self.last_steer >= 0.5:
                self.steer_percent, self.last_steer = 0, None

    def fields(self):
        with self.lock:
            return {'motor_state': self.state, 'latch_reason': self.latch_reason, 'setpoint_cps': self.setpoint_cps}


class FakeBoardPort:
    """A board in real time: answers HELLO with HELLO_ACK, then sends STATE every 10 ms and
    STATUS every 100 ms, with the motor of a FakeMotor (motor). read() waits up to timeout for
    bytes like pyserial. fail() makes reads raise, as an unplugged port does."""

    def __init__(self, timeout=0.02, build_flags=0, status=None):
        self.timeout = timeout
        self.build_flags = build_flags
        self.status = status or {}       # STATUS field overrides (e.g. no pack on wall power)
        self.motor = FakeMotor(latch_reason=self.status.get('latch_reason', 1))
        self.buffer = bytearray()
        self.condition = threading.Condition()
        self.writes = []
        self.closed = False
        self.error = None
        self.streaming = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    @property
    def in_waiting(self):
        with self.condition:
            return len(self.buffer)

    def _send(self, data):
        with self.condition:
            self.buffer += data
            self.condition.notify_all()

    def _run(self):
        seq = {'STATE': 0, 'STATUS': 0}     # one counter per type, like the firmware
        t_ms = 0
        next_time = time.monotonic()
        while not self.closed:
            next_time += 0.01
            time.sleep(max(0.0, next_time - time.monotonic()))
            if not self.streaming:
                continue
            self.motor.tick(time.monotonic())
            motor = self.motor.fields()
            t_ms += 10
            self._send(state_frame(seq['STATE'], t_ms, motor_state=motor['motor_state']))
            seq['STATE'] += 1
            if t_ms % 100 == 0:
                fields = dict(motor, **{key: value for key, value in self.status.items() if key not in motor})
                self._send(status_frame(seq['STATUS'], t_ms=t_ms - 2, **fields))
                seq['STATUS'] += 1

    def read(self, size):
        with self.condition:
            if self.error is not None:
                raise self.error
            if not self.buffer:
                self.condition.wait(self.timeout)
            if self.error is not None:
                raise self.error
            data = bytes(self.buffer[:max(size, 1)])
            del self.buffer[:len(data)]
            return data

    def write(self, data):
        self.writes.append(data)
        self.motor.receive(data, time.monotonic())
        if rover_link.HELLO in written_types([data]):
            self._send(hello_ack_frame(build_flags=self.build_flags))
            self.streaming = True

    def flush(self):
        pass

    def fail(self, error):
        with self.condition:
            self.error = error
            self.condition.notify_all()

    def close(self):
        self.closed = True


EXPECTED_GYRO_Z = math.radians(10.0)
EXPECTED_ACCEL_Z = 9.80665


class PtyBoard:
    """A board on a pty, for running the real node as a process: answers HELLO with HELLO_ACK, then
    STATE every 10 ms and STATUS every 100 ms (real time) with a FakeMotor (motor; battery OK), until
    stop(). path: the port to open."""

    def __init__(self):
        import os
        import pty
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.motor = FakeMotor()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        import os
        import select
        decoder = rover_link.FrameDecoder()
        decoder.feed(b'\x00')
        started = None
        sent = 0
        while not self.stopping.is_set():
            try:
                if select.select([self.master], [], [], 0.002)[0]:
                    data = os.read(self.master, 4096)
                    self.motor.receive(data, time.monotonic())
                    for status, frame, _ in decoder.feed(data):
                        if status == rover_link.OK and frame.type == rover_link.HELLO and started is None:
                            os.write(self.master, hello_ack_frame())
                            started = time.monotonic()
                if started is None:
                    continue
                while (sent + 1) * 0.01 <= time.monotonic() - started:
                    sent += 1
                    self.motor.tick(time.monotonic())
                    motor = self.motor.fields()
                    out = state_frame(sent, sent * 10, motor_state=motor['motor_state'])
                    if sent % 10 == 0:
                        out += status_frame(sent // 10, t_ms=sent * 10 - 2, **motor)
                    os.write(self.master, out)
            except OSError:
                return

    def stop(self):
        import os
        self.stopping.set()
        self.thread.join(timeout=1.0)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
