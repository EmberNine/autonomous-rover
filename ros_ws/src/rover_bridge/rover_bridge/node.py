"""rover_bridge node (group 3): the Nucleo's STATE and STATUS as ROS topics, and driving.

Driving (safety.py has the rules): /cmd_vel (geometry_msgs/Twist; linear.x m/s -> counts/s;
angular.z -> steering %, bicycle model) is the operator's heartbeat. /rover/arm and /rover/arm_force
(std_srvs/Trigger) arm; /rover/estop sends ESTOP x3. The TX thread (link.py) sends a DRIVE every
20 ms only while armed. ESTOP x3 goes out on every way out (SIGINT, SIGTERM, SIGHUP, an
exception) before anything is torn down.


  /rover/imu/raw      sensor_msgs/Imu           100 Hz: rad/s, m/s^2, frame imu_link, no orientation
  /rover/joint_states sensor_msgs/JointState    100 Hz: the encoder wheel (rear_left_wheel_joint):
                                                position rad (cumulative odometer), velocity rad/s
                                                only while the speed estimate is valid
  /rover/steering     sensor_msgs/JointState    10 Hz (STATUS): joint 'steering', the angle the
                                                servo has (rad, + left; conversions.steering_from_status),
                                                none while the steering is in FAULT
  /rover/battery      sensor_msgs/BatteryState  1 Hz (battery.py): no pack = not present, voltage NaN
  /diagnostics        diagnostic_msgs/DiagnosticArray  1 Hz (diagnostics.py): motor, battery, link,
                                                clock fit, operator heartbeat (placeholder)
IMU and joints with sensor-data QoS (best effort, keep last 5); battery
reliable + transient local (a late subscriber gets the last one);
diagnostics reliable. Stamps: the board-clock fit (clock_fit.py), i.e. when
the board made the frame, on the Pi's clock; until the fit has ~2 s of
STATE, the earliest-arrival offset alone.

The log line every 10 s shows the motor state from STATUS and the safety
state. A HELLO_ACK of a test build (build_flags != 0) is an ERROR, and arming
it is refused. The port is opened exclusively; if it fails (unplugged), the node exits
with code 1 and the container's restart policy starts it again. With
wait_for_serial set (the Nucleo's USB serial number), it first waits until
the board is back (sysfs, up to wait_for_serial_timeout): a restart while the
device is missing would fail to start the container.

raw_log: '' (default) or 'disabled' = no raw log; a directory = bridge_<date>_<time>.log
(+ .times.csv) in it, exactly as serial_logger --binary writes them
(tools/link_dump.py reads them). Written by a writer thread
(serial_logger.QueuedWriter): the reader never waits for the disk. If the
file can't be opened, the bridge runs without it.

Parameters: port (/dev/rover-nucleo), baud (115200), imu_frame (imu_link),
wheel_joint (rear_left_wheel_joint), status_period (10.0 s),
diagnostics_period (1.0 s), raw_log (''), wait_for_serial ('' = exit at
once), wait_for_serial_timeout (120.0 s), clock_window (20.0 s),
vehicle_cfg (the repo's config/vehicle.cfg), meters_per_encoder_tick (0.0 =
from vehicle_cfg), state_dir (/run/rover: battery.json for the Pi's low-battery shutdown watcher,
watch.json read back into /diagnostics; power_state.py; not a writable directory = off).
Run: python3 -m rover_bridge.node [--ros-args -p port:=/dev/ttyACM0]
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_srvs.srv import Trigger

from rover_bridge import battery, conversions, power_state
from rover_bridge.clock_fit import ClockFit
from rover_bridge.diagnostics import Diagnostics
from rover_bridge.link import SYSFS_TTY, Link, usb_serial_tty
from rover_bridge.safety import ESTOP_REPEATS, Safety
from serial_logger import ExitSignals, QueuedWriter

DEFAULT_VEHICLE_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..',
                                   'config', 'vehicle.cfg')     # the repo's (in the container: /repo/config)
SHUTDOWN_ESTOP_WAIT = 0.5        # s: the TX thread sends the ESTOPs, 20 ms apart
COMMAND_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # an old command is useless

STOP_SIGNALS = ('SIGINT', 'SIGTERM', 'SIGHUP')   # Ctrl+C, docker stop / shutdown, the terminal gone

READ_TIMEOUT_SECONDS = 0.02      # the reader returns at least 50 times a second
BOARD_POLL_SECONDS = 0.5
BOARD_SETTLE_SECONDS = 1.0
WRITE_TIMEOUT_SECONDS = 0.05     # a stuck USB write raises instead of hanging the TX thread
PUBLISH_PERIOD_SECONDS = 0.01
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def open_serial(port, baud):
    import serial
    return serial.Serial(port, baud, timeout=READ_TIMEOUT_SECONDS, write_timeout=WRITE_TIMEOUT_SECONDS,
                         exclusive=True)


def stamp(unix_seconds):
    seconds = int(unix_seconds)
    return Time(sec=seconds, nanosec=int((unix_seconds - seconds) * 1e9))


def ms_text(seconds):
    return 'n/a' if seconds is None else f'{seconds * 1000:.0f}ms'


def age_text(seconds):
    """'12.3 s ago', or 'at start' (an event without a time: the bridge starting)."""
    return 'at start' if seconds is None else f'{seconds:.1f} s ago'


def raw_log_path(setting, now=None):
    """'' or 'disabled' -> None; a directory -> <dir>/bridge_<YYYYmmdd_HHMMSS>.log; anything else as given.
    ('disabled', not 'off': ROS parses -p values as YAML, where off is false.)"""
    if not setting or setting == 'disabled':
        return None
    if setting.endswith('/') or os.path.isdir(setting):
        return os.path.join(setting, time.strftime('bridge_%Y%m%d_%H%M%S.log', time.localtime(now)))
    return setting


class BridgeNode(Node):

    def __init__(self, open_port=open_serial, clock=time):
        super().__init__('rover_bridge')
        self.port_name = self.declare_parameter('port', '/dev/rover-nucleo').value
        baud = self.declare_parameter('baud', 115200).value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.wheel_joint = self.declare_parameter('wheel_joint', 'rear_left_wheel_joint').value
        status_period = self.declare_parameter('status_period', 10.0).value
        diagnostics_period = self.declare_parameter('diagnostics_period', 1.0).value
        raw_log = self.declare_parameter('raw_log', '').value
        self.wait_for_serial = self.declare_parameter('wait_for_serial', '').value
        self.wait_for_serial_timeout = self.declare_parameter('wait_for_serial_timeout', 120.0).value
        clock_window = self.declare_parameter('clock_window', 20.0).value
        vehicle_cfg = self.declare_parameter('vehicle_cfg', os.path.normpath(DEFAULT_VEHICLE_CFG)).value
        meters_per_tick = self.declare_parameter('meters_per_encoder_tick', 0.0).value
        self.state_dir = self.declare_parameter('state_dir', '/run/rover').value
        if not meters_per_tick > 0:
            meters_per_tick = conversions.vehicle_value(vehicle_cfg, 'meters_per_encoder_tick')
        wheelbase = conversions.vehicle_value(vehicle_cfg, 'wheelbase_meters')
        max_steering_angle = conversions.vehicle_value(vehicle_cfg, 'max_steering_angle_radians')
        self.clock = clock
        self.imu_publisher = self.create_publisher(Imu, '/rover/imu/raw', qos_profile_sensor_data)
        self.joint_publisher = self.create_publisher(JointState, '/rover/joint_states', qos_profile_sensor_data)
        self.steering_publisher = self.create_publisher(JointState, '/rover/steering', qos_profile_sensor_data)
        self.max_steering_angle = max_steering_angle
        self.battery_publisher = self.create_publisher(BatteryState, '/rover/battery', LATCHED_QOS)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.published = {'imu': 0, 'joint': 0}
        self.fit = ClockFit(window_seconds=clock_window)
        self.safety = Safety(meters_per_tick, wheelbase, max_steering_angle)
        self.diagnostics = Diagnostics()
        self.boot_id = power_state.read_boot_id()
        self.state_writable = bool(self.state_dir) and os.path.isdir(self.state_dir) and os.access(self.state_dir, os.W_OK)
        self.state_write_failed = False
        self.last_status = None
        self.last_status_at = None       # monotonic, when the last STATUS / STATE was handled
        self.last_state_at = None
        self.last_epoch = None
        self.last_speed_cps = None       # the last STATE's estimate, while valid
        port = open_port(self.port_name, baud)
        self.raw_files = self._open_raw_log(raw_log)
        writers = [writer for _file, writer in self.raw_files]
        self.link = Link(port, clock=clock, raw_file=writers[0] if writers else None,
                         times_file=writers[1] if writers else None, tx_source=self.safety.tick)
        self.get_logger().info(f'opened {self.port_name} at {baud} (exclusive); DISARMED until /rover/arm; '
                               f'{meters_per_tick:.4e} m per count; steering: wheelbase {wheelbase} m, '
                               f'full lock {max_steering_angle} rad')
        self.get_logger().info(f'battery state for the shutdown watcher: {self.state_dir}/battery.json' if self.state_writable
                               else f'battery state file off: {self.state_dir!r} is not a writable directory')
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, COMMAND_QOS)
        self.create_service(Trigger, '/rover/arm', lambda request, response: self.on_arm(response, False))
        self.create_service(Trigger, '/rover/arm_force', lambda request, response: self.on_arm(response, True))
        self.create_service(Trigger, '/rover/estop', self.on_estop)
        self._cpu_mark = (time.process_time(), clock.monotonic())
        # A steady clock: rclpy's default (the ROS clock, i.e. the wall clock) stalls a timer for as
        # long as the wall clock steps back (WSL: ~0.8 s every 32 s; a Pi without an RTC can be
        # NTP-stepped after boot), and publish_pending is what hands STATUS to safety.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.bridge_timers = [self.create_timer(PUBLISH_PERIOD_SECONDS, self.publish_pending, clock=steady),
                              self.create_timer(status_period, self.log_status, clock=steady),
                              self.create_timer(diagnostics_period, self.publish_diagnostics, clock=steady)]
        self.link.start()

    def _open_raw_log(self, setting):
        path = raw_log_path(setting)
        if path is None:
            return []
        try:
            files = [open(path, 'wb'), open(path + '.times.csv', 'w', encoding='ascii', newline='')]
        except OSError as error:
            self.get_logger().error(f'raw log {path}: {error}; running without it')
            return []
        self.get_logger().info(f'raw link log: {path} (+ .times.csv)')
        return [(file, QueuedWriter(file)) for file in files]

    # --- frames ---

    def publish_pending(self):
        for received_at, name, _seq, fields, *read_at in self.link.take_events():
            if name == 'STATE':
                self.publish_state(received_at, fields, *read_at)
            elif name == 'STATUS':
                self.last_status = fields
                self.last_status_at = self.clock.monotonic()
                self.safety.on_status(fields, self.last_status_at)
                self.publish_steering(fields)
            elif name == 'HELLO_ACK':
                self.on_hello_ack(fields)
                self.safety.on_hello_ack(fields, self.clock.monotonic())
            elif name == 'boot':
                self.fit.reset()
                self.last_epoch = None
                self.safety.on_board_reset(self.clock.monotonic())
        for note in self.link.take_notes():
            self.get_logger().info(note)
        # Arm, DRIVE 0, ESTOP...: every event logged as it happens (not only when the text changes).
        for warn, event in self.safety.take_events(self.clock.monotonic()):
            # Two call sites: rclpy refuses one call site changing severity between calls.
            if warn:
                self.get_logger().warn(f'safety: {event}')
            else:
                self.get_logger().info(f'safety: {event}')

    def publish_steering(self, status):
        """/rover/steering (joint 'steering', rad): the angle the servo has, for the EKF; nothing in FAULT."""
        angle = conversions.steering_from_status(status, self.max_steering_angle)
        if angle is None:
            return
        message = JointState()
        message.header.stamp = stamp(self.fit.stamp(status['t_ms'], self.clock.time()))
        message.name = ['steering']
        message.position = [angle]
        self.steering_publisher.publish(message)

    def publish_state(self, received_at, state, read_at=None):
        """read_at: the read's monotonic time (lets the fit tell a host clock step)."""
        self.fit.add(state['t_ms'], received_at, read_at)
        self.last_state_at = self.clock.monotonic()
        header_stamp = stamp(self.fit.state_stamp(state['t_ms'], received_at))
        imu = conversions.imu_from_state(state)
        if imu is not None:
            message = Imu()
            message.header.stamp = header_stamp
            message.header.frame_id = self.imu_frame
            message.orientation_covariance[0] = -1.0          # no orientation estimate (REP 145)
            (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z), \
                (message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z) = imu
            self.imu_publisher.publish(message)
            self.published['imu'] += 1
        position, velocity = conversions.wheel_from_state(state)
        self.last_speed_cps = state['speed_cps'] if velocity is not None else None
        joint = JointState()
        joint.header.stamp = header_stamp
        joint.name = [self.wheel_joint]
        joint.position = [position]
        joint.velocity = [] if velocity is None else [velocity]
        self.joint_publisher.publish(joint)
        self.published['joint'] += 1
        epoch = state['odometer_epoch']
        if self.last_epoch is not None and epoch != self.last_epoch:
            self.get_logger().warn(f'odometer epoch {self.last_epoch} -> {epoch}: distance uncertain')
        self.last_epoch = epoch

    # --- driving ---

    def on_cmd_vel(self, message):
        self.safety.on_cmd_vel(message.linear.x, message.angular.z, self.clock.monotonic())

    def on_arm(self, response, force):
        response.success, response.message = self.safety.arm(self.clock.monotonic(), force=force)
        text = f"{'arm_force' if force else 'arm'}: {response.message}"
        if response.success:                       # two call sites (rclpy: one severity per call site)
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        return response

    def on_estop(self, request, response):
        self.safety.estop(self.clock.monotonic(), 'requested (/rover/estop)')
        response.success, response.message = True, f'ESTOP x{ESTOP_REPEATS} sent; /rover/arm to drive again'
        self.get_logger().warn('estop: ' + response.message)
        return response

    def shutdown_estop(self):
        """ESTOP x3 through the TX thread (the port's only writer), and wait until they are out.
        Nothing to do if the port is gone (the firmware's watchdog latches the motor)."""
        if self.link.failed.is_set():
            return False
        before = self.link.sent['ESTOP']
        self.safety.shutdown(self.clock.monotonic())
        end = self.clock.monotonic() + SHUTDOWN_ESTOP_WAIT
        while self.link.sent['ESTOP'] < before + ESTOP_REPEATS and self.clock.monotonic() < end \
                and not self.link.failed.is_set():
            time.sleep(0.005)
        return self.link.sent['ESTOP'] >= before + ESTOP_REPEATS

    def on_hello_ack(self, fields):
        # The HELLO_ACK line itself is a note (serial_logger.handle_hello_events, hello_ack_text).
        if fields['build_flags']:
            self.get_logger().error('test build (build_flags != 0): never drive the truck with it')

    # --- 1 Hz: battery and diagnostics ---

    def _age(self, at):
        return None if at is None else self.clock.monotonic() - at

    def snapshot(self):
        return {'status': self.last_status, 'hello_ack': self.link.logger.hello_ack,
                'state_age': self._age(self.last_state_at), 'status_age': self._age(self.last_status_at),
                'counts': self.link.counts(), 'stale_text': self.link.logger.stale_text(),
                'read_gap_period': self.link.take_read_gap_max(),
                'read_gap_session': self.link.logger.read_gap.longest, 'clock': self.fit,
                'safety': self.safety.snapshot(self.clock.monotonic()), 'speed_cps': self.last_speed_cps,
                'power_watch': self.read_power_watch()}

    def read_power_watch(self):
        if not self.state_dir:
            return None
        return power_state.read_watch(os.path.join(self.state_dir, 'watch.json'), self.clock.monotonic(), self.boot_id)

    def write_battery_state(self):
        """battery.json for the host's shutdown watcher (power_state.py), from the last STATUS."""
        if not self.state_writable or self.last_status is None:
            return
        try:
            power_state.write_atomic(os.path.join(self.state_dir, 'battery.json'), power_state.battery_state(
                self.last_status, self.last_status_at, self.boot_id, self.clock.time()))
        except OSError as error:
            if not self.state_write_failed:
                self.state_write_failed = True
                self.get_logger().error(f'battery state file: {error} (the shutdown watcher sees stale data)')

    def publish_diagnostics(self):
        now = stamp(self.clock.time())
        array = DiagnosticArray()
        array.header.stamp = now
        for report in self.diagnostics.build(self.snapshot()):
            array.status.append(DiagnosticStatus(
                level=bytes([report['level']]), name=report['name'], message=report['message'],
                hardware_id='nucleo', values=[KeyValue(key=key, value=value) for key, value in report['values']]))
        self.diagnostics_publisher.publish(array)
        if self.last_status is not None:
            message = BatteryState(**battery.battery_fields(self.last_status))
            message.header.stamp = stamp(self.fit.stamp(self.last_status['t_ms'], self.clock.time()))
            self.battery_publisher.publish(message)
        self.write_battery_state()

    def log_status(self, final=False):
        cpu, now = time.process_time(), self.clock.monotonic()
        cpu_percent = 100.0 * (cpu - self._cpu_mark[0]) / max(now - self._cpu_mark[1], 1e-6)
        self._cpu_mark = (cpu, now)
        motor = conversions.motor_text(self.last_status) if self.last_status else 'no STATUS yet'
        vbat = f" vbat={self.last_status['vbat_mv']}mV" if self.last_status else ''
        drift = self.fit.drift_ppm()
        clock_text = 'clock fitting' if drift is None else \
            f'clock {drift:+.0f}ppm residual={self.fit.residual() * 1000:.2f}ms'
        longest = self.link.logger.read_gap.longest
        # The board's own TX drops next to the seq gaps: a frame it never sent is a seq gap here
        # (2026-10-19: one STATE dropped when a HELLO met a text line still going out).
        board_drops = f" (board tx_dropped={self.last_status['tx_dropped']})" if self.last_status else ''
        safety = self.safety.snapshot(now)
        sending = 'none' if safety['drive_cps'] is None else f"{safety['drive_cps']}cps"
        if safety['drive_steer'] is not None:
            sending += f" steer={safety['drive_steer']}%"
        drive = (f"{safety['state']} cmd={safety['command_cps']}cps steer={safety['command_steer']}% "
                 f"sending={sending} sent DRIVE={self.link.sent['DRIVE']} "
                 f"ESTOP={self.link.sent['ESTOP']} heartbeat stops={safety['counts']['heartbeat stops']} "
                 f"(since arm {safety['stops_since_arm']}, cmd_vel gap max {ms_text(safety['gap_max'])}, "
                 f"last stop {ms_text(safety['last_stop_gap'])}, gaps>=150ms {safety['slow_gaps_since_arm']}) (last event {age_text(safety['last_event_age'])}: "
                 f"{safety['last_event']})")
        text = (f"motor {motor}{vbat}; {drive}; published imu={self.published['imu']} joint={self.published['joint']}; "
                f'{self.link.stats_text()}{board_drops}; read_gap_max={0 if longest is None else longest * 1000:.0f}ms; '
                f'{clock_text}; cpu={cpu_percent:.1f}% of one core')
        self.get_logger().info(f'final: {text}' if final else text)

    # --- the end ---

    def stop_timers(self):
        """No callback runs (and publishes) after this: the shutdown's first step."""
        for timer in self.bridge_timers:          # (Node.timers is rclpy's own, read-only)
            self.destroy_timer(timer)
        self.bridge_timers = []

    def wait_for_board(self, sysfs_tty=SYSFS_TTY, sleep=time.sleep, stop=None):
        """After a lost port, before exiting: wait (if wait_for_serial is set) until a tty with
        that USB serial is back, so the container's restart finds /dev/rover-nucleo. Gives up
        after wait_for_serial_timeout, or at a stop signal (docker stop: stop is set)."""
        log = self.get_logger()
        if not self.wait_for_serial:
            log.error('exiting so the container restarts')
            return
        log.error(f'waiting up to {self.wait_for_serial_timeout:.0f} s for the board '
                  f'(USB serial {self.wait_for_serial}) to come back')
        end = self.clock.monotonic() + self.wait_for_serial_timeout
        while (stop is None or not stop.is_set()) and self.clock.monotonic() < end:
            tty = usb_serial_tty(self.wait_for_serial, sysfs_tty)
            if tty is not None:
                sleep(BOARD_SETTLE_SECONDS)                # udev makes the symlink just after the tty appears
                log.error(f'board is back as {tty}; exiting so the container restarts')
                return
            sleep(BOARD_POLL_SECONDS)
        log.error('board not back (or shutting down); exiting')

    def close(self):
        self.link.stop()
        for file, writer in self.raw_files:          # everything queued, written; then closed
            writer.finish()
            if writer.problem_text():
                self.get_logger().error(f'raw log {writer.problem_text()}')
            file.close()
        self.raw_files = []


def run(args=None, open_port=open_serial):
    """The node's life: returns the exit code (0 after a stop signal, 1 if the port failed or
    couldn't be opened).

    The shutdown is the bridge's own: rclpy's signal handlers are off, because on SIGTERM they
    shut the context down from inside spin_once while a timer callback was publishing (docker
    compose stop, 2026-10-17: RCLError "publisher's context is invalid", a traceback). SIGINT,
    SIGTERM and SIGHUP only set stop; the loop ends, then in this order: the timers stop (no more
    publishing), ESTOP x3 (through the TX thread, while the link is open and ROS is up; also after
    an exception), the final: line, the link and the raw log close, the node is destroyed, rclpy
    shuts down.
    The handlers go in before rclpy.init, so SignalHandlerOptions.NO is what keeps them: without
    it rclpy's own would replace them (and the stop test fails)."""
    stop = threading.Event()
    node = None
    code = 0
    with ExitSignals(stop.set, names=STOP_SIGNALS) as signals:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        try:
            try:
                node = BridgeNode(open_port=open_port)
            except Exception as error:      # no port (unplugged, busy, wrong path): the restart policy retries
                rclpy.logging.get_logger('rover_bridge').error(f'cannot open the port: {error}')
                return 1
            while not stop.is_set() and not node.link.failed.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
            if stop.is_set():
                node.get_logger().info(f'{signals.received}: stopping')
            else:
                node.get_logger().error(f'port lost: {node.link.failure!r}')
                node.wait_for_board(stop=stop)
                code = 1
        finally:
            if node is not None:
                node.stop_timers()
                sent = node.shutdown_estop()
                node.get_logger().info(f'shutdown: ESTOP x{ESTOP_REPEATS} ' +
                                       ('sent' if sent else 'not sent (the port is gone: the firmware watchdog '
                                                            'latches the motor)'))
                node.log_status(final=True)
                node.close()
                node.destroy_node()
            rclpy.try_shutdown()
    return code


def main(args=None):
    sys.exit(run(args))


if __name__ == '__main__':
    main()

