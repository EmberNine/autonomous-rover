"""Keyboard teleop node (group 4): runs on the Pi, inside the bridge's container, started by
tools/teleop_keys.py on the laptop over SSH. The keys and a keepalive byte every 100 ms come in
on stdin (a pipe: ssh -T, docker exec -i); teleop.py has the rules and the deadman.

  /cmd_vel          geometry_msgs/Twist, 20 Hz on a STEADY-clock timer (a wall-clock step must
                    not delay it: the bridge stops the wheel after 300 ms), the bridge's QoS
  /rover/arm        on r
  /rover/estop      on x, and on every way out: q, end of input, 2 s without a byte, SIGINT,
                    SIGTERM, SIGHUP: one last /cmd_vel of 0, then the ESTOP, then nothing more
  /diagnostics      read for the status line: the bridge's state and the steering it sends, the
                    battery and the Pi shutdown watcher's countdown (group 5)

stdout: READY_LINE once running (the forwarder reads keys only after it, so ssh's password
prompt has the console to itself), event lines as '[epoch s] text', and a status line ('\\r',
5 Hz). The events also go to teleop_<date>_<time>.log in --log-dir (default /logs, the
container's raw-log volume, if writable), with the Pi's epoch time, next to the bridge's log.

Run (normally through tools/teleop_keys.py):
    docker exec -i rover-bridge /ros_entrypoint.sh python3 -u -m rover_bridge.teleop_node
"""
import argparse
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from rover_bridge import _tools  # noqa: F401  (tools/ on the path: serial_logger)
from rover_bridge import conversions
from rover_bridge.teleop import Teleop
from serial_logger import ExitSignals

READY_LINE = 'teleop: ready'
DEFAULT_VEHICLE_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..',
                                   'config', 'vehicle.cfg')
PUBLISH_PERIOD_SECONDS = 0.05          # 20 Hz
STATUS_EVERY_TICKS = 4                 # 5 Hz
EXIT_ESTOP_WAIT_SECONDS = 1.0
STATUS_WIDTH = 190
COMMAND_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # as the bridge's subscription
STOP_SIGNALS = ('SIGINT', 'SIGTERM', 'SIGHUP')
OPERATOR_STATUS = 'rover_bridge: operator'
BATTERY_STATUS = 'rover_bridge: battery'


def read_input(stream, received, clock=time):
    """The reader thread: (text, monotonic receive time) for each read; (None, time) at the end."""
    fd = stream.fileno()
    while True:
        try:
            data = os.read(fd, 64)
        except OSError:
            data = b''
        now = clock.monotonic()
        if not data:
            received.put((None, now))
            return
        received.put((data.decode('ascii', errors='replace'), now))


class TeleopNode(Node):

    def __init__(self, vehicle_cfg, out, log_file=None, stdin=sys.stdin, clock=time):
        super().__init__('rover_teleop')
        self.clock = clock
        self.out = out
        self.log_file = log_file
        wheelbase = conversions.vehicle_value(vehicle_cfg, 'wheelbase_meters')
        max_angle = conversions.vehicle_value(vehicle_cfg, 'max_steering_angle_radians')
        self.cps_per_meter = 1.0 / conversions.vehicle_value(vehicle_cfg, 'meters_per_encoder_tick')
        self.teleop = Teleop(wheelbase, max_angle, clock.monotonic())
        self.publisher = self.create_publisher(Twist, '/cmd_vel', COMMAND_QOS)
        self.triggers = {name: self.create_client(Trigger, f'/rover/{name}') for name in ('arm', 'estop')}
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.bridge = 'bridge: no /diagnostics yet'
        self.battery = ''
        self.reply = ''
        self.ticks = 0
        self.published = 0
        self.received = queue.Queue()
        self.estop_future = None
        threading.Thread(target=read_input, args=(stdin, self.received, clock), daemon=True).start()
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(PUBLISH_PERIOD_SECONDS, self.on_tick, clock=steady)
        self.event(f'wheelbase {wheelbase} m, full lock {max_angle} rad; DISARMED until r (/rover/arm)')
        self.write(READY_LINE + '\n')

    # --- 20 Hz ---

    def on_tick(self):
        while True:
            try:
                text, at = self.received.get_nowait()
            except queue.Empty:
                break
            self.act(self.teleop.on_eof(at) if text is None else self.teleop.on_bytes(text, at))
        now = self.clock.monotonic()
        self.act(self.teleop.tick(now))
        twist = self.teleop.twist()
        if twist is not None:
            message = Twist()
            message.linear.x, message.angular.z = twist
            self.publisher.publish(message)
            self.published += 1
        self.ticks += 1
        if self.ticks % STATUS_EVERY_TICKS == 0 and self.teleop.finished is None:
            self.write('\r' + self.status_text(now).ljust(STATUS_WIDTH)[:STATUS_WIDTH])

    def publish_zero(self):
        """One last /cmd_vel of 0 on the way out; nothing is published after it."""
        self.publisher.publish(Twist())
        self.published += 1

    def act(self, requests):
        for text in self.teleop.take_events():
            self.event(text)
        for request in requests:
            self.call(request)

    def call(self, name):
        client = self.triggers[name]
        if name == 'estop':
            self.estop_future = None          # an earlier x's reply is not this one's
        if not client.service_is_ready():
            self.reply = f'/rover/{name}: not available (bridge not running?)'
            self.event(self.reply)
            return None
        future = client.call_async(Trigger.Request())

        def done(result):
            response = result.result()
            self.reply = f'/rover/{name}: ' + ('ok' if response.success else 'refused') + f' ({response.message})'
            self.event(self.reply)
        future.add_done_callback(done)
        if name == 'estop':
            self.estop_future = future
        return future

    # --- /diagnostics: the bridge's side, for the status line ---

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == OPERATOR_STATUS:
                values = {item.key: item.value for item in status.values}
                self.bridge = (f"bridge {values.get('state', '?')} sends {values.get('drive cps (sent now)', '?')} cps "
                               f"{values.get('drive steer % (sent now)', '?')}%")
            elif status.name == BATTERY_STATUS:
                self.battery = f'bat {status.message}'     # with the shutdown watcher's countdown, if any

    # --- output ---

    def status_text(self, now):
        text = self.teleop.status_text(now, self.cps_per_meter) + f' | {self.bridge}'
        if self.battery:
            text += f' | {self.battery}'
        return text + (f' | {self.reply}' if self.reply else '')

    def event(self, text):
        line = f'[{time.time():.3f}] {text}'
        self.write('\r' + line.ljust(STATUS_WIDTH) + '\n')
        if self.log_file is not None:
            self.log_file.write(line + '\n')
            self.log_file.flush()

    def write(self, text):
        try:
            self.out.write(text)
            self.out.flush()
        except (OSError, ValueError):
            pass                         # the laptop is gone: the deadman or end of input acts

    def finish_line(self):
        counts = self.teleop.counts
        return (f"final: {self.teleop.finished}; published {self.published}; keys {counts['keys']}, keepalives "
                f"{counts['keepalives']}, ignored {counts['ignored']}, link stops {counts['link stops']}, "
                f'longest gap {self.teleop.gap_max * 1000:.0f} ms')


def wind_down(node, spin_once, signal_name=None):
    """Every way out: (a signal: the teleop finishes now) the timer stops, one /cmd_vel of 0, the
    /rover/estop that finishing asked for is waited for (up to EXIT_ESTOP_WAIT_SECONDS), the final
    line. spin_once(seconds) spins whatever executor the node is on."""
    if node.teleop.finished is None:
        node.act(node.teleop.on_signal(signal_name or 'stopped'))
    node.destroy_timer(node.timer)
    node.publish_zero()                                   # the wheel stops even without /rover/estop
    end = time.monotonic() + EXIT_ESTOP_WAIT_SECONDS
    while node.estop_future is not None and not node.estop_future.done() and time.monotonic() < end:
        spin_once(0.05)
    if node.estop_future is None or not node.estop_future.done():
        node.event('no /rover/estop reply: the bridge ESTOPs 1 s after /cmd_vel stops (heartbeat)')
    node.event(node.finish_line())


def open_log(directory):
    if not directory or not os.path.isdir(directory) or not os.access(directory, os.W_OK):
        return None
    path = os.path.join(directory, time.strftime('teleop_%Y%m%d_%H%M%S.log'))
    try:
        return open(path, 'a', encoding='utf-8')
    except OSError:
        return None


def run(argv=None, stdin=sys.stdin, out=sys.stdout):
    parser = argparse.ArgumentParser(description='Keyboard teleop node (run through tools/teleop_keys.py).')
    parser.add_argument('--vehicle-cfg', default=os.path.normpath(DEFAULT_VEHICLE_CFG))
    parser.add_argument('--log-dir', default='/logs')
    args, _ros_args = parser.parse_known_args(argv)       # rclpy reads its own (--ros-args) from sys.argv
    stop = threading.Event()
    log_file = open_log(args.log_dir)
    node = None
    with ExitSignals(stop.set, names=STOP_SIGNALS) as signals:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        try:
            node = TeleopNode(args.vehicle_cfg, out, log_file=log_file, stdin=stdin)
            while not stop.is_set() and node.teleop.finished is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            wind_down(node, lambda seconds: rclpy.spin_once(node, timeout_sec=seconds), signals.received)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
            if log_file is not None:
                log_file.close()
    return 0


def main():
    sys.exit(run(sys.argv[1:]))


if __name__ == '__main__':
    main()
