"""rover_bridge.teleop_node with a pipe for stdin, in real time, in one process (the teleop node,
a /cmd_vel listener and fake /rover/arm and /rover/estop services share one participant, so no
same-host discovery is involved). Needs ROS 2 (rclpy): skipped without it (CI job python-tests);
CI job ros-tests runs it.

Isolated from the network like test_bridge_node.py: localhost-only discovery on a domain of its own."""
import io
import os
import threading
import time

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'   # before rclpy reads them
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 50)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from rover_bridge import teleop, teleop_node  # noqa: E402


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def spin_until(executor, condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if condition():
            return True
    return False


class Bridge:
    """The bridge as the teleop sees it: /cmd_vel received, /rover/arm and /rover/estop answered."""

    def __init__(self):
        self.node = rclpy.create_node('fake_bridge')
        self.received = []                                       # (monotonic, linear.x, angular.z)
        self.calls = []
        self.node.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel,
                                      QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        for name in ('arm', 'estop'):
            self.node.create_service(Trigger, f'/rover/{name}', self.answer(name))

    def on_cmd_vel(self, message):
        self.received.append((time.monotonic(), message.linear.x, message.angular.z))

    def answer(self, name):
        def callback(request, response):
            self.calls.append(name)
            response.success, response.message = True, f'{name} done'
            return response
        return callback


class Keepalives:
    def __init__(self, fd):
        self.fd, self.running = fd, True
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while self.running:
            os.write(self.fd, teleop.KEEPALIVE.encode())
            time.sleep(0.1)


def test_20_hz_the_keys_and_end_of_input_estop_then_silence(tmp_path):
    rclpy.init()
    try:
        read_end, write_end = os.pipe()
        out = io.StringIO()
        log = open(tmp_path / 'teleop.log', 'a', encoding='utf-8')
        node = teleop_node.TeleopNode(os.path.normpath(teleop_node.DEFAULT_VEHICLE_CFG), out, log_file=log,
                                      stdin=os.fdopen(read_end, 'rb', buffering=0))
        bridge = Bridge()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(bridge.node)
        keepalives = Keepalives(write_end)
        assert teleop_node.READY_LINE + '\n' in out.getvalue()
        spin_for(executor, 0.3)                                  # discovery within the process
        before = len(bridge.received)
        spin_for(executor, 1.0)
        assert 16 <= len(bridge.received) - before <= 24, len(bridge.received) - before   # ~20 Hz
        assert bridge.received[-1][1:] == (0.0, 0.0)
        os.write(write_end, b'waaa')
        assert spin_until(executor, lambda: bridge.received[-1][1] == 0.05, 1.0)
        assert bridge.received[-1][2] == pytest.approx(teleop.angular_z(0.05, 30, 0.133, 0.204))
        os.write(write_end, b'r')
        assert spin_until(executor, lambda: bridge.calls == ['arm'], 2.0)
        assert spin_until(executor, lambda: '/rover/arm: ok (arm done)' in out.getvalue(), 1.0)
        assert '\r' in out.getvalue() and 'last byte' in out.getvalue()   # the status line
        keepalives.running = False
        time.sleep(0.15)
        os.close(write_end)                                      # ssh gone: end of input
        assert spin_until(executor, lambda: node.teleop.finished is not None, 1.0)
        assert 'end of input' in node.teleop.finished
        teleop_node.wind_down(node, lambda seconds: executor.spin_once(timeout_sec=seconds))
        assert bridge.calls == ['arm', 'estop']
        spin_for(executor, 0.2)
        assert bridge.received[-1][1:] == (0.0, 0.0)             # the last /cmd_vel: 0
        count = len(bridge.received)
        spin_for(executor, 0.5)
        assert len(bridge.received) == count                     # nothing after the ESTOP
        log.close()
        logged = (tmp_path / 'teleop.log').read_text()
        assert 'keyboard link closed (end of input): speed 0, /rover/estop, publishing stops' in logged
        assert '/rover/estop: ok (estop done)' in logged and 'final: keyboard link closed' in logged
        node.destroy_node()
        bridge.node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_without_a_bridge_the_exit_says_the_heartbeat_will_estop():
    rclpy.init()
    try:
        read_end, write_end = os.pipe()
        out = io.StringIO()
        node = teleop_node.TeleopNode(os.path.normpath(teleop_node.DEFAULT_VEHICLE_CFG), out,
                                      stdin=os.fdopen(read_end, 'rb', buffering=0))
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        os.write(write_end, b'q')
        assert spin_until(executor, lambda: node.teleop.finished == 'quit (q)', 1.0)
        teleop_node.wind_down(node, lambda seconds: executor.spin_once(timeout_sec=seconds))
        text = out.getvalue()
        # Not available, or (in the full suite) a dead bridge's /rover/estop still in the DDS graph
        # that never answers: either way the exit says the heartbeat will ESTOP.
        assert 'no /rover/estop reply: the bridge ESTOPs 1 s after /cmd_vel stops (heartbeat)' in text
        assert 'final: quit (q)' in text
        os.close(write_end)
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
