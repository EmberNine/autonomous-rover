"""The built rover_ekf node, wired up: fake /rover/imu/raw, /rover/joint_states and /rover/steering
in, /odom and TF out, /rover/ekf/reset. The logic itself is tested with the C++ core
(tests/test_odometry_fusion.cpp); this checks topic names, units, frames and signs.

Needs ROS 2 and a colcon build of rover_ekf: skipped without them. The node is found with
`ros2 pkg prefix rover_ekf`, or ROVER_EKF_NODE (the executable's path). In WSL:
    source /opt/ros/jazzy/setup.bash
    colcon build --base-paths ros_ws/src/rover_ekf --build-base /tmp/ekf_ws/build --install-base /tmp/ekf_ws/install
    source /tmp/ekf_ws/install/setup.bash
    python3 -m pytest -q ros_ws/src/rover_ekf/test
Isolated like the bridge's node tests (localhost discovery, a domain of its own) and, as two
processes on one WSL host, with a discovery anchor started first (docs/ros-bridge.md)."""
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 50)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
METERS_PER_TICK = 3.9336e-5                    # config/vehicle.cfg
RADIANS_PER_COUNT = 2 * math.pi / 4096
ANCHOR = """import rclpy
from rclpy.executors import ExternalShutdownException
rclpy.init()
node = rclpy.create_node('discovery_anchor')
print('ready', flush=True)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
"""


def node_executable():
    if os.environ.get('ROVER_EKF_NODE'):
        return os.environ['ROVER_EKF_NODE']
    if not shutil.which('ros2'):
        return None
    result = subprocess.run(['ros2', 'pkg', 'prefix', 'rover_ekf'], capture_output=True, text=True)
    path = Path(result.stdout.strip()) / 'lib' / 'rover_ekf' / 'ekf_node'
    return str(path) if result.returncode == 0 and path.exists() else None


def spin_for(executor, seconds, each=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.002)
        if each is not None:
            each()


class Inputs:
    """The bridge's three topics at 100 Hz (STATE) and 10 Hz (STATUS), stamped now."""

    def __init__(self, node):
        self.node = node
        self.imu = node.create_publisher(Imu, '/rover/imu/raw', qos_profile_sensor_data)
        self.joints = node.create_publisher(JointState, '/rover/joint_states', qos_profile_sensor_data)
        self.steering = node.create_publisher(JointState, '/rover/steering', qos_profile_sensor_data)
        self.speed, self.yaw_rate, self.angle = 0.0, 0.0, 0.0
        self.next_at, self.frames = time.monotonic(), 0

    def __call__(self):
        if time.monotonic() < self.next_at:
            return
        self.next_at += 0.01
        stamp = self.node.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ['rear_left_wheel_joint']
        joint.position = [0.0]
        # The shipped cfg's rear axle is locked: the encoder wheel reads the centre's speed.
        joint.velocity = [self.speed / METERS_PER_TICK * RADIANS_PER_COUNT]
        self.joints.publish(joint)
        imu = Imu()
        imu.header.stamp = stamp
        imu.angular_velocity.z = self.yaw_rate
        self.imu.publish(imu)
        if self.frames % 10 == 0:
            steering = JointState()
            steering.header.stamp = stamp
            steering.name = ['steering']
            steering.position = [self.angle]
            self.steering.publish(steering)
        self.frames += 1


def test_odom_and_tf_from_the_three_inputs_and_the_reset():
    executable = node_executable()
    if executable is None:
        pytest.skip('rover_ekf is not built (colcon build, then source its install/setup.bash)')
    if shutil.which('fastdds'):
        subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True, timeout=30)
    anchor = subprocess.Popen([sys.executable, '-c', ANCHOR], stdout=subprocess.PIPE, text=True)
    assert anchor.stdout.readline().strip() == 'ready'
    ekf = subprocess.Popen([executable, '--ros-args', '-p', f'vehicle_cfg:={REPO}/config/vehicle.cfg'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    try:
        node = rclpy.create_node('ekf_test')
        odometry = []
        node.create_subscription(Odometry, '/odom', odometry.append, 10)
        from tf2_msgs.msg import TFMessage
        transforms = []
        node.create_subscription(TFMessage, '/tf', transforms.append, 100)
        reset = node.create_client(Trigger, '/rover/ekf/reset')
        inputs = Inputs(node)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        started = time.monotonic()                               # at rest from here
        end = started + 8.0                                      # discovery (~1.5 s in WSL) + 2 s at rest
        while not odometry and time.monotonic() < end:
            spin_for(executor, 0.05, inputs)
        assert odometry, 'no /odom after 8 s at rest'
        assert time.monotonic() - started >= 2.0                 # not before 2 s at rest
        assert odometry[-1].header.frame_id == 'odom' and odometry[-1].child_frame_id == 'base_link'
        count = len(odometry)
        spin_for(executor, 1.0, inputs)
        assert 35 <= len(odometry) - count <= 60, len(odometry) - count       # ~50 Hz
        inputs.speed = 0.1                                       # 2 s straight at 0.1 m/s
        spin_for(executor, 2.0, inputs)
        last = odometry[-1]
        assert last.pose.pose.position.x == pytest.approx(0.2, abs=0.04), last.pose.pose.position
        assert abs(last.pose.pose.position.y) < 0.01
        assert last.twist.twist.linear.x == pytest.approx(0.1, abs=0.01)    # the centre's speed
        inputs.yaw_rate, inputs.angle = 0.2, math.atan(0.133 * 0.2 / 0.1)    # a left turn
        spin_for(executor, 1.5, inputs)
        heading = 2 * math.atan2(odometry[-1].pose.pose.orientation.z, odometry[-1].pose.pose.orientation.w)
        assert heading == pytest.approx(0.3, abs=0.06), heading               # + = left (CCW)
        assert any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_link'
                   for message in transforms for t in message.transforms)
        inputs.speed = inputs.yaw_rate = inputs.angle = 0.0
        assert reset.wait_for_service(timeout_sec=2.0)
        future = reset.call_async(Trigger.Request())
        spin_for(executor, 0.5, inputs)
        assert future.done() and future.result().success, future.result()
        assert abs(odometry[-1].pose.pose.position.x) < 0.01
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
        ekf.send_signal(signal.SIGINT)
        try:
            output, _ = ekf.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            ekf.kill()
            output, _ = ekf.communicate()
        anchor.send_signal(signal.SIGINT)
        try:
            anchor.wait(timeout=5)
        except subprocess.TimeoutExpired:
            anchor.kill()
        print(output)
    assert 'started: gyro bias' in output and 'pose reset to (0, 0, 0)' in output, output
