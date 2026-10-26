"""rover_bridge.node against a fake board in real time. Needs ROS 2 (rclpy): skipped without it
(CI); runs in WSL with /opt/ros/jazzy sourced.

Isolated from the network: localhost-only discovery on a domain of its own, no static peers. With
~/rover_net.sh sourced (domain 42, the Pi as a peer) the listener here also got the Pi's real
/rover/imu/raw (2026-10-16), and the test failed on its stamps."""
import os
import threading
import time

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'   # before rclpy reads them
os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 50)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402

import bridge_fakes  # noqa: E402
from rover_bridge import node as bridge_node  # noqa: E402
import rover_link  # noqa: E402


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


@pytest.fixture(scope='module', autouse=True)
def discovery_anchor():
    """A third DDS participant on this module's domain, alive for all its tests, in a process of its
    own (all nodes of one rclpy context share one participant, so a node in this process wouldn't do).

    On WSL (2026-10-21) two processes on the same host whose participants start seconds apart never
    discovered each other: a subscriber first, `ros2 topic pub` 3 s later, 0 of ~180 messages, 0/4
    runs. With any third participant started before both (the ros2 CLI daemon, or an idle rclpy node)
    it worked 4/4 and 4/4. The stop tests start the bridge as a subprocess and the operator after it.
    Not understood beyond that (Fast DDS same-host discovery; plain UDP works on every path).

    Also `fastdds shm clean` first: shared-memory segments left by killed processes (the SIGKILL
    test makes them) blocked shared-memory delivery. It only removes those of dead processes."""
    import shutil
    import subprocess
    import sys
    if shutil.which('fastdds'):
        subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True, timeout=30)
    anchor = subprocess.Popen([sys.executable, '-c', ANCHOR], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True)
    assert anchor.stdout.readline().strip() == 'ready', 'the discovery anchor did not start'
    yield
    anchor.send_signal(__import__('signal').SIGINT)
    try:
        anchor.wait(timeout=5)
    except subprocess.TimeoutExpired:
        anchor.kill()


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def test_state_frames_become_imu_and_joint_messages_and_only_hello_is_sent():
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        listener = rclpy.create_node('listener')
        imus, joints = [], []
        listener.create_subscription(Imu, '/rover/imu/raw', imus.append, qos_profile_sensor_data)
        listener.create_subscription(JointState, '/rover/joint_states', joints.append, qos_profile_sensor_data)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(listener)
        spin_for(executor, 1.5)
        bridge.log_status()
        assert len(imus) > 80 and len(joints) > 80               # ~100 Hz after the 0.1 s HELLO delay
        imu = imus[-1]
        assert imu.header.frame_id == 'imu_link' and imu.orientation_covariance[0] == -1.0
        assert imu.angular_velocity.z == pytest.approx(bridge_fakes.EXPECTED_GYRO_Z)
        assert imu.linear_acceleration.z == pytest.approx(bridge_fakes.EXPECTED_ACCEL_Z)
        assert abs(imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9 - time.time()) < 1.0
        joint = joints[-1]
        assert joint.name == ['rear_left_wheel_joint']
        assert joint.position[0] == pytest.approx(2 * 3.141592653589793)
        assert joint.velocity[0] == pytest.approx(3.141592653589793)
        assert bridge.last_status['motor_state'] == 2
        logger = bridge.link.logger
        assert logger.counts['bad'] == 0 and not logger.seq_gaps and logger.t_ms_gaps == 0
        assert set(bridge_fakes.written_types(board.writes)) == {rover_link.HELLO}   # read-only
        bridge.close()
        assert board.closed
    finally:
        rclpy.try_shutdown()


def test_no_wheel_velocity_and_no_imu_while_invalid():
    rclpy.init()
    try:
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: bridge_fakes.FakeBoardPort())
        sent = {'imu': [], 'joint': []}
        bridge.imu_publisher = type('Stub', (), {'publish': staticmethod(sent['imu'].append)})
        bridge.joint_publisher = type('Stub', (), {'publish': staticmethod(sent['joint'].append)})
        bridge.publish_state(time.time(), bridge_fakes.state_fields(10, enc_flags=0, imu_flags=0))
        assert sent['imu'] == [] and len(sent['joint']) == 1
        assert len(sent['joint'][0].velocity) == 0 and len(sent['joint'][0].position) == 1
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_the_port_is_opened_exclusively_with_timeouts(monkeypatch):
    import types
    opened = {}
    fake_serial = types.SimpleNamespace(Serial=lambda *args, **kwargs: opened.update(args=args, kwargs=kwargs))
    monkeypatch.setitem(__import__('sys').modules, 'serial', fake_serial)
    bridge_node.open_serial('/dev/rover-nucleo', 115200)
    assert opened['args'] == ('/dev/rover-nucleo', 115200)
    assert opened['kwargs'] == {'timeout': bridge_node.READ_TIMEOUT_SECONDS,
                                'write_timeout': bridge_node.WRITE_TIMEOUT_SECONDS, 'exclusive': True}


def test_stamps_come_from_the_fit_and_a_board_reset_restarts_it():
    rclpy.init()
    try:
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: bridge_fakes.FakeBoardPort())
        sent = []
        bridge.imu_publisher = type('Stub', (), {'publish': staticmethod(sent.append)})
        bridge.joint_publisher = type('Stub', (), {'publish': staticmethod(lambda message: None)})
        start = 1_792_000_000.0
        for n in range(500):                               # 5 s, every frame 2 ms after it was made
            bridge.publish_state(start + n * 0.01 + 0.002, bridge_fakes.state_fields(n * 10))
        late = start + 500 * 0.01 + 0.002 + 0.080         # then one frame 80 ms late (a burst)
        bridge.publish_state(late, bridge_fakes.state_fields(5000))
        header = sent[-1].header.stamp
        assert abs(header.sec + header.nanosec * 1e-9 - (start + 5.002)) < 0.001   # when made, not when received
        bridge.link.events.append((late, 'boot', None, None))
        bridge.publish_pending()
        assert bridge.fit.resets == 1 and not bridge.fit.fitted
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_run_exits_1_when_the_port_is_lost():
    board = bridge_fakes.FakeBoardPort()
    threading.Timer(0.5, board.fail, args=(OSError(5, 'device disconnected'),)).start()
    started = time.monotonic()
    assert bridge_node.run(args=[], open_port=lambda port, baud: board) == 1
    assert time.monotonic() - started < 2.0 and board.closed


def test_after_a_lost_port_the_node_waits_for_the_board_to_come_back(tmp_path):
    import test_bridge_link
    rclpy.init(args=['--ros-args', '-p', f'wait_for_serial:={test_bridge_link.NUCLEO_SERIAL}',
                     '-p', 'wait_for_serial_timeout:=5.0'])
    try:
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: bridge_fakes.FakeBoardPort())
        sysfs = str(tmp_path / 'class' / 'tty')
        checks = []

        def sleep(seconds):
            checks.append(seconds)
            if len(checks) == 3:                      # the board comes back during the third poll
                test_bridge_link.fake_sysfs(tmp_path, {'ttyACM1': test_bridge_link.NUCLEO_SERIAL})

        bridge.wait_for_board(sysfs_tty=sysfs, sleep=sleep)
        assert checks == [bridge_node.BOARD_POLL_SECONDS] * 3 + [bridge_node.BOARD_SETTLE_SECONDS]
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_run_exits_1_when_the_port_cannot_be_opened():
    def missing(port, baud):
        raise OSError(2, f'could not open port {port}')

    assert bridge_node.run(args=[], open_port=missing) == 1


def test_wall_power_battery_diagnostics_fitted_stamps_and_raw_log(tmp_path):
    import math
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import BatteryState
    from rover_bridge import battery
    import link_dump
    rclpy.init(args=['--ros-args', '-p', f'raw_log:={tmp_path}/'])
    try:
        board = bridge_fakes.FakeBoardPort(status={'battery_state': 0, 'vbat_mv': 2, 'latch_reason': 1})
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        listener = rclpy.create_node('listener')
        batteries, arrays, imus = [], [], []
        listener.create_subscription(BatteryState, '/rover/battery', batteries.append, bridge_node.LATCHED_QOS)
        listener.create_subscription(DiagnosticArray, '/diagnostics', arrays.append, 10)
        listener.create_subscription(Imu, '/rover/imu/raw', imus.append, qos_profile_sensor_data)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(listener)
        spin_for(executor, 4.5)
        assert batteries and arrays and len(imus) > 300
        message = batteries[-1]
        assert not message.present and math.isnan(message.voltage)
        assert message.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN == battery.HEALTH_UNKNOWN
        assert message.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN == battery.STATUS_UNKNOWN
        assert message.power_supply_technology == BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO == battery.TECHNOLOGY_LIPO
        assert (BatteryState.POWER_SUPPLY_HEALTH_GOOD, BatteryState.POWER_SUPPLY_HEALTH_DEAD,
                BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE, BatteryState.POWER_SUPPLY_STATUS_DISCHARGING) == \
            (battery.HEALTH_GOOD, battery.HEALTH_DEAD, battery.HEALTH_UNSPEC_FAILURE, battery.STATUS_DISCHARGING)
        statuses = {status.name: status for status in arrays[-1].status}
        assert set(statuses) == {f'rover_bridge: {part}' for part in ('motor', 'battery', 'link', 'clock', 'operator')}
        assert statuses['rover_bridge: motor'].message == 'LATCHED (BOOT)'
        assert statuses['rover_bridge: battery'].message == 'no pack connected (e.g. on wall power)'
        assert statuses['rover_bridge: link'].level == b'\x00' and statuses['rover_bridge: motor'].hardware_id == 'nucleo'
        assert bridge.fit.fitted or bridge.fit.host_steps   # ~3 s of STATE (unless WSL's clock just stepped)
        stamps = [imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9 for imu in imus]
        # Strictly in order, except where the host clock stepped back (WSL's clock gets corrected
        # now and then; the stamps follow the system clock there): at most one step back per step.
        backwards = sum(1 for earlier, later in zip(stamps, stamps[1:]) if later <= earlier)
        assert backwards <= bridge.fit.host_steps, (backwards, bridge.fit.host_steps)
        # Never in the future (the fit caps a stamp at its receive time); recent, allowing for a
        # single-threaded test executor that falls behind on a busy machine (0.67 s seen in a full run).
        assert time.time() - 2.0 < stamps[-1] <= time.time()
        bridge.close()
        logs = sorted(tmp_path.glob('bridge_*.log'))
        assert len(logs) == 1 and (tmp_path / (logs[0].name + '.times.csv')).exists()
        logger, rows = link_dump.dump(logs[0].read_bytes())
        assert logger.counts['bad'] == 0 and not logger.seq_gaps and len(rows['STATE']) > 300
        assert link_dump.exit_code(logger) == 0
    finally:
        rclpy.try_shutdown()


def test_the_battery_state_file_for_the_shutdown_watcher_and_its_countdown_back(tmp_path):
    """battery.json from the last STATUS, with the time it was received (not the write time: a
    stale STATUS must look stale to the watcher), and watch.json read back into /diagnostics."""
    import json
    rclpy.init(args=['--ros-args', '-p', f'state_dir:={tmp_path}'])
    try:
        board = bridge_fakes.FakeBoardPort(status={'battery_state': 1, 'vbat_mv': 7850, 'latch_reason': 1})
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        assert bridge.state_writable
        spin_for(executor, 1.5)                                     # STATUS, and a diagnostics tick
        bridge.publish_pending()
        bridge.publish_diagnostics()
        data = json.loads((tmp_path / 'battery.json').read_text())
        assert data['boot_id'] == bridge.boot_id and data['boot_id']
        assert data['status_monotonic'] == bridge.last_status_at
        assert (data['battery_state'], data['vbat_mv'], data['motor_state']) == (1, 7850, 2)
        assert abs(data['wall'] - time.time()) < 2.0
        (tmp_path / 'watch.json').write_text(json.dumps({
            'state': 'low', 'text': 'x', 'held_s': 12.0, 'shutdown_in_s': 18.0, 'threshold_mv': 7900,
            'boot_id': bridge.boot_id, 'monotonic': time.monotonic(), 'action': 'log'}))
        reports = {report['name']: report for report in bridge.diagnostics.build(bridge.snapshot())}
        assert reports['rover_bridge: battery']['message'] == '7.85 V; Pi SHUTDOWN in 18 s (log)'
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_the_steering_the_servo_has_is_published_for_the_ekf():
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort(status={'steer_state': 1, 'steer_percent': 50})
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        listener = rclpy.create_node('steering_listener')
        steering = []
        listener.create_subscription(JointState, '/rover/steering', steering.append, qos_profile_sensor_data)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(listener)
        spin_for(executor, 1.5)
        assert len(steering) >= 5, len(steering)                         # STATUS at 10 Hz
        message = steering[-1]
        assert list(message.name) == ['steering']
        assert message.position[0] == pytest.approx(0.5 * 0.204)          # 50 % of the cfg's full lock
        assert abs(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9 - time.time()) < 2.0
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_without_a_state_dir_the_bridge_writes_no_battery_file(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'state_dir:={tmp_path}/missing'])
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        assert not bridge.state_writable
        spin_for(rclpy.executors.SingleThreadedExecutor(), 0.0)
        bridge.publish_diagnostics()                                 # no error, no file
        assert not (tmp_path / 'missing').exists()
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_a_raw_log_that_cannot_be_opened_is_not_fatal(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'raw_log:={tmp_path}/missing_dir/x.log'])
    try:
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: bridge_fakes.FakeBoardPort())
        assert bridge.raw_files == [] and bridge.link.logger.raw_file is None
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_raw_log_path():
    assert bridge_node.raw_log_path('') is None and bridge_node.raw_log_path('disabled') is None
    assert bridge_node.raw_log_path('/logs/x.log') == '/logs/x.log'


def test_compose_parameters_parse_and_the_raw_log_is_off_by_default():
    """The exact -p values compose passes (ROS parses them as YAML: '' fails, 'off' is a bool)."""
    from pathlib import Path
    compose = (Path(__file__).resolve().parents[3] / 'docker' / 'compose.yaml').read_text()
    assert 'raw_log:=${ROVER_RAW_LOG:-disabled}' in compose
    serial = '0673FF535451897067112541'
    rclpy.init(args=['--ros-args', '-p', 'port:=/dev/rover-nucleo', '-p', f'wait_for_serial:={serial}',
                     '-p', 'raw_log:=disabled'])
    try:
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: bridge_fakes.FakeBoardPort())
        assert bridge.raw_files == [] and bridge.wait_for_serial == serial
        bridge.close()
    finally:
        rclpy.try_shutdown()
    assert bridge_node.raw_log_path('/logs/', now=0).startswith('/logs/bridge_1970') or \
        bridge_node.raw_log_path('/logs/', now=0).startswith('/logs/bridge_1969')


@pytest.mark.parametrize('name', ('SIGTERM', 'SIGINT'))
def test_a_stop_signal_shuts_down_cleanly(tmp_path, name):
    """docker compose stop (SIGTERM) and Ctrl+C (SIGINT): exit 0, no traceback, the final: line.
    With rclpy's own handlers (2026-10-17) SIGTERM gave an RCLError traceback from a timer callback."""
    import signal
    import subprocess
    import sys
    from pathlib import Path
    board = bridge_fakes.PtyBoard()
    package = Path(bridge_node.__file__).resolve().parents[1]
    tools = package.parents[2] / 'tools'
    env = dict(os.environ, PYTHONPATH=f"{package}:{tools}:{os.environ.get('PYTHONPATH', '')}",
               ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    process = subprocess.Popen([sys.executable, '-m', 'rover_bridge.node', '--ros-args', '-p', f'port:={board.path}',
                                '-p', 'raw_log:=disabled'], env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(2.5)                                     # publishing at 100 Hz by now
        process.send_signal(getattr(signal, name))
        output, _ = process.communicate(timeout=15)
    finally:
        if process.poll() is None:
            process.kill()
        board.stop()
    assert process.returncode == 0, output
    assert 'Traceback' not in output and 'RCLError' not in output and 'context is invalid' not in output, output
    assert f'{name}: stopping' in output
    final = [line for line in output.splitlines() if 'final: motor ' in line]
    assert len(final) == 1 and 'bad=0 seq_gaps=0' in final[0], output


def test_a_stop_signal_ends_the_wait_for_a_lost_board(tmp_path):
    """Unplugged, waiting up to 60 s for the board to come back: docker stop must not wait that long."""
    import signal
    import subprocess
    import sys
    from pathlib import Path
    board = bridge_fakes.PtyBoard()
    package = Path(bridge_node.__file__).resolve().parents[1]
    tools = package.parents[2] / 'tools'
    env = dict(os.environ, PYTHONPATH=f"{package}:{tools}:{os.environ.get('PYTHONPATH', '')}",
               ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    process = subprocess.Popen([sys.executable, '-m', 'rover_bridge.node', '--ros-args', '-p', f'port:={board.path}',
                                '-p', 'raw_log:=disabled', '-p', 'wait_for_serial:=NOT_A_REAL_SERIAL_0000',
                                '-p', 'wait_for_serial_timeout:=60.0'], env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(2.0)
        board.stop()                                        # "unplugged": the node now waits for it
        time.sleep(1.5)
        started = time.monotonic()
        process.send_signal(signal.SIGTERM)
        output, _ = process.communicate(timeout=20)
    finally:
        if process.poll() is None:
            process.kill()
    assert time.monotonic() - started < 5.0, output
    assert 'waiting up to 60 s for the board' in output and 'Traceback' not in output, output
    assert any('final: motor ' in line for line in output.splitlines()), output


# --- driving (group 3) ----------------------------------------------------------------------

def spin_until(executor, condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if condition():
            return True
    return False


class Operator:
    """A test-side node: /cmd_vel at 20 Hz while publishing, and the bridge's services."""

    def __init__(self, speed=0.1):
        from geometry_msgs.msg import Twist
        from std_srvs.srv import Trigger
        self.node = rclpy.create_node('operator')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.publishing = True
        self.message = Twist()
        self.message.linear.x = speed
        self.last_published = None
        # STEADY: on the default (wall) clock, a WSL clock step back (~0.8 s every 32 s) delays the
        # next /cmd_vel by the step, and the bridge stops the wheel (2026-10-19, clock_step_probe).
        from rclpy.clock import Clock, ClockType
        self.node.create_timer(0.05, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.clients = {name: self.node.create_client(Trigger, name)
                        for name in ('/rover/arm', '/rover/arm_force', '/rover/estop')}
        self.request = Trigger.Request

    def _publish(self):
        if self.publishing:
            self.publisher.publish(self.message)
            self.last_published = time.monotonic()

    def spin_in_background(self):
        """Its own executor thread: /cmd_vel keeps its 20 Hz however busy the bridge's executor is
        (with one shared executor, the full suite's load delayed it past the 300 ms stop)."""
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.executor.spin()
        except ExternalShutdownException:              # rclpy shut down at the end of the test
            pass

    def call(self, executor, name, timeout=3.0):
        """executor: the one spinning this node, or None when it spins in the background."""
        def wait(condition):
            if executor is not None:
                return spin_until(executor, condition, timeout)
            end = time.monotonic() + timeout
            while time.monotonic() < end and not condition():
                time.sleep(0.01)
            return condition()
        client = self.clients[name]
        assert wait(client.service_is_ready)
        future = client.call_async(self.request())
        assert wait(future.done)
        return future.result()

    def arm(self, executor, attempts=4):
        """/rover/arm until accepted, as an operator would: on a loaded test machine the bridge's
        own freshness checks (STATUS 0.5 s, heartbeat 0.3 s) can refuse a first try, correctly."""
        for _ in range(attempts):
            response = self.call(executor, '/rover/arm', timeout=5.0)
            if response.success:
                return response
            time.sleep(0.5)
        return response


def test_driving_arm_then_the_heartbeat_stops_then_estops():
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        operator = Operator(speed=0.1)
        operator.spin_in_background()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        spin_for(executor, 1.0)                                         # HELLO, HELLO_ACK, STATUS, /cmd_vel
        assert board.motor.received['DRIVE'] == 0                       # nothing before an arm
        response = operator.arm(executor)
        assert response.success, response.message
        assert spin_until(executor, lambda: board.motor.fields()['motor_state'] == 1, 1.0)
        # STATUS echoes the DRIVE as received (2542); the motor runs on 51 units (2550).
        assert board.motor.fields() == {'motor_state': 1, 'latch_reason': 0, 'setpoint_cps': 2542}
        assert board.motor.units == 51
        assert bridge.safety.state == 'ARMED'
        before = board.motor.received['DRIVE']
        spin_for(executor, 0.5)
        assert board.motor.received['DRIVE'] - before > 0.5 * 50 * 0.8   # ~50 Hz
        assert board.motor.fields()['setpoint_cps'] == 2542, (board.motor.fields(), bridge.safety.snapshot(time.monotonic()))
        motor = dict(next(report['values'] for report in bridge.diagnostics.build(bridge.snapshot())
                          if report['name'] == 'rover_bridge: motor'))
        assert motor['setpoint cps (board echo of last DRIVE)'] == '2542'      # the STATUS the bridge got
        assert motor['setpoint cps (board, 50-cps units)'] == '2550'
        operator.publishing = False                                      # the operator goes quiet
        time.sleep(0.06)                                                 # (a publish already under way lands)
        quiet = operator.last_published                                  # what the bridge reacts to
        assert spin_until(executor, lambda: board.motor.fields()['setpoint_cps'] == 0, 1.0)
        stopped = time.monotonic() - quiet
        assert 0.25 < stopped < 0.5 and board.motor.fields()['motor_state'] == 0   # DRIVE 0, not latched
        assert spin_until(executor, lambda: board.motor.fields()['latch_reason'] == 6, 1.5)
        estopped = time.monotonic() - quiet
        assert 0.95 < estopped < 1.25
        assert spin_until(executor, lambda: board.motor.received['ESTOP'] == 3, 0.3)   # 3, 20 ms apart
        assert bridge.safety.state == 'DISARMED'
        drives = board.motor.received['DRIVE']
        spin_for(executor, 0.4)
        assert board.motor.received['DRIVE'] == drives                   # nothing after the ESTOP
        operator.publishing = True
        spin_for(executor, 0.3)
        assert board.motor.received['DRIVE'] == drives                   # a command doesn't re-arm
        assert operator.call(executor, '/rover/estop').success
        assert spin_until(executor, lambda: board.motor.received['ESTOP'] == 6, 1.0)
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_driving_steers_from_the_cfg_and_the_firmware_timeout_centres_after_an_estop():
    """angular.z reaches the board as steer_pct with the cfg's wheelbase and full lock (0.1 m/s,
    0.1 rad/s: atan(0.133 * 0.1 / 0.1) / 0.204 -> 65 %). After /rover/estop no DRIVE goes out
    (decision A, 2026-10-22): the steering stays where it was until the firmware's 500 ms timeout
    centres it."""
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        operator = Operator(speed=0.1)
        operator.message.angular.z = 0.1
        operator.spin_in_background()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        spin_for(executor, 1.0)
        assert board.motor.steer_percent == 0 and board.motor.received['DRIVE'] == 0
        response = operator.arm(executor)
        assert response.success, response.message
        assert spin_until(executor, lambda: board.motor.steer_percent == 65, 1.0), board.motor.steer_percent
        assert board.motor.fields()['setpoint_cps'] == 2542
        operator.message.linear.x = -0.1                                 # reversing: CCW is steering right
        assert spin_until(executor, lambda: board.motor.steer_percent == -65, 1.0), board.motor.steer_percent
        assert operator.call(executor, '/rover/estop').success
        assert spin_until(executor, lambda: board.motor.received['ESTOP'] == 3, 1.0)
        estopped = time.monotonic()
        drives = board.motor.received['DRIVE']
        assert board.motor.steer_percent == -65                          # an ESTOP doesn't touch it
        assert spin_until(executor, lambda: board.motor.steer_percent == 0, 1.0)
        assert 0.3 < time.monotonic() - estopped < 0.8                   # the firmware's 500 ms timeout
        spin_for(executor, 0.2)
        assert board.motor.received['DRIVE'] == drives                   # no DRIVE after the ESTOP
        bridge.close()
    finally:
        rclpy.try_shutdown()


class LogRecorder:
    """Stands in for the node's logger: records (level, text) and passes each line on (one call
    site per level: rclpy refuses one call site changing severity)."""

    def __init__(self, logger):
        self.logger, self.lines = logger, []

    def info(self, text):
        self.lines.append(('info', text))
        self.logger.info(text)

    def warn(self, text):
        self.lines.append(('warn', text))
        self.logger.warn(text)

    def error(self, text):
        self.lines.append(('error', text))
        self.logger.error(text)


def test_two_heartbeat_pauses_are_each_logged_and_counted():
    # 2026-10-19 bench: a drive's second pause (same text) went unlogged, and nothing said how long
    # a pause was. Two 0.5 s pauses, ~2 s apart (beyond the one-line-a-second limit).
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        log = LogRecorder(bridge.get_logger())
        bridge.get_logger = lambda: log
        operator = Operator(speed=0.1)
        operator.spin_in_background()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        spin_for(executor, 1.0)
        response = operator.arm(executor)
        assert response.success, response.message
        assert spin_until(executor, lambda: board.motor.fields()['motor_state'] == 1, 1.0)
        spin_for(executor, 0.5)
        for _pause in range(2):
            operator.publishing = False
            spin_for(executor, 0.5)
            operator.publishing = True
            spin_for(executor, 1.5)
        assert bridge.safety.state == 'ARMED' and board.motor.received['ESTOP'] == 0
        assert board.motor.fields()['motor_state'] == 1                           # driving again
        stops = [text for level, text in log.lines
                 if level == 'warn' and text == 'safety: no /cmd_vel for 300 ms: DRIVE 0 (still armed)']
        backs = [int(text.split('after ')[1].split(' ms')[0]) for level, text in log.lines
                 if level == 'info' and text.startswith('safety: /cmd_vel back after ')]
        assert len(stops) >= 2 and len(backs) >= 2, log.lines
        shot = bridge.safety.snapshot(time.monotonic())
        assert shot['counts']['heartbeat stops'] >= 2 and shot['stops_since_arm'] == shot['counts']['heartbeat stops']
        assert 0.45 < shot['gap_max'] < 0.8 and 0.45 < shot['last_stop_gap'] < 0.8, shot
        assert sum(450 < back < 800 for back in backs) >= 2, backs
        operator_report = dict(next(report['values'] for report in bridge.diagnostics.build(bridge.snapshot())
                                    if report['name'] == 'rover_bridge: operator'))
        assert operator_report['heartbeat stops since arm'] == str(shot['stops_since_arm'])
        assert 450 < float(operator_report['cmd_vel gap max ms (since arm)']) < 800
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_every_event_is_logged_even_two_in_one_poll_or_the_same_text_twice():
    # Polling last_event (the code before 2026-10-19) loses both: only the latest per poll, and
    # nothing when the text repeats.
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        log = LogRecorder(bridge.get_logger())
        bridge.get_logger = lambda: log
        bridge.stop_timers()                                     # this test polls by hand
        bridge.publish_pending()
        now = time.monotonic()
        bridge.safety.estop(now, 'first')
        bridge.safety.estop(now, 'second')                       # two events before one poll
        bridge.publish_pending()
        bridge.safety.estop(now, 'second')                       # the same text again
        bridge.publish_pending()
        safety_lines = [line for line in log.lines if line[1].startswith('safety: ESTOP')]
        assert safety_lines == [('warn', 'safety: ESTOP x3: first'), ('warn', 'safety: ESTOP x3: second'),
                                ('warn', 'safety: ESTOP x3: second')]
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_the_10_s_line_has_the_heartbeat_numbers_and_the_event_age():
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        log = LogRecorder(bridge.get_logger())
        bridge.get_logger = lambda: log
        bridge.stop_timers()
        time.sleep(0.3)                                          # a STATUS from the fake board
        bridge.publish_pending()
        assert bridge.last_status is not None
        bridge.safety.estop(time.monotonic() - 12.34, 'requested (/rover/estop)')
        bridge.safety.slow_gaps_since_arm = 3                    # not the stops (0)
        bridge.log_status()
        line = log.lines[-1][1]
        assert 'DISARMED cmd=0cps steer=0% sending=none sent DRIVE=0' in line, line
        assert 'heartbeat stops=0 (since arm 0, cmd_vel gap max n/a, last stop n/a, gaps>=150ms 3)' in line, line
        assert '(board tx_dropped=0); read_gap_max=' in line, line
        assert '(last event 12.3 s ago: ESTOP x3: requested (/rover/estop))' in line, line
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_the_timers_keep_firing_when_the_wall_clock_steps_back():
    # The node's ROS clock stands in for the wall clock: frozen at 1000 s, then stepped back 5 s.
    # A timer on rclpy's default clock would not fire again until it caught up; the bridge's run
    # on a steady clock (2026-10-19: WSL steps back ~0.8 s every 32 s, and the stall timed out an arm).
    from rclpy.time import Time
    rclpy.init(args=['--ros-args', '-p', 'status_period:=0.2', '-p', 'diagnostics_period:=0.2'])
    try:
        board = bridge_fakes.FakeBoardPort()
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        log = LogRecorder(bridge.get_logger())
        bridge.get_logger = lambda: log
        builds = []
        build = bridge.diagnostics.build
        bridge.diagnostics.build = lambda snapshot: builds.append(1) or build(snapshot)
        wall = bridge.get_clock()
        wall._set_ros_time_is_active(True)
        wall.set_ros_time_override(Time(seconds=1000))
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        spin_for(executor, 0.5)
        wall.set_ros_time_override(Time(seconds=995))                   # the step back
        imu, diagnostics, lines = bridge.published['imu'], len(builds), len(log.lines)
        spin_for(executor, 1.0)
        assert bridge.published['imu'] - imu > 50                        # publish_pending (100 Hz STATE)
        assert len(builds) - diagnostics >= 3                            # 0.2 s diagnostics
        status_lines = [text for _level, text in log.lines[lines:] if text.startswith('motor ')]
        assert len(status_lines) >= 3, log.lines[lines:]                 # 0.2 s summary
        bridge.close()
    finally:
        rclpy.try_shutdown()


def test_arm_is_refused_on_wall_power():
    rclpy.init()
    try:
        board = bridge_fakes.FakeBoardPort(status={'battery_state': 0, 'vbat_mv': 2})
        bridge = bridge_node.BridgeNode(open_port=lambda port, baud: board)
        operator = Operator()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(operator.node)
        spin_for(executor, 1.0)
        response = operator.call(executor, '/rover/arm')
        assert not response.success and response.message.startswith('battery NOT_PRESENT')
        assert not operator.call(executor, '/rover/arm_force').success
        spin_for(executor, 0.3)
        assert board.motor.received['DRIVE'] == 0
        bridge.close()
    finally:
        rclpy.try_shutdown()


def wait_for(condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.005)
    return condition()


def start_bridge_process(tmp_path, board):
    import subprocess
    import sys
    from pathlib import Path
    package = Path(bridge_node.__file__).resolve().parents[1]
    tools = package.parents[2] / 'tools'
    env = dict(os.environ, PYTHONPATH=f"{package}:{tools}:{os.environ.get('PYTHONPATH', '')}",
               ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    return subprocess.Popen([sys.executable, '-m', 'rover_bridge.node', '--ros-args', '-p', f'port:={board.path}',
                             '-p', 'raw_log:=disabled'], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


@pytest.mark.parametrize('how', ('SIGTERM', 'SIGKILL'))
def test_stopping_the_process_while_driving(tmp_path, how):
    """docker compose stop (SIGTERM): ESTOP x3 before the bridge goes, the board latches (6), exit 0,
    and the heartbeat is still fresh, so the ESTOP is the shutdown's. docker kill (SIGKILL): nothing
    can be sent; the board's watchdog latches (2) within ~0.5 s."""
    import signal
    board = bridge_fakes.PtyBoard()
    process = start_bridge_process(tmp_path, board)
    rclpy.init()
    try:
        operator = Operator(speed=0.1)
        operator.spin_in_background()                                       # /cmd_vel keeps coming throughout
        time.sleep(2.5)
        assert wait_for(lambda: 'rover_bridge' in operator.node.get_node_names(), 5.0), \
            'same-host DDS discovery failed (no anchor participant?): the operator never saw rover_bridge'
        response = operator.arm(None)
        assert response.success, response.message
        assert wait_for(lambda: board.motor.fields()['motor_state'] == 1, 2.0)
        time.sleep(0.3)
        stopped = time.monotonic()
        process.send_signal(getattr(signal, how))
        assert wait_for(lambda: board.motor.fields()['motor_state'] == 2, 2.0)
        latched_after = time.monotonic() - stopped
        output, _ = process.communicate(timeout=15)
    finally:
        if process.poll() is None:
            process.kill()
        rclpy.try_shutdown()
        board.stop()
    if how == 'SIGTERM':
        assert process.returncode == 0, output
        assert board.motor.fields()['latch_reason'] == 6 and board.motor.received['ESTOP'] == 3
        assert latched_after < 0.5 and 'shutdown: ESTOP x3 sent' in output and 'Traceback' not in output
        assert 'safety: armed: motor OFF' in output                    # events are logged as they happen
    else:
        assert process.returncode == -9
        assert board.motor.fields()['latch_reason'] == 2 and board.motor.received['ESTOP'] == 0
        assert 0.45 < latched_after < 0.8                                  # the firmware's 500 ms
