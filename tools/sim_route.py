#!/usr/bin/env python3
"""Drive one Week 6 route in Gazebo and compare the EKF with the sim's ground truth.

Needs the sim with the EKF running (ros2 launch rover_gazebo sim_ekf.launch.py). On sim time:
waits for the EKF's first /odom (it has started), 1 s at rest, /rover/ekf/reset, the route
on /cmd_vel at 20 Hz, 3 s of zeros, then prints the summary. The summary is the record.

  straight      2 m: 0.10 m/s for 20 s
  half_circle   full lock left at 0.05 m/s for pi / yaw rate (the bridge's full lock:
                angular.z = v tan(max_steering_angle) / wheelbase, as teleop sends it)

Truth is /odometry/ground_truth (the physics engine's base_link, the rear-axle centre), taken
in the frame of the pose at the reset, so it lines up with the EKF's (0, 0, 0).

Pass tolerances (docs/kalman-filter.md, real vs sim):
  straight      sim EKF distance error vs truth within 1 %; and within 3 % of the real run's
                (-1.5 %), the bench's pass
  half_circle   truth's effective radius within 10 % of the model's L / tan(delta): set from
                the observed real spread, 0.595-0.670 m (-7.5 % to +4.2 %); sim EKF heading
                within 0.03 rad and end point within 3 cm of truth; real and sim both within
                0.1 rad and 10 cm

  python3 tools/sim_route.py straight|half_circle [--vehicle-cfg config/vehicle.cfg]
"""
import argparse
import math
from pathlib import Path

REAL_STRAIGHT_ERROR = -0.015      # 2026-10-25: tape 2.073 m, EKF 2.041 m
REAL_HEADING_ERROR = -0.072       # 2026-10-25 half circle: EKF 2.897 rad vs gyro 2.969 rad
REAL_END_POINT_ERROR = 0.072      # 2026-10-25 half circle: EKF 1.258 m vs tape 1.186 m
RATE = 20.0                       # /cmd_vel, Hz
REST_BEFORE, REST_AFTER = 1.0, 3.0


def vehicle_value(path, key):
    """A number from vehicle.cfg ('key = value', '#' comments)."""
    for raw_line in Path(path).read_text(encoding='utf-8').splitlines():
        line = raw_line.split('#', 1)[0].strip()
        if '=' in line:
            name, value = (part.strip() for part in line.split('=', 1))
            if name == key:
                return float(value)
    raise KeyError(f'{key} not in {path}')


def route(name, wheelbase, max_angle):
    """(linear.x, angular.z, seconds) of a route."""
    if name == 'straight':
        return 0.10, 0.0, 20.0
    speed = 0.05
    yaw_rate = speed * math.tan(max_angle) / wheelbase
    return speed, yaw_rate, math.pi / yaw_rate


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Track:
    """A pose stream relative to an origin (default: its first pose): position in that frame,
    heading unwrapped."""

    def __init__(self, origin=None):
        self.origin = origin
        self.x = self.y = self.heading = 0.0
        self._last_yaw = None if origin is None else origin[2]

    def add(self, x, y, yaw):
        if self.origin is None:
            self.origin = (x, y, yaw)
        x0, y0, yaw0 = self.origin
        cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
        self.x = cos0 * (x - x0) + sin0 * (y - y0)
        self.y = -sin0 * (x - x0) + cos0 * (y - y0)
        if self._last_yaw is not None:
            self.heading += wrap(yaw - self._last_yaw)
        self._last_yaw = yaw


def effective_radius(chord, heading_change):
    """Radius of the arc with this chord and turn (rear-axle centre)."""
    return chord / (2.0 * abs(math.sin(heading_change / 2.0)))


def check(label, ok):
    return f'  {"PASS" if ok else "FAIL"}  {label}'


def summary(name, truth, ekf, wheelbase, max_angle):
    lines = [f'route {name}',
             f'  truth  x {truth.x:+.4f} m  y {truth.y:+.4f} m  heading {truth.heading:+.4f} rad',
             f'  ekf    x {ekf.x:+.4f} m  y {ekf.y:+.4f} m  heading {ekf.heading:+.4f} rad']
    if name == 'straight':
        truth_distance, ekf_distance = math.hypot(truth.x, truth.y), math.hypot(ekf.x, ekf.y)
        error = (ekf_distance - truth_distance) / truth_distance
        lines += [f'  distance truth {truth_distance:.4f} m, ekf {ekf_distance:.4f} m, error {error * 100:+.2f} % '
                  f'(real {REAL_STRAIGHT_ERROR * 100:+.1f} %)',
                  check('sim EKF distance within 1 % of truth', abs(error) <= 0.01),
                  check('sim vs real distance error within 3 %', abs(error - REAL_STRAIGHT_ERROR) <= 0.03)]
        return '\n'.join(lines)
    model_radius = wheelbase / math.tan(max_angle)
    radius = effective_radius(math.hypot(truth.x, truth.y), truth.heading)
    heading_error = ekf.heading - truth.heading
    end_error = math.hypot(ekf.x - truth.x, ekf.y - truth.y)
    lines += [f'  radius truth {radius:.4f} m vs model {model_radius:.4f} m ({(radius / model_radius - 1) * 100:+.1f} %; '
              f'real 0.595-0.670 m)',
              f'  ekf heading error {heading_error:+.4f} rad (real {REAL_HEADING_ERROR:+.3f}), '
              f'end point error {end_error * 100:.1f} cm (real {REAL_END_POINT_ERROR * 100:.1f})',
              check('truth radius within 10 % of the model', abs(radius / model_radius - 1) <= 0.10),
              check('sim EKF heading within 0.03 rad of truth', abs(heading_error) <= 0.03),
              check('sim EKF end point within 3 cm of truth', end_error <= 0.03),
              check('real and sim heading within 0.1 rad',
                    abs(heading_error) <= 0.1 and abs(REAL_HEADING_ERROR) <= 0.1),
              check('real and sim end point within 10 cm', end_error <= 0.10 and REAL_END_POINT_ERROR <= 0.10)]
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('route', choices=('straight', 'half_circle'))
    parser.add_argument('--vehicle-cfg', default=str(Path(__file__).resolve().parents[1] / 'config' / 'vehicle.cfg'))
    args = parser.parse_args()
    wheelbase = vehicle_value(args.vehicle_cfg, 'wheelbase_meters')
    max_angle = vehicle_value(args.vehicle_cfg, 'max_steering_angle_radians')
    speed, yaw_rate, seconds = route(args.route, wheelbase, max_angle)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('sim_route', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest = {}
    tracks = {'truth': None, 'ekf': None}

    def on_odometry(key):
        def callback(message):
            pose = message.pose.pose
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            latest[key] = (pose.position.x, pose.position.y, yaw)
            if tracks[key] is not None:
                tracks[key].add(*latest[key])
        return callback

    node.create_subscription(Odometry, '/odometry/ground_truth', on_odometry('truth'), 10)
    node.create_subscription(Odometry, '/odom', on_odometry('ekf'), 10)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    reset = node.create_client(Trigger, '/rover/ekf/reset')

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def drive(linear_x, angular_z, duration):
        command = Twist()
        command.linear.x, command.angular.z = linear_x, angular_z
        end = now() + duration
        next_send = now()
        while rclpy.ok() and now() < end:
            if now() >= next_send:
                publisher.publish(command)
                next_send += 1.0 / RATE
            rclpy.spin_once(node, timeout_sec=0.005)

    try:
        print('waiting for the sim clock, ground truth and the EKF (/odom: it has started)', flush=True)
        while rclpy.ok() and not ('truth' in latest and 'ekf' in latest and now() > 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        drive(0.0, 0.0, REST_BEFORE)
        if not reset.wait_for_service(timeout_sec=10.0):
            raise SystemExit('/rover/ekf/reset not available')
        future = reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future.result() is None or not future.result().success:
            raise SystemExit(f'EKF reset failed: {future.result()}')
        tracks['truth'], tracks['ekf'] = Track(), Track(origin=(0.0, 0.0, 0.0))   # the reset's (0, 0, 0)
        tracks['truth'].add(*latest['truth'])
        drive(0.0, 0.0, 0.2)
        print(f'{args.route}: {speed} m/s, angular.z {yaw_rate:.4f} rad/s for {seconds:.2f} s', flush=True)
        drive(speed, yaw_rate, seconds)
        drive(0.0, 0.0, REST_AFTER)
        print(summary(args.route, tracks['truth'], tracks['ekf'], wheelbase, max_angle), flush=True)
    except KeyboardInterrupt:
        publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
