"""The bridge's stand-in for the Gazebo model: what the EKF reads, from the sim's topics.

  in   /cmd_vel              geometry_msgs/Twist, as the bridge takes it
       /joint_states         sensor_msgs/JointState from Gazebo (wheel joints in rad/s)
  out  /rover/joint_states   rear_left_wheel_joint, in the bridge's units: velocity [rad/s] of
                             an encoder wheel whose meters_per_encoder_tick is the cfg's, so the
                             EKF's conversion gives the sim wheel's surface speed (radius
                             tire_diameter_meters / 2, the URDF's)
       /rover/steering       10 Hz, joint 'steering' [rad], the angle the real servo would have

Steering follows the bridge (safety.py) on the same /cmd_vel: the bicycle model's angle from
linear.x and angular.z, clamped to full lock and rounded to whole percent; held while the
speed is sent as 0 (linear.x 0 or below the minimum: no division, a car can't turn in place);
centred for a non-finite command, and when /cmd_vel is older than the heartbeat stop (0.3 s),
which also centres the held value, as the bridge's heartbeat stop does. Before any /cmd_vel:
0 (the servo OFF). /imu comes straight from Gazebo's IMU sensor (bridge.yaml).

Stamps are sim time (use_sim_time true in the launch file), like Gazebo's own messages.
"""
import math

from rover_bridge import conversions, safety

WHEEL_JOINT = 'rear_left_wheel_joint'
STEERING_PERIOD = 0.1   # s: STATUS's 10 Hz


def encoder_velocity(joint_velocity, wheel_radius, meters_per_radian):
    """The bridge's encoder-wheel rad/s for a sim wheel turning at joint_velocity rad/s."""
    return joint_velocity * wheel_radius / meters_per_radian


class SimSteering:
    """The steering angle the bridge would have the servo at, from /cmd_vel and its age."""

    def __init__(self, meters_per_tick, wheelbase, max_angle):
        self.meters_per_tick = meters_per_tick
        self.wheelbase = wheelbase
        self.max_angle = max_angle
        self.percent = safety.CENTRE
        self.command_at = None

    def on_cmd_vel(self, linear_x, angular_z, now):
        if self.command_at is not None and now - self.command_at > safety.HEARTBEAT_STOP:
            self.percent = safety.CENTRE                          # the heartbeat stop had centred it
        cps, _ = safety.speed_cps(linear_x, self.meters_per_tick)
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self.percent = safety.CENTRE
        elif cps != 0:
            self.percent, _ = safety.steer_percent(linear_x, angular_z, self.wheelbase, self.max_angle)
        self.command_at = now

    def angle(self, now):
        if self.command_at is None or now - self.command_at > safety.HEARTBEAT_STOP:
            return 0.0
        return self.percent / 100.0 * self.max_angle


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    class SimAdapter(Node):

        def __init__(self):
            super().__init__('rover_sim_adapter')
            cfg = self.declare_parameter('vehicle_cfg', '/repo/config/vehicle.cfg').value
            meters_per_tick = conversions.vehicle_value(cfg, 'meters_per_encoder_tick')
            self.meters_per_radian = meters_per_tick / conversions.RADIANS_PER_COUNT
            self.wheel_radius = conversions.vehicle_value(cfg, 'tire_diameter_meters') / 2.0
            self.steering = SimSteering(meters_per_tick, conversions.vehicle_value(cfg, 'wheelbase_meters'),
                                        conversions.vehicle_value(cfg, 'max_steering_angle_radians'))
            self.joints = self.create_publisher(JointState, '/rover/joint_states', qos_profile_sensor_data)
            self.steering_publisher = self.create_publisher(JointState, '/rover/steering', qos_profile_sensor_data)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
            self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
            self.create_timer(STEERING_PERIOD, self.publish_steering)
            self.get_logger().info(f'{cfg}: wheel radius {self.wheel_radius} m, '
                                   f'{meters_per_tick:.4e} m per count')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_cmd_vel(self, message):
            self.steering.on_cmd_vel(message.linear.x, message.angular.z, self.now())

        def on_joint_states(self, message):
            if WHEEL_JOINT not in message.name:
                return
            index = message.name.index(WHEEL_JOINT)
            out = JointState()
            out.header.stamp = message.header.stamp
            out.name = [WHEEL_JOINT]
            if index < len(message.position):
                out.position = [encoder_velocity(message.position[index], self.wheel_radius, self.meters_per_radian)]
            if index < len(message.velocity):
                out.velocity = [encoder_velocity(message.velocity[index], self.wheel_radius, self.meters_per_radian)]
            self.joints.publish(out)

        def publish_steering(self):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = ['steering']
            message.position = [self.steering.angle(self.now())]
            self.steering_publisher.publish(message)

    rclpy.init(args=args)
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
