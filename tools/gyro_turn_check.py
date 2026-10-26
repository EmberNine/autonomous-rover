#!/usr/bin/env python3
"""Where does a hand turn's yaw go missing? The gyro integrated two ways, next to the EKF.

Why: on 2026-10-22 a 90 deg CCW hand turn along a straight edge (real, checked)
gave /odom yaw 0.79 rad, and the scale chain from the chip register to the EKF
input has no error (65.5 LSB/dps throughout). This runs next to the EKF and
integrates /rover/imu/raw angular_velocity.z itself, so the loss can be pinned
to one stage.

While running, one line per second; on Ctrl-C a summary:
  gyro (header)   integral of z * dt, dt from consecutive header stamps (the
                  bridge's board-clock fit; what the EKF predicts with)
  gyro (receive)  the same with dt from this process's steady clock at receipt
  ekf             /odom yaw, 2*atan2(z, w), unwrapped (after a reset it starts at 0)
  imu             message count, the largest header dt, how many exceed 0.05 s
                  (the EKF holds the pose across those: OdometryFusionConfig
                  maxPredictStepSeconds)
  accel           mean linear_acceleration x/y/z over the first 2 s (keep still)
  elapsed         header time vs receive time, and their ratio (1.00 expected)

Reading it (turn 90 deg = 1.57 rad):
  gyro header   gyro receive   ekf     where the loss is
  ~1.57         ~1.57          ~0.79   inside the EKF (filter or its timing); look
                                       at the >0.05 s count first
  ~0.79         ~1.57          ~0.79   the clock fit: header stamps advance too slowly
                                       (elapsed ratio ~0.5)
  ~0.79         ~0.79          ~0.79   before ROS: the sensor or its mount; accel z
                                       should be ~9.8 (a tilt of ~60 deg halves z)

Usage (Pi, the repo is mounted read-only at /repo in the bridge container):
  docker exec -it rover-bridge /ros_entrypoint.sh python3 /repo/tools/gyro_turn_check.py
-it so Ctrl-C reaches it. Nothing is written; PYTHONDONTWRITEBYTECODE is set there.
"""
import math
import time

GAP_SECONDS = 0.05     # OdometryFusionConfig::maxPredictStepSeconds
REST_SECONDS = 2.0     # accel mean over this much header time from the first message


class TurnStats:
    """The bookkeeping, ROS-free: feed it IMU and odom samples, read the numbers."""

    def __init__(self):
        self.imu_count = 0
        self.first_header = self.last_header = None
        self.first_receive = self.last_receive = None
        self.yaw_header = 0.0
        self.yaw_receive = 0.0
        self.max_header_dt = 0.0
        self.gaps = 0
        self.accel_sum = [0.0, 0.0, 0.0]
        self.accel_count = 0
        self.odom_count = 0
        self.ekf_start = None
        self.ekf_yaw = None         # unwrapped
        self._ekf_last_raw = None

    def on_imu(self, header_seconds, receive_seconds, yaw_rate, accel):
        if self.last_header is not None:
            header_dt = header_seconds - self.last_header
            receive_dt = receive_seconds - self.last_receive
            self.yaw_header += yaw_rate * header_dt
            self.yaw_receive += yaw_rate * receive_dt
            self.max_header_dt = max(self.max_header_dt, header_dt)
            if header_dt > GAP_SECONDS:
                self.gaps += 1
        else:
            self.first_header, self.first_receive = header_seconds, receive_seconds
        self.last_header, self.last_receive = header_seconds, receive_seconds
        self.imu_count += 1
        if header_seconds - self.first_header <= REST_SECONDS:
            for axis in range(3):
                self.accel_sum[axis] += accel[axis]
            self.accel_count += 1

    def on_odom(self, z, w):
        raw = 2.0 * math.atan2(z, w)
        if self._ekf_last_raw is None:
            self.ekf_start = self.ekf_yaw = math.atan2(math.sin(raw), math.cos(raw))
        else:
            step = raw - self._ekf_last_raw
            self.ekf_yaw += math.atan2(math.sin(step), math.cos(step))
        self._ekf_last_raw = raw
        self.odom_count += 1

    def accel_mean(self):
        if not self.accel_count:
            return None
        return tuple(total / self.accel_count for total in self.accel_sum)

    def elapsed(self):
        """(header s, receive s, header / receive or None)."""
        if self.first_header is None:
            return 0.0, 0.0, None
        header = self.last_header - self.first_header
        receive = self.last_receive - self.first_receive
        return header, receive, (header / receive if receive > 0 else None)

    def line(self):
        ekf = 'n/a' if self.ekf_yaw is None else f'{self.ekf_yaw:+.3f}'
        return (f'gyro header {self.yaw_header:+.3f}  receive {self.yaw_receive:+.3f}  '
                f'ekf {ekf} rad  imu {self.imu_count}  gaps>{GAP_SECONDS}s {self.gaps}')

    def summary(self):
        header, receive, ratio = self.elapsed()
        accel = self.accel_mean()
        ekf = ('n/a (no /odom)' if self.ekf_yaw is None else
               f'{self.ekf_yaw:+.4f} rad ({math.degrees(self.ekf_yaw):+.1f} deg), '
               f'start {self.ekf_start:+.4f}, {self.odom_count} msgs')
        return '\n'.join([
            f'gyro (header dt)   {self.yaw_header:+.4f} rad ({math.degrees(self.yaw_header):+.1f} deg)',
            f'gyro (receive dt)  {self.yaw_receive:+.4f} rad ({math.degrees(self.yaw_receive):+.1f} deg)',
            f'ekf yaw            {ekf}',
            f'imu                {self.imu_count} msgs, max header dt {self.max_header_dt:.4f} s, '
            f'{self.gaps} over {GAP_SECONDS} s',
            'accel first 2 s    ' + ('n/a' if accel is None else
                                     'x {:+.3f}  y {:+.3f}  z {:+.3f} m/s^2'.format(*accel)),
            f'elapsed            header {header:.3f} s, receive {receive:.3f} s, ratio '
            + ('n/a' if ratio is None else f'{ratio:.4f}'),
        ])


def main():
    import rclpy
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    stats = TurnStats()
    last_print = [time.monotonic()]

    def on_imu(message):
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        acc = message.linear_acceleration
        stats.on_imu(stamp, now, message.angular_velocity.z, (acc.x, acc.y, acc.z))
        if now - last_print[0] >= 1.0:
            last_print[0] = now
            print(stats.line(), flush=True)

    def on_odom(message):
        orientation = message.pose.pose.orientation
        stats.on_odom(orientation.z, orientation.w)

    rclpy.init()
    node = rclpy.create_node('gyro_turn_check')
    node.create_subscription(Imu, '/rover/imu/raw', on_imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom, QoSProfile(depth=10))
    print('keep still for 2 s, then turn; Ctrl-C for the summary', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n' + stats.summary(), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
