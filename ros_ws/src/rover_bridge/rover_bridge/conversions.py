"""STATE fields (tools/rover_link.py) -> ROS units. Pure: no ROS, no I/O.

Scales are the firmware's configuration, not measurements:
  gyro  +/-500 dps, 65.5 LSB per deg/s  (firmware/Core/Src/icm42688.c, ICM42688_GYRO_FS_500DPS)
  accel +/-4 g,     8192 LSB per g      (ICM42688_ACCEL_FS_4G)
  encoder           4096 counts per wheel revolution (AS5600, config/vehicle.cfg)
STATE carries IMU sums over exactly the samples since the previous STATE
(firmware/Core/Inc/link_protocol.h, LinkStateMessage), so sum / imu_samples
is the period's mean. Axes are the IMU chip's own; how the chip sits on the
chassis is not measured yet (a static transform comes with calibration).
"""
import math

from rover_bridge import _tools  # noqa: F401  (tools/ on sys.path)
import rover_link  # noqa: E402

GYRO_LSB_PER_DPS = 65.5
ACCEL_LSB_PER_G = 8192.0
STANDARD_GRAVITY = 9.80665           # m/s^2 per g
ENCODER_COUNTS_PER_REVOLUTION = 4096
RADIANS_PER_COUNT = 2.0 * math.pi / ENCODER_COUNTS_PER_REVOLUTION


# SteeringState (firmware steering_control.h): what the servo is doing, from STATUS.
STEERING_OFF, STEERING_ACTIVE, STEERING_TIMED_OUT, STEERING_FAULT = range(4)


def steering_from_status(status, max_steering_angle):
    """The steering angle the servo has (rad, + = left, the bicycle model's), from STATUS, for
    /rover/steering: ACTIVE: steer_percent / 100 * full lock; TIMED_OUT (centred by the
    firmware's timeout) and OFF (no pulses since boot, wheels straight): 0; FAULT (no pulses
    whatever is commanded) or an unknown state: None, nothing to publish. Percent -> angle is
    treated as linear (only full lock was measured)."""
    state = status['steer_state']
    if state == STEERING_ACTIVE:
        return status['steer_percent'] / 100.0 * max_steering_angle
    if state in (STEERING_OFF, STEERING_TIMED_OUT):
        return 0.0
    return None


def imu_from_state(state):
    """((gx, gy, gz) rad/s, (ax, ay, az) m/s^2), the means over the STATE period,
    or None when there is nothing to publish: IMU init failed (no IMU_OK) or no
    sample in the period. A period with a failed read (IMU_READ_FAILED) still
    gives the mean of the samples that were read."""
    samples = state['imu_samples']
    if not state['imu_flags'] & rover_link.IMU_OK or samples == 0:
        return None
    gyro = tuple(math.radians(state[key] / samples / GYRO_LSB_PER_DPS) for key in ('gyro_x', 'gyro_y', 'gyro_z'))
    accel = tuple(state[key] / samples / ACCEL_LSB_PER_G * STANDARD_GRAVITY for key in ('accel_x', 'accel_y', 'accel_z'))
    return gyro, accel


def wheel_from_state(state):
    """(position rad, velocity rad/s or None) of the encoder wheel. position is the
    corrected odometer count as an angle (cumulative; it survives lost frames);
    velocity only while the speed estimate is valid (ENC_VALID_NOW), else None."""
    position = state['count'] * RADIANS_PER_COUNT
    if not state['enc_flags'] & rover_link.ENC_VALID_NOW:
        return position, None
    return position, state['speed_cps'] * RADIANS_PER_COUNT


def vehicle_value(path, key):
    """A number from config/vehicle.cfg (the C++ core's rules: '#' starts a comment, 'key = value'
    per line, unknown keys ignored). Raises ValueError if the key is missing or not positive."""
    with open(path, encoding='utf-8') as file:
        for raw_line in file:
            line = raw_line.split('#', 1)[0].strip(' \t\r\n')
            if not line or '=' not in line:
                continue
            name, value = (part.strip(' \t\r\n') for part in line.split('=', 1))
            if name == key:
                number = float(value)
                if not number > 0:
                    raise ValueError(f'{key} = {value} in {path}: must be positive')
                return number
    raise ValueError(f'{key} not in {path}')


def motor_text(status):
    """'LATCHED (HELLO)' from a STATUS: motor state and, when latched, the reason."""
    states = ('OFF', 'RUNNING', 'LATCHED', 'FAULT')
    state = status['motor_state']
    text = states[state] if state < len(states) else f'STATE_{state}'
    if state == 2:
        reason = status['latch_reason']
        names = rover_link.LATCH_REASON_NAMES
        text += f' ({names[reason] if reason < len(names) else reason})'
    return text
