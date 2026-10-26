"""rover_bridge.conversions: STATE -> ROS units. Pure, runs anywhere (CI included)."""
import math

import pytest

import bridge_fakes
from rover_bridge import conversions
import rover_link


def test_imu_means_scaled_to_si():
    gyro, accel = conversions.imu_from_state(bridge_fakes.state_fields(10, gyro_x=-131 * 10, accel_x=4096 * 10))
    assert gyro == pytest.approx((math.radians(-2.0), 0.0, bridge_fakes.EXPECTED_GYRO_Z))
    assert accel == pytest.approx((9.80665 / 2, 0.0, bridge_fakes.EXPECTED_ACCEL_Z))


def test_imu_mean_uses_the_samples_actually_read():
    # 7 samples (a read failed in the period): the sum covers those 7.
    state = bridge_fakes.state_fields(10, imu_samples=7, gyro_z=655 * 7, accel_z=8192 * 7,
                                      imu_flags=rover_link.IMU_OK | rover_link.IMU_READ_FAILED)
    gyro, accel = conversions.imu_from_state(state)
    assert gyro[2] == pytest.approx(math.radians(10.0)) and accel[2] == pytest.approx(9.80665)


def test_no_imu_message_without_imu_or_samples():
    assert conversions.imu_from_state(bridge_fakes.state_fields(10, imu_flags=0)) is None
    assert conversions.imu_from_state(bridge_fakes.state_fields(10, imu_samples=0)) is None


def test_wheel_position_and_velocity_only_while_valid():
    position, velocity = conversions.wheel_from_state(bridge_fakes.state_fields(10))
    assert position == pytest.approx(2 * math.pi) and velocity == pytest.approx(math.pi)
    position, velocity = conversions.wheel_from_state(bridge_fakes.state_fields(10, count=-1024, enc_flags=0))
    assert position == pytest.approx(-math.pi / 2) and velocity is None
    flags = rover_link.ENC_INVALID_IN_PERIOD | rover_link.ENC_READ_FAILED
    assert conversions.wheel_from_state(bridge_fakes.state_fields(10, enc_flags=flags))[1] is None


def test_motor_text():
    assert conversions.motor_text(bridge_fakes.status_fields()) == 'LATCHED (HELLO)'
    assert conversions.motor_text(bridge_fakes.status_fields(latch_reason=1)) == 'LATCHED (BOOT)'
    assert conversions.motor_text(bridge_fakes.status_fields(motor_state=1, latch_reason=0)) == 'RUNNING'
    assert conversions.motor_text(bridge_fakes.status_fields(motor_state=3)) == 'FAULT'


def test_steering_from_status_is_the_angle_the_servo_has():
    lock = 0.204
    assert conversions.steering_from_status({'steer_state': 1, 'steer_percent': 50}, lock) == pytest.approx(0.102)
    assert conversions.steering_from_status({'steer_state': 1, 'steer_percent': -100}, lock) == pytest.approx(-0.204)
    assert conversions.steering_from_status({'steer_state': 2, 'steer_percent': 40}, lock) == 0.0   # timed out: centred
    assert conversions.steering_from_status({'steer_state': 0, 'steer_percent': 0}, lock) == 0.0    # no pulses yet
    assert conversions.steering_from_status({'steer_state': 3, 'steer_percent': 40}, lock) is None  # FAULT
    assert conversions.steering_from_status({'steer_state': 9, 'steer_percent': 40}, lock) is None
