"""gyro_turn_check.TurnStats: the bookkeeping, without ROS."""
import math

import pytest

from gyro_turn_check import TurnStats


def test_integrals_gaps_and_elapsed_ratio():
    stats = TurnStats()
    # 1 s at 100 Hz, pi/2 rad/s; receive dt twice the header dt (a clock fit running slow).
    for index in range(101):
        stats.on_imu(index * 0.01, index * 0.02, math.pi / 2, (0.0, 0.0, 9.8))
    stats.on_imu(1.1, 2.2, 0.0, (0.0, 0.0, 9.8))  # a 0.1 s header gap
    assert stats.yaw_header == pytest.approx(math.pi / 2)
    assert stats.yaw_receive == pytest.approx(math.pi)
    assert stats.gaps == 1 and stats.max_header_dt == pytest.approx(0.1)
    assert stats.elapsed()[2] == pytest.approx(0.5)
    assert stats.accel_mean() == pytest.approx((0.0, 0.0, 9.8))


def test_accel_mean_only_over_the_first_two_seconds():
    stats = TurnStats()
    stats.on_imu(0.0, 0.0, 0.0, (0.0, 0.0, 9.8))
    stats.on_imu(2.0, 2.0, 0.0, (0.0, 0.0, 9.8))
    stats.on_imu(2.5, 2.5, 0.0, (5.0, 5.0, 0.0))
    assert stats.accel_mean() == pytest.approx((0.0, 0.0, 9.8))


def test_ekf_yaw_unwraps_through_pi():
    stats = TurnStats()
    for degrees in range(0, 400, 10):  # past 180 and 360
        half = math.radians(degrees) / 2.0
        stats.on_odom(math.sin(half), math.cos(half))
    assert stats.ekf_start == pytest.approx(0.0)
    assert stats.ekf_yaw == pytest.approx(math.radians(390))
