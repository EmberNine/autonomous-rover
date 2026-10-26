"""rover_bridge.sim_adapter: the sim's encoder and steering in the bridge's terms. Pure, no ROS."""
import math

import pytest

from rover_bridge import safety
from rover_bridge.sim_adapter import SimSteering, encoder_velocity

TICK = 3.9336e-5
WHEELBASE = 0.133
LOCK = 0.204


def steering():
    return SimSteering(TICK, WHEELBASE, LOCK)


def test_encoder_velocity_gives_the_sim_wheels_surface_speed_through_the_ekfs_conversion():
    meters_per_radian = TICK / (2 * math.pi / 4096)
    velocity = encoder_velocity(10.0, 0.027, meters_per_radian)
    assert velocity * meters_per_radian == pytest.approx(0.27)


def test_full_lock_is_the_angle_and_beyond_it_clamps():
    sim = steering()
    sim.on_cmd_vel(0.05, 0.05 * math.tan(LOCK) / WHEELBASE, 10.0)
    assert sim.angle(10.05) == pytest.approx(LOCK)
    sim.on_cmd_vel(0.05, 5.0, 10.1)
    assert sim.angle(10.1) == pytest.approx(LOCK)


def test_rounded_to_whole_percent_as_the_bridge_sends_it():
    sim = steering()
    sim.on_cmd_vel(0.1, 0.1 * math.tan(0.333 * LOCK) / WHEELBASE, 0.0)
    assert sim.angle(0.0) == pytest.approx(0.33 * LOCK)


def test_zero_speed_holds_the_last_angle_without_dividing():
    sim = steering()
    sim.on_cmd_vel(0.05, 0.05 * math.tan(LOCK) / WHEELBASE, 0.0)
    sim.on_cmd_vel(0.0, 0.5, 0.1)                       # v = 0 with a turn asked: held, no ZeroDivisionError
    assert sim.angle(0.1) == pytest.approx(LOCK)
    sim.on_cmd_vel(0.0, 0.0, 0.2)
    assert sim.angle(0.2) == pytest.approx(LOCK)
    sim.on_cmd_vel(TICK * (safety.MIN_SPEED_CPS - 1), 0.5, 0.3)   # below the minimum: sent as 0, held
    assert sim.angle(0.3) == pytest.approx(LOCK)


def test_zero_speed_from_the_start_is_centred():
    sim = steering()
    assert sim.angle(0.0) == 0.0                         # no /cmd_vel yet: OFF
    sim.on_cmd_vel(0.0, 1.0, 0.0)
    assert sim.angle(0.0) == 0.0


def test_a_stale_command_centres_and_forgets_the_held_angle():
    sim = steering()
    sim.on_cmd_vel(0.05, 0.05 * math.tan(LOCK) / WHEELBASE, 0.0)
    assert sim.angle(0.3) == pytest.approx(LOCK)
    assert sim.angle(0.31) == 0.0                        # the heartbeat stop
    sim.on_cmd_vel(0.0, 0.0, 1.0)                        # back, stopped: nothing held any more
    assert sim.angle(1.0) == 0.0


def test_non_finite_centres():
    sim = steering()
    sim.on_cmd_vel(0.05, 0.05 * math.tan(LOCK) / WHEELBASE, 0.0)
    sim.on_cmd_vel(float('nan'), 0.0, 0.1)
    assert sim.angle(0.1) == 0.0
