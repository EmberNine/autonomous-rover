"""sim_route: the route timing and the metrics, without ROS."""
import math

import pytest

import sim_route
from sim_route import Track, effective_radius, route, summary

WHEELBASE, LOCK = 0.133, 0.204


def test_half_circle_turns_pi_at_full_lock():
    speed, yaw_rate, seconds = route('half_circle', WHEELBASE, LOCK)
    assert speed == 0.05 and yaw_rate * seconds == pytest.approx(math.pi)
    assert route('straight', WHEELBASE, LOCK) == (0.10, 0.0, 20.0)


def test_track_is_in_the_start_frame_and_unwraps():
    track = Track()
    track.add(5.0, 5.0, math.pi / 2)              # facing +y
    track.add(5.0, 6.0, math.pi - 0.1)
    track.add(5.0, 6.0, -math.pi + 0.1)           # across +-pi
    assert (track.x, track.y) == pytest.approx((1.0, 0.0))
    assert track.heading == pytest.approx(math.pi / 2 + 0.1)


def test_effective_radius_from_chord_and_turn():
    assert effective_radius(1.29, math.pi) == pytest.approx(0.645)
    assert effective_radius(math.sqrt(2) * 0.5, math.pi / 2) == pytest.approx(0.5)


def half_circle(radius, heading):
    track = Track(origin=(0.0, 0.0, 0.0))
    track.add(0.0, 0.0, 0.0)
    track.x, track.y, track.heading = 0.0, 2 * radius, heading
    return track


def test_half_circle_passes_with_the_real_spread_and_fails_beyond_ten_percent():
    model = WHEELBASE / math.tan(LOCK)
    for factor, verdict in ((0.925, 'PASS'), (1.042, 'PASS'), (0.89, 'FAIL')):
        truth = half_circle(model * factor, math.pi)
        text = summary('half_circle', truth, half_circle(model * factor, math.pi), WHEELBASE, LOCK)
        assert f'{verdict}  truth radius within 10 %' in text, factor


def test_straight_error_against_truth_and_the_real_run():
    truth, ekf = Track(origin=(0.0, 0.0, 0.0)), Track(origin=(0.0, 0.0, 0.0))
    truth.x, ekf.x = 2.0, 2.0 * 0.995
    text = summary('straight', truth, ekf, WHEELBASE, LOCK)
    assert 'PASS  sim EKF distance within 1 %' in text and 'PASS  sim vs real' in text
    ekf.x = 2.0 * 1.02
    assert 'FAIL  sim EKF distance within 1 %' in summary('straight', truth, ekf, WHEELBASE, LOCK)
    assert sim_route.REAL_STRAIGHT_ERROR == -0.015
