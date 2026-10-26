"""rover_bridge.teleop: keys -> /cmd_vel setpoint, and the keyboard-link deadman. Pure, injected
clock (CI included). What must hold: a = wheels left at every speed (the bridge gives back the
percent shown), no level in the firmware's coast band, the 0.5 s stop latched at 0, the 2 s
ESTOP, end of input -> ESTOP, nothing published once finished."""
import math

import pytest

from rover_bridge import safety, teleop
from rover_bridge.teleop import LINK_ESTOP_SECONDS, LINK_STOP_SECONDS, SPEED_LEVELS, Teleop

WHEELBASE = 0.133                              # config/vehicle.cfg
FULL_LOCK = 0.204                              # config/vehicle.cfg (2026-10-22)
METERS_PER_TICK = 3.9336e-5                    # config/vehicle.cfg


def new(now=10.0):
    return Teleop(WHEELBASE, FULL_LOCK, now)


def keys(core, text, now=10.0):
    return core.on_bytes(text, now)


# --- speed -----------------------------------------------------------------------------------

def test_speed_levels_skip_the_coast_band_and_stop_at_015_and_minus_010():
    assert SPEED_LEVELS[0] == -0.10 and SPEED_LEVELS[-1] == 0.15 and 0.0 in SPEED_LEVELS
    for speed in SPEED_LEVELS:
        if speed:
            assert safety.speed_cps(speed, METERS_PER_TICK)[1] == '', speed      # >= 1000 cps, <= 5000
    assert safety.speed_cps(0.05, METERS_PER_TICK) == (1271, '')
    assert safety.speed_cps(0.15, METERS_PER_TICK) == (3813, '')


def test_w_and_s_step_through_zero_and_clamp_at_the_ends():
    core = new()
    seen = []
    for _ in range(8):
        keys(core, 'w')
        seen.append(core.speed)
    assert seen == [0.05, 0.075, 0.10, 0.125, 0.15, 0.15, 0.15, 0.15]
    seen = []
    for _ in range(12):
        keys(core, 's')
        seen.append(core.speed)
    assert seen == [0.125, 0.10, 0.075, 0.05, 0.0, -0.05, -0.075, -0.10, -0.10, -0.10, -0.10, -0.10]


# --- steering --------------------------------------------------------------------------------

def test_a_and_d_step_10_percent_and_clamp_and_c_centres():
    core = new()
    keys(core, 'aaa')
    assert core.steer_percent == 30
    keys(core, 'a' * 20)
    assert core.steer_percent == 100
    keys(core, 'd' * 25)
    assert core.steer_percent == -100
    keys(core, 'c')
    assert core.steer_percent == 0


def test_angular_z_is_v_tan_delta_over_the_wheelbase():
    assert teleop.angular_z(0.05, 30, WHEELBASE, FULL_LOCK) == pytest.approx(0.05 * math.tan(0.0612) / 0.133)
    assert teleop.angular_z(0.05, 30, WHEELBASE, FULL_LOCK) == pytest.approx(0.023036, abs=1e-6)


@pytest.mark.parametrize('speed', [s for s in SPEED_LEVELS if s])
def test_the_bridge_gives_back_the_percent_shown_at_every_speed(speed):
    """Not saturated at low speed: the bridge's atan(L * w / v) undoes v * tan(delta) / L."""
    for percent in range(-100, 101, 10):
        w = teleop.angular_z(speed, percent, WHEELBASE, FULL_LOCK)
        sent, what = safety.steer_percent(speed, w, WHEELBASE, FULL_LOCK)
        assert sent == percent, (speed, percent, sent)
        assert what == '' or abs(percent) == 100                # +-100 may round to just past full lock


def test_a_is_wheels_left_forward_and_reversing():
    core = new()
    keys(core, 'waaa')                                           # 0.05 m/s, 30 % left
    v, w = core.twist()
    assert v == 0.05 and w > 0                                   # forward: CCW
    assert safety.steer_percent(v, w, WHEELBASE, FULL_LOCK)[0] == 30
    keys(core, 'ss')                                             # -0.05 m/s, still 30 % left
    v, w = core.twist()
    assert v == -0.05 and w < 0                                  # reversing: CW, the wheels still left
    assert safety.steer_percent(v, w, WHEELBASE, FULL_LOCK)[0] == 30
    keys(core, 'd' * 6)                                          # 30 % right
    assert safety.steer_percent(*core.twist(), WHEELBASE, FULL_LOCK)[0] == -30


def test_at_v_0_angular_z_is_0_and_the_status_says_the_steering_waits():
    core = new()
    keys(core, 'aaa')
    assert core.twist() == (0.0, 0.0)                            # the bridge holds its last steering
    assert 'applies when moving' in core.status_text(10.0, 1 / METERS_PER_TICK)
    keys(core, 'w')
    assert core.twist()[1] > 0
    assert 'applies when moving' not in core.status_text(10.0, 1 / METERS_PER_TICK)


# --- space, x, r, q, end of input ------------------------------------------------------------

def test_space_stops_and_centres_without_a_request():
    core = new()
    assert keys(core, 'wwaa ') == []
    assert core.twist() == (0.0, 0.0) and core.steer_percent == 0 and core.finished is None


def test_x_stops_centres_and_asks_for_an_estop_but_keeps_running():
    core = new()
    assert keys(core, 'wwaax') == ['estop']
    assert core.twist() == (0.0, 0.0) and core.finished is None
    assert keys(core, 'w') == [] and core.speed == 0.05          # still the operator's


def test_r_asks_for_an_arm():
    assert keys(new(), 'r') == ['arm']


def test_q_stops_asks_for_an_estop_and_nothing_is_published_after_it():
    core = new()
    assert keys(core, 'wwq') == ['estop']
    assert core.twist() is None and core.finished == 'quit (q)'
    assert keys(core, 'w') == [] and core.tick(20.0) == []        # finished: keys and the deadman do nothing
    assert core.twist() is None


def test_end_of_input_and_a_signal_finish_with_an_estop():
    core = new()
    keys(core, 'ww')
    assert core.on_eof(10.1) == ['estop']
    assert core.twist() is None and 'end of input' in core.finished
    assert core.on_eof(10.2) == []                               # once
    other = new()
    assert other.on_signal('SIGHUP') == ['estop'] and other.finished == 'SIGHUP'


def test_unknown_bytes_are_ignored_but_feed_the_deadman():
    core = new()
    keys(core, 'w')
    assert keys(core, 'zZ\n', 10.4) == []
    assert core.counts['ignored'] == 3 and core.speed == 0.05
    assert core.tick(10.8) == [] and not core.link_stopped       # 0.4 s since the last byte


# --- the deadman -----------------------------------------------------------------------------

def test_no_byte_for_half_a_second_stops_and_centres_but_keeps_publishing():
    core = new()
    keys(core, 'wwaa', 10.0)
    assert core.tick(10.0 + LINK_STOP_SECONDS) == [] and not core.link_stopped     # not yet: > 0.5 s
    assert core.tick(10.52) == []
    assert core.link_stopped and core.twist() == (0.0, 0.0) and core.steer_percent == 0
    assert core.finished is None                                 # still armed, still the heartbeat
    assert core.counts['link stops'] == 1
    assert core.take_events() == ['no keyboard byte for 520 ms: speed 0, centre (still armed; w to drive)']
    core.tick(10.6)
    assert core.counts['link stops'] == 1 and core.take_events() == []           # once per gap


def test_keepalives_after_a_stop_dont_drive_again_w_does():
    core = new()
    keys(core, 'www', 10.0)
    core.tick(10.6)
    core.take_events()
    assert keys(core, '.', 10.7) == []
    assert not core.link_stopped and core.speed == 0.0          # latched at 0
    assert core.take_events() == ['keyboard link back after 700 ms: speed stays 0 (w to drive)']
    for at in (10.8, 10.9, 11.0):
        keys(core, '.', at)
        core.tick(at)
    assert core.speed == 0.0
    keys(core, 'w', 11.05)
    assert core.speed == 0.05


def test_no_byte_for_two_seconds_estops_and_stops_publishing():
    core = new()
    keys(core, 'ww', 10.0)
    core.tick(10.6)
    assert core.tick(10.0 + LINK_ESTOP_SECONDS) == [] and core.finished is None
    assert core.tick(12.02) == ['estop']
    assert core.twist() is None and core.finished == 'no keyboard byte for 2020 ms'
    assert core.gap_max == pytest.approx(2.02)
    assert core.tick(13.0) == []                                 # once


def test_the_deadman_counts_from_the_start():
    core = new(10.0)                                             # no byte ever (the forwarder never came)
    core.tick(10.51)
    assert core.link_stopped
    assert core.tick(12.01) == ['estop'] and core.twist() is None


def test_keepalives_every_100_ms_keep_it_driving():
    core = new()
    keys(core, 'wwwww', 10.0)
    for step in range(1, 101):                                   # 10 s
        now = 10.0 + step * 0.1
        keys(core, '.', now)
        core.tick(now + 0.05)
    assert core.speed == 0.15 and not core.link_stopped and core.counts['link stops'] == 0
    assert core.counts['keepalives'] == 100


def test_the_status_line_shows_the_last_byte_and_the_longest_gap_in_10_s():
    core = new()
    keys(core, '.', 10.0)
    keys(core, '.', 10.4)                                        # a 400 ms gap
    keys(core, '.', 10.5)
    text = core.status_text(10.53, 1 / METERS_PER_TICK)
    assert 'last byte 30 ms' in text and 'max 400 ms/10 s' in text and 'link ok' in text
    for at in (12.0, 15.0, 18.0, 20.45):
        keys(core, '.', at)
    assert 'max 3000 ms/10 s' in core.status_text(20.5, 1 / METERS_PER_TICK)
    for step in range(1, 101):                                   # then 10 s of keepalives every 100 ms
        keys(core, '.', 20.45 + step * 0.1)
    assert 'max 100 ms/10 s' in core.status_text(30.5, 1 / METERS_PER_TICK)  # the long gaps aged out
    assert core.gap_max == pytest.approx(3.0)                    # the final line's: since the start


def test_the_status_line_shows_a_link_stop():
    core = new()
    core.tick(10.6)
    assert 'LINK STOP (w to drive)' in core.status_text(10.6, 1 / METERS_PER_TICK)
