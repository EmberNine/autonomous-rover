"""rover_bridge.safety: the state machine that decides every DRIVE and ESTOP. Pure, injected clock
(CI included). What must hold: never a DRIVE unless armed; the heartbeat's two stages; every arm
refusal; ESTOP x3 on estop(), the heartbeat's 1 s and shutdown(); nothing after shutdown()."""
import math

import pytest

import bridge_fakes
from rover_bridge import safety
from rover_bridge.safety import ARMED, ARMING, DISARMED, Safety

METERS_PER_TICK = 3.9336e-5                    # config/vehicle.cfg
WHEELBASE = 0.133                              # config/vehicle.cfg
FULL_LOCK = 0.204                              # config/vehicle.cfg (2026-10-22)


def new_safety():
    return Safety(METERS_PER_TICK, WHEELBASE, FULL_LOCK)


def status(**changes):
    """A STATUS of a booted board on the LiPo: LATCHED (BOOT), battery OK."""
    return bridge_fakes.status_fields(**dict({'motor_state': 2, 'latch_reason': 1, 'battery_state': 1,
                                              'vbat_mv': 7700}, **changes))


def ready(now=10.0, **status_changes):
    """HELLO_ACK in, a fresh STATUS, a fresh /cmd_vel at 0.1 m/s: ready to arm."""
    core = new_safety()
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), now)
    core.on_status(status(**status_changes), now)
    core.on_cmd_vel(0.1, 0.0, now)
    return core


def armed(now=10.0):
    core = ready(now)
    assert core.arm(now) == (True, core.arm(now)[1])
    assert core.tick(now + 0.02) == [('DRIVE', 0, 0, True)]          # the arm DRIVE
    core.on_status(status(motor_state=0, latch_reason=0), now + 0.03)  # the board: OFF
    assert core.state == ARMED
    return core


def ticks(core, start, end, cmd_every=None):
    """Every 20 ms from start to end: the frames tick() returns; /cmd_vel at 0.1 m/s every cmd_every s."""
    out = []
    now, next_cmd = start, start
    while now < end - 1e-9:
        if cmd_every is not None and now >= next_cmd:
            core.on_cmd_vel(0.1, 0.0, now)
            next_cmd += cmd_every
        out.append((round(now, 3), core.tick(now)))
        now += 0.02
    return out


# --- speed ------------------------------------------------------------------------------

def test_speed_conversion_min_and_clamp():
    assert safety.speed_cps(0.1, METERS_PER_TICK) == (2542, '')
    assert safety.speed_cps(-0.1, METERS_PER_TICK) == (-2542, '')
    assert safety.speed_cps(0.0, METERS_PER_TICK) == (0, '')
    assert safety.speed_cps(0.03, METERS_PER_TICK) == (0, 'below minimum')        # 763 cps: never rounded up
    assert safety.speed_cps(0.0394, METERS_PER_TICK) == (1002, '')
    assert safety.speed_cps(0.5, METERS_PER_TICK) == (5000, 'clamped')
    assert safety.speed_cps(-0.5, METERS_PER_TICK) == (-5000, 'clamped')
    assert safety.speed_cps(math.nan, METERS_PER_TICK) == (0, 'invalid')
    assert safety.speed_cps(math.inf, METERS_PER_TICK) == (0, 'invalid')


def test_commands_are_counted():
    core = armed()
    core.on_cmd_vel(0.03, 0.5, 10.1)
    assert core.tick(10.1) == [('DRIVE', 0, 0, False)]                 # coasting: nothing to hold yet
    core.on_cmd_vel(0.9, 0.0, 10.12)
    assert core.tick(10.12) == [('DRIVE', 5000, 0, False)]
    core.on_cmd_vel(math.nan, -0.2, 10.14)
    assert core.tick(10.14) == [('DRIVE', 0, 0, False)]
    counts = core.snapshot(10.14)['counts']
    assert (counts['below minimum'], counts['clamped'], counts['invalid']) == (1, 1, 1)
    assert (counts['steering held'], counts['steering invalid'], counts['steering clamped']) == (1, 1, 0)


# --- steering -------------------------------------------------------------------------------
# delta = atan(0.133 * w / v), percent = delta / 0.204 * 100: (0.3, 0.2) -> 0.0884 rad, 43.35 % -> 43;
# (0.1, 0.1) -> 0.1322 rad, 64.82 % -> 65 (truncating would give 64); (0.1, 0.5) -> 0.587 rad: full lock.

def steer(v, w):
    return safety.steer_percent(v, w, WHEELBASE, FULL_LOCK)


def test_steering_sign_left_right_and_reverse():
    assert steer(0.3, 0.2) == (43, '')                                  # angular.z > 0: left, s > 0
    assert steer(0.3, -0.2) == (-43, '')                                # right
    assert steer(-0.1, 0.1) == (-65, '')                                # CCW while reversing: steer right
    assert steer(-0.1, -0.1) == (65, '')
    assert steer(0.3, 0.0) == (0, '')
    core = armed()
    core.on_cmd_vel(0.1, 0.1, 10.04)
    assert core.tick(10.04) == [('DRIVE', 2542, 65, False)]
    core.on_cmd_vel(-0.1, 0.1, 10.06)
    assert core.tick(10.06) == [('DRIVE', -2542, -65, False)]


def test_steering_rounds_and_clamps_at_full_lock():
    assert steer(0.1, 0.1) == (65, '')                                  # 64.82: rounded, not truncated
    assert steer(0.1, 0.5) == (100, 'clamped')
    assert steer(0.1, -0.5) == (-100, 'clamped')
    assert steer(-0.1, 0.5) == (-100, 'clamped')
    assert steer(0.1, 1e308) == (100, 'clamped')                        # huge but finite
    # Exactly at full lock is not a clamp: w = v * tan(full lock) / wheelbase.
    assert steer(0.1, 0.1 * math.tan(FULL_LOCK) / WHEELBASE * (1 - 1e-12)) == (100, '')
    core = armed()
    core.on_cmd_vel(0.1, 0.5, 10.04)
    assert core.tick(10.04) == [('DRIVE', 2542, 100, False)]
    core.on_cmd_vel(0.1, -2.0, 10.06)
    assert core.tick(10.06) == [('DRIVE', 2542, -100, False)]
    assert core.snapshot(10.06)['counts']['steering clamped'] == 2


def test_steering_is_held_while_the_speed_is_sent_as_zero():
    core = armed()
    core.on_cmd_vel(0.3, 0.2, 10.04)
    assert core.tick(10.04) == [('DRIVE', 5000, 43, False)]
    core.on_cmd_vel(0.0, 0.5, 10.06)                                    # stopped: can't turn in place
    assert core.tick(10.06) == [('DRIVE', 0, 43, False)]
    core.on_cmd_vel(0.03, -0.5, 10.08)                                  # the coast region: also 0 cps
    assert core.tick(10.08) == [('DRIVE', 0, 43, False)]
    core.on_cmd_vel(0.0, 0.0, 10.10)                                    # a plain stop holds too
    assert core.tick(10.10) == [('DRIVE', 0, 43, False)]
    assert core.snapshot(10.10)['counts']['steering held'] == 2          # only those asking for a turn
    core.on_cmd_vel(0.1, -0.1, 10.12)                                   # moving again: a new angle
    assert core.tick(10.12) == [('DRIVE', 2542, -65, False)]


@pytest.mark.parametrize('v, w', [(0.1, math.nan), (0.1, math.inf), (0.1, -math.inf),
                                  (math.nan, 0.2), (math.inf, 0.2), (math.nan, math.nan)])
def test_non_finite_commands_centre_the_steering(v, w):
    core = armed()
    core.on_cmd_vel(0.3, 0.2, 10.04)
    assert core.tick(10.04)[0][2] == 43
    core.on_cmd_vel(v, w, 10.06)
    frames = core.tick(10.06)
    assert frames[0][2] == 0, frames                                   # not held: centred
    assert core.snapshot(10.06)['counts']['steering invalid'] == 1


def test_the_heartbeat_stop_centres_and_the_held_steering_is_reset():
    core = armed()
    core.on_cmd_vel(0.1, 0.1, 10.04)
    assert core.tick(10.04) == [('DRIVE', 2542, 65, False)]
    assert core.tick(10.36) == [('DRIVE', 0, 0, False)]                 # 320 ms: stop, centred
    core.on_cmd_vel(0.0, 0.5, 10.40)                                    # back, but stopped: nothing to hold
    assert core.tick(10.40) == [('DRIVE', 0, 0, False)]


def test_arming_centres_whatever_was_commanded_before_or_during_it():
    core = ready()
    core.on_cmd_vel(0.1, 0.1, 10.0)                                     # steering while DISARMED
    assert core.snapshot(10.0)['command_steer'] == 65
    assert core.arm(10.0)[0]
    assert core.snapshot(10.0)['command_steer'] == 0                    # reset at arm
    assert core.tick(10.02) == [('DRIVE', 0, 0, True)]
    core.on_cmd_vel(0.1, 0.1, 10.03)                                    # a turn while ARMING
    assert core.tick(10.04) == [('DRIVE', 0, 0, False)]                 # ARMING: 0 %, explicitly
    core.on_status(status(motor_state=0, latch_reason=0), 10.05)
    assert core.tick(10.06) == [('DRIVE', 2542, 65, False)]             # ARMED: now it steers


@pytest.mark.parametrize('how', ['estop', 'heartbeat', 'shutdown', 'board latch', 'board reset'])
def test_after_every_stop_no_drive_goes_out_and_steering_is_reset(how):
    """Decision A (2026-10-22): no DRIVE after an ESTOP or a disarm, not even one centring the
    steering; the firmware's 500 ms steering timeout centres it. The held steering is reset, so
    the next arm starts centred."""
    core = armed()
    core.on_cmd_vel(0.1, 0.1, 10.04)
    assert core.tick(10.04) == [('DRIVE', 2542, 65, False)]
    if how == 'estop':
        core.estop(10.05)
    elif how == 'shutdown':
        core.shutdown(10.05)
    elif how == 'board latch':
        core.on_status(status(motor_state=2, latch_reason=2), 10.05)
    elif how == 'board reset':
        core.on_board_reset(10.05)
    start = 10.06 if how != 'heartbeat' else 11.06                       # heartbeat: ESTOP at 1 s
    if how == 'heartbeat':
        for now, frames in ticks(core, 10.06, 11.0):
            assert frames in ([('DRIVE', 0, 0, False)], [('DRIVE', 2542, 65, False)]), (now, frames)
    frames = [frame for _now, out in ticks(core, start, start + 1.0) for frame in out]
    expected = [('ESTOP',)] * 3 if how in ('estop', 'heartbeat', 'shutdown') else []
    assert frames == expected
    assert core.state == DISARMED and core.snapshot(start + 1.0)['command_steer'] == 0
    assert core.snapshot(start + 1.0)['drive_steer'] is None


def test_the_steering_being_sent_is_not_the_command():
    core = armed()
    core.on_cmd_vel(0.1, 0.1, 10.04)
    core.tick(10.04)
    shot = core.snapshot(10.04)
    assert (shot['command_steer'], shot['drive_steer']) == (65, 65)
    core.on_cmd_vel(0.1, 0.1, 10.05)
    core.tick(10.40)                                                     # heartbeat stop: 0 % sent
    shot = core.snapshot(10.40)
    assert shot['drive_steer'] == 0
    core.on_cmd_vel(0.1, 0.1, 10.42)
    core.estop(10.43)
    core.tick(10.44)
    shot = core.snapshot(10.44)
    assert shot['drive_steer'] is None and shot['command_steer'] == 0    # an ESTOP is not a DRIVE


# --- never a DRIVE unless armed -----------------------------------------------------------

def test_nothing_is_sent_until_armed():
    core = ready()
    assert all(frames == [] for _, frames in ticks(core, 10.0, 12.0, cmd_every=0.05))    # commands but no arm
    assert core.state == DISARMED


def test_nothing_before_the_hello_ack_not_even_with_a_command():
    core = new_safety()
    core.on_cmd_vel(0.1, 0.0, 1.0)
    assert core.tick(1.0) == [] and core.arm(1.0)[0] is False


def test_arming_sends_one_arm_drive_then_zero_until_the_board_says_off():
    core = ready()
    accepted, message = core.arm(10.0)
    assert accepted and core.state == ARMING and 'arm bit' in message
    assert core.tick(10.02) == [('DRIVE', 0, 0, True)]
    assert core.tick(10.04) == [('DRIVE', 0, 0, False)]                  # still LATCHED: speed 0
    core.on_status(status(), 10.05)                                       # an older STATUS: still latched
    assert core.state == ARMING and core.tick(10.06) == [('DRIVE', 0, 0, False)]
    core.on_status(status(motor_state=0, latch_reason=0), 10.1)
    assert core.state == ARMED and core.tick(10.1) == [('DRIVE', 2542, 0, False)]


def test_an_arm_the_board_ignores_times_out_with_an_estop():
    core = ready()
    core.arm(10.0)
    core.on_cmd_vel(0.1, 0.0, 10.4)
    out = [(when, frames) for when, frames in ticks(core, 10.02, 10.8) if frames]
    drives = [when for when, frames in out if frames[0][0] == 'DRIVE']
    estops = [when for when, frames in out if frames == [('ESTOP',)]]
    assert drives[-1] - 10.0 <= safety.ARM_TIMEOUT + 0.001 and len(estops) == 3 and estops[0] > drives[-1]
    assert core.state == DISARMED and 'arm timed out' in core.snapshot(10.8)['last_event']
    assert core.tick(10.82) == []


def test_the_board_latching_by_itself_disarms():
    core = armed()
    core.on_status(status(motor_state=2, latch_reason=2), 10.2)           # WATCHDOG, e.g. a USB stall
    assert core.state == DISARMED and 'latched the motor (WATCHDOG)' in core.snapshot(10.2)['last_event']
    assert core.tick(10.22) == []
    core = armed()
    core.on_status(status(motor_state=3, fault_reason=7), 10.2)
    assert core.state == DISARMED and core.tick(10.22) == []


def test_a_board_reset_disarms():
    core = armed()
    core.on_board_reset(10.2)
    assert core.state == DISARMED and core.tick(10.22) == []
    core = armed()
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.2)          # a second HELLO_ACK: reset too
    assert core.state == DISARMED and core.tick(10.22) == []


# --- heartbeat ------------------------------------------------------------------------------

def test_heartbeat_first_stops_then_resumes():
    core = armed()
    core.on_cmd_vel(0.1, 0.0, 10.10)
    assert core.tick(10.38) == [('DRIVE', 2542, 0, False)]                # 280 ms: still driving
    assert core.tick(10.42) == [('DRIVE', 0, 0, False)]                   # 320 ms: DRIVE 0, no latch
    assert core.state == ARMED and core.snapshot(10.42)['counts']['heartbeat stops'] == 1
    core.on_cmd_vel(0.1, 0.0, 10.60)
    assert core.tick(10.62) == [('DRIVE', 2542, 0, False)]                # the operator is back
    assert core.snapshot(10.62)['counts']['estops'] == 0


def test_heartbeat_then_estops_and_disarms_at_one_second():
    core = armed()
    core.on_cmd_vel(0.1, 0.0, 10.10)
    out = ticks(core, 10.12, 11.30)
    frames = [(when, frames) for when, frames in out if frames]
    assert all(f == [('DRIVE', 2542, 0, False)] for when, f in frames if when <= 10.40)
    assert all(f == [('DRIVE', 0, 0, False)] for when, f in frames if 10.42 <= when <= 11.10)
    estops = [when for when, f in frames if f == [('ESTOP',)]]
    assert len(estops) == 3 and 1.0 < estops[0] - 10.10 <= 1.03          # at 1 s, then on the next two ticks
    assert estops == sorted(estops) and estops[2] - estops[0] == pytest.approx(0.04)
    assert core.state == DISARMED and frames[-1][1] == [('ESTOP',)]       # and nothing after them
    core.on_cmd_vel(0.1, 0.0, 11.4)
    assert core.tick(11.42) == []                                          # a command doesn't re-arm
    assert core.snapshot(11.42)['counts']['heartbeat estops'] == 1


STOP_TEXT = 'no /cmd_vel for 300 ms: DRIVE 0 (still armed)'


def test_every_heartbeat_stop_is_logged_not_only_the_first():
    # 2026-10-19 bench: the second stop of a drive had the same text and went unlogged.
    core = armed()
    events = core.take_events(10.03)
    assert events[0] == (False, 'started: DISARMED') and events[-1] == (False, 'armed: motor OFF')
    core.on_cmd_vel(0.1, 0.0, 10.10)
    core.tick(10.42)                                                       # stop 1
    assert core.take_events(10.42) == [(True, STOP_TEXT)]
    ticks(core, 10.60, 12.0, cmd_every=0.05)                               # driving again
    assert core.take_events(12.0) == [(False, '/cmd_vel back after 500 ms: driving again')]
    core.on_cmd_vel(0.1, 0.0, 12.0)
    core.tick(12.32)                                                       # stop 2, 1.9 s later, same text
    assert core.take_events(12.32) == [(True, STOP_TEXT)]
    assert core.snapshot(12.32)['counts']['heartbeat stops'] == 2
    assert core.take_events(12.32) == []                                   # taken once


def test_flapping_stops_are_counted_each_but_logged_once_a_second():
    core = armed()
    core.take_events(10.03)
    logged, now = [], 10.10
    for _ in range(8):                                                     # a stop every 0.4 s for 3.2 s
        core.on_cmd_vel(0.1, 0.0, now)
        core.tick(now + 0.02)                                              # driving
        core.tick(now + 0.32)                                              # stopped
        logged += core.take_events(now + 0.32)
        now += 0.4
    assert core.snapshot(now)['counts']['heartbeat stops'] == 8
    stops = [(warn, text) for warn, text in logged if text.startswith(STOP_TEXT)]
    resumes = [(warn, text) for warn, text in logged if text.startswith('/cmd_vel back after 400 ms')]
    assert len(stops) + len(resumes) == len(logged)
    assert all(warn for warn, _text in stops) and not any(warn for warn, _text in resumes)
    assert len(stops) == 3 and len(resumes) == 3                         # 8 and 7, one a second
    held = [text for _warn, text in stops if '(+' in text]
    assert held[0].endswith('(+2 more in the previous 1.2 s, not logged)'), held
    assert sum(int(text.split('(+')[1].split()[0]) for text in held) + len(stops) <= 8


def gaps(core, now):
    shot = core.snapshot(now)
    return shot['stops_since_arm'], shot['gap_max'], shot['last_stop_gap']


def test_cmd_vel_gaps_since_arm():
    core = armed()                                                         # last /cmd_vel at 10.00
    assert gaps(core, 10.03) == (0, None, None)
    core.on_cmd_vel(0.1, 0.0, 10.05)
    assert gaps(core, 10.05) == (0, pytest.approx(0.05), None)
    core.tick(10.40)                                                       # stop 1 (350 ms)
    core.on_cmd_vel(0.1, 0.0, 10.45)                                       # back after 400 ms
    assert gaps(core, 10.45) == (1, pytest.approx(0.40), pytest.approx(0.40))
    assert core.take_events(10.45)[-1] == (False, '/cmd_vel back after 400 ms: driving again')
    core.tick(11.00)                                                       # stop 2
    core.on_cmd_vel(0.1, 0.0, 11.40)                                       # back after 950 ms
    for at in (11.45, 11.50, 11.55):                                       # 50 ms apart again
        core.on_cmd_vel(0.1, 0.0, at)
        core.tick(at + 0.01)
    assert gaps(core, 11.56) == (2, pytest.approx(0.95), pytest.approx(0.95))   # the 50 ms ones change neither
    assert core.take_events(11.56)[-1] == (False, '/cmd_vel back after 950 ms: driving again')
    ticks(core, 11.58, 12.70)                                              # silence: ESTOP at 1 s
    assert core.state == DISARMED
    stops, longest, last = gaps(core, 12.70)
    assert stops == 3 and 1.0 < longest <= 1.03 and last == pytest.approx(0.95)   # stop 3, then the ESTOP's gap
    core.on_cmd_vel(0.1, 0.0, 13.0)
    core.on_cmd_vel(0.1, 0.0, 15.0)                                        # 2 s, but DISARMED: not counted
    assert gaps(core, 15.0)[1] == longest
    core.on_status(status(motor_state=2, latch_reason=6), 15.0)
    assert core.arm(15.0)[0]                                               # a new arm starts over
    assert gaps(core, 15.0) == (0, None, None)
    assert core.snapshot(15.0)['counts']['heartbeat stops'] == 3           # the total stays


def test_estops_are_never_held_back():
    core = armed()
    core.take_events(10.03)
    core.estop(10.05, 'first')
    core.estop(10.06, 'second')
    assert core.take_events(10.06) == [(True, 'ESTOP x3: first'), (True, 'ESTOP x3: second')]


def test_an_arm_times_out_before_the_heartbeat_could():
    assert safety.ARM_TIMEOUT < safety.HEARTBEAT_ESTOP                    # so ARMING never waits for it


# --- arm refusals ---------------------------------------------------------------------------

@pytest.mark.parametrize('changes, refusal', [
    ({'motor_state': 3, 'fault_reason': 7}, 'motor FAULT'),
    ({'battery_state': 0, 'vbat_mv': 2}, 'battery NOT_PRESENT'),
    ({'battery_state': 2, 'vbat_mv': 6950}, 'battery LOW'),
    ({'battery_state': 3}, 'battery CUTOFF'),
    ({'battery_state': 4}, 'battery ADC_FAULT'),
    ({'battery_state': 5}, 'battery REMOVED_WHILE_DRIVING'),
    ({'latch_reason': 3}, 'latched by STALL LOCKED'),
    ({'latch_reason': 4}, 'latched by STALL SATURATION'),
    ({'latch_reason': 5}, 'latched by ESTIMATE INVALID'),
])
def test_arm_refusals_from_the_status(changes, refusal):
    core = ready(**changes)
    accepted, message = core.arm(10.0)
    assert not accepted and message.startswith(refusal) and core.state == DISARMED
    assert core.tick(10.02) == [] and core.snapshot(10.02)['counts']['arm refusals'] == 1


@pytest.mark.parametrize('reason', (0, 1, 2, 6, 7))       # none, BOOT, WATCHDOG, ESTOP, HELLO
def test_arm_clears_these_latch_reasons(reason):
    core = ready(latch_reason=reason, motor_state=2 if reason else 0)
    assert core.arm(10.0)[0] and core.tick(10.02) == [('DRIVE', 0, 0, True)]


@pytest.mark.parametrize('reason', (3, 4, 5))
def test_force_arms_after_a_stall_or_an_invalid_estimate(reason):
    core = ready(latch_reason=reason)
    assert not core.arm(10.0)[0]
    accepted, _ = core.arm(10.0, force=True)
    assert accepted and core.tick(10.02) == [('DRIVE', 0, 0, True)]
    assert 'forced' in core.snapshot(10.02)['last_event']


def test_force_doesnt_override_the_other_refusals():
    for changes in ({'motor_state': 3}, {'battery_state': 0}):
        assert not ready(**changes).arm(10.0, force=True)[0]


def test_arm_refusals_from_the_link_and_the_operator():
    core = new_safety()
    core.on_status(status(), 10.0)
    core.on_cmd_vel(0.1, 0.0, 10.0)
    assert core.arm(10.0) == (False, 'no HELLO_ACK from the board yet')
    core = new_safety()
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS, build_flags=0x10), 10.0)
    core.on_status(status(), 10.0)
    core.on_cmd_vel(0.1, 0.0, 10.0)
    accepted, message = core.arm(10.0)
    assert not accepted and message.startswith('test build (BENCH_TEXT_COMMANDS)')
    core = ready()
    assert core.arm(10.0 + safety.STATUS_FRESH + 0.01)[1] == 'no fresh STATUS from the board'
    core = new_safety()
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.0)
    assert core.arm(10.0)[1] == 'no fresh STATUS from the board'
    core.on_status(status(), 10.0)
    assert core.arm(10.0)[1].startswith('no fresh /cmd_vel')              # never published
    core.on_cmd_vel(0.0, 0.0, 9.6)
    assert core.arm(10.0)[1].startswith('no fresh /cmd_vel')              # 400 ms old
    core.on_cmd_vel(0.0, 0.0, 9.8)
    assert core.arm(10.0)[0]                                              # 200 ms: fresh


# --- ESTOP ------------------------------------------------------------------------------------

def test_estop_service_sends_three_and_disarms():
    core = armed()
    core.estop(10.2)
    assert [core.tick(10.2 + 0.02 * n) for n in range(5)] == [[('ESTOP',)]] * 3 + [[], []]
    assert core.state == DISARMED
    core.on_cmd_vel(0.1, 0.0, 10.4)
    assert core.tick(10.42) == [] and core.arm(10.42)[0]                  # arm again: allowed


@pytest.mark.parametrize('state', ('disarmed', 'arming', 'armed', 'heartbeat stop'))
def test_shutdown_sends_three_estops_in_every_state_and_then_nothing(state):
    core = ready() if state == 'disarmed' else armed()
    if state == 'arming':
        core = ready()
        core.arm(10.0)
    if state == 'heartbeat stop':
        core.tick(10.5)
    core.shutdown(10.55)
    out = [frames for _, frames in ticks(core, 10.6, 11.6, cmd_every=0.05)]   # commands keep coming
    assert out[:3] == [[('ESTOP',)]] * 3 and all(frames == [] for frames in out[3:])
    assert not core.arm(11.6)[0] and core.arm(11.6)[1] == 'the bridge is shutting down'


def test_an_estop_interrupts_driving_even_mid_command():
    core = armed()
    core.on_cmd_vel(0.1, 0.0, 10.1)
    assert core.tick(10.1) == [('DRIVE', 2542, 0, False)]
    core.estop(10.11)
    assert core.tick(10.12) == [('ESTOP',)]                               # not one more DRIVE first


def test_the_last_event_age_counts_from_that_event():
    core = armed()                                                         # armed at 10.03
    assert core.snapshot(10.03)['last_event_age'] == pytest.approx(0.0)
    assert core.snapshot(22.33)['last_event_age'] == pytest.approx(12.3)
    core.on_cmd_vel(0.1, 0.0, 22.0)
    core.tick(22.31)                                                       # stop: a new event
    assert core.snapshot(22.81)['last_event_age'] == pytest.approx(0.5)
    assert new_safety().snapshot(5.0)['last_event_age'] is None   # 'started' has no time


def test_the_drive_being_sent_is_not_the_command():
    core = ready()
    core.tick(10.0)
    assert core.snapshot(10.0)['drive_cps'] is None                        # DISARMED: no DRIVE
    core = armed()
    core.tick(10.04)
    assert core.snapshot(10.04)['drive_cps'] == 2542
    core.tick(10.32)                                                       # heartbeat stop
    shot = core.snapshot(10.32)
    assert shot['drive_cps'] == 0 and shot['command_cps'] == 2542
    core.estop(10.33)
    core.tick(10.34)
    assert core.snapshot(10.34)['drive_cps'] is None                       # an ESTOP is not a DRIVE


def test_the_resume_line_has_this_stops_gap_not_the_longest():
    core = armed()
    core.on_cmd_vel(0.1, 0.0, 10.10)
    core.tick(10.50)                                                       # stop 1
    core.on_cmd_vel(0.1, 0.0, 11.00)                                       # back after 900 ms
    assert core.take_events(11.00)[-1] == (False, '/cmd_vel back after 900 ms: driving again')
    core.on_cmd_vel(0.1, 0.0, 11.25)
    core.on_cmd_vel(0.1, 0.0, 11.50)
    core.tick(11.85)                                                       # stop 2, shorter
    core.on_cmd_vel(0.1, 0.0, 12.05)                                       # back after 550 ms
    assert core.take_events(12.05)[-1] == (False, '/cmd_vel back after 550 ms: driving again')
    assert core.snapshot(12.05)['gap_max'] == pytest.approx(0.90)


def test_slow_gaps_are_counted_and_logged_before_they_cost_a_stop():
    # 2026-10-19 Control B (Wi-Fi): a 251 ms gap, then 433 and 341 ms stops; the 251 had no Pi time.
    core = armed()                                                         # last /cmd_vel at 10.00
    core.take_events(10.03)
    core.on_cmd_vel(0.1, 0.0, 10.14)                                       # 140 ms: not slow
    core.tick(10.20)
    core.tick(10.38)                                                       # 240 ms: no stop yet
    core.on_cmd_vel(0.1, 0.0, 10.39)                                       # 250 ms: slow, no stop
    assert core.take_events(10.39) == [(False, '/cmd_vel gap 250 ms (no stop)')]
    core.tick(10.80)                                                       # stop
    core.on_cmd_vel(0.1, 0.0, 10.85)                                       # back after 460 ms
    assert core.take_events(10.85) == [(True, STOP_TEXT), (False, '/cmd_vel back after 460 ms: driving again')]
    assert core.snapshot(10.85)['slow_gaps_since_arm'] == 2               # the stop's gap counts too
    core.estop(10.9)
    core.on_cmd_vel(0.1, 0.0, 11.5)                                        # 650 ms, but DISARMED
    assert core.snapshot(11.5)['slow_gaps_since_arm'] == 2
    core.on_status(status(motor_state=2, latch_reason=6), 11.5)
    assert core.arm(11.5)[0] and core.snapshot(11.5)['slow_gaps_since_arm'] == 0


def test_held_back_lines_are_said_within_a_second_not_at_the_next_event():
    # 2026-10-19: a stop held back at 848.4 was only mentioned 220 s later, at the next stop.
    core = armed()
    core.take_events(10.03)
    core.on_cmd_vel(0.1, 0.0, 10.10)
    core.tick(10.42)                                                       # stop 1: logged
    assert core.take_events(10.42) == [(True, STOP_TEXT)]
    core.on_cmd_vel(0.1, 0.0, 10.50)                                       # resume 1: logged
    core.take_events(10.50)
    core.tick(10.82)                                                       # stop 2: held back
    assert core.take_events(10.82) == []
    for at in (10.90, 10.95, 11.00, 11.05, 11.10, 11.15, 11.20, 11.25, 11.30, 11.35, 11.40, 11.45, 11.50, 11.55):
        core.on_cmd_vel(0.1, 0.0, at)                                      # resume 2 at 10.90: held back
        core.tick(at + 0.01)
        lines = core.take_events(at + 0.01)
        if at + 0.01 < 11.42:
            assert lines == [], (at, lines)
        elif at == 11.45:
            assert lines == [(True, '+1 more "heartbeat stop" in the 1.0 s after the last such line, '
                                    'not logged (counted)')], lines
        elif at == 11.50:
            assert lines == [(False, '+1 more "heartbeat resume" in the 1.0 s after the last such line, '
                                     'not logged (counted)')], lines
    assert core.take_events(11.6) == []                                    # said once
    assert core.snapshot(11.6)['counts']['heartbeat stops'] == 2


def test_slow_gap_lines_are_rate_limited_but_all_counted():
    core = armed()                                                         # last /cmd_vel at 10.00
    core.take_events(10.03)
    lines = []
    for at in (10.20, 10.40, 10.60):                                       # three 200 ms gaps, no stop
        core.on_cmd_vel(0.1, 0.0, at)
        lines += core.take_events(at)
    assert lines == [(False, '/cmd_vel gap 200 ms (no stop)')]
    assert core.take_events(11.25) == [(False, '+2 more "slow gap" in the 1.1 s after the last such line, '
                                               'not logged (counted)')]
    assert core.snapshot(11.25)['slow_gaps_since_arm'] == 3
