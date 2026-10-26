"""rover_bridge.battery and rover_bridge.diagnostics. Pure, runs anywhere (CI included)."""
import math

import bridge_fakes
from rover_bridge import battery
from rover_bridge.clock_fit import ClockFit
from rover_bridge.diagnostics import ERROR, OK, STALE, WARN, Diagnostics
from rover_bridge.safety import Safety


def test_wall_power_is_no_pack_not_a_dead_one():
    fields = battery.battery_fields(bridge_fakes.status_fields(battery_state=battery.NOT_PRESENT, vbat_mv=2))
    assert not fields['present'] and math.isnan(fields['voltage'])
    assert fields['power_supply_status'] == battery.STATUS_UNKNOWN
    assert fields['power_supply_health'] == battery.HEALTH_UNKNOWN          # not DEAD
    assert fields['power_supply_technology'] == battery.TECHNOLOGY_LIPO and fields['design_capacity'] == 1.2
    assert all(math.isnan(fields[key]) for key in ('current', 'charge', 'capacity', 'percentage', 'temperature'))


def test_battery_states():
    def fields(state, vbat_mv=7400):
        return battery.battery_fields(bridge_fakes.status_fields(battery_state=state, vbat_mv=vbat_mv))

    ok = fields(battery.OK, 7713)
    assert ok['present'] and ok['voltage'] == 7.713 and ok['power_supply_health'] == battery.HEALTH_GOOD
    assert ok['power_supply_status'] == battery.STATUS_DISCHARGING
    low = fields(battery.LOW, 6950)
    assert low['present'] and low['voltage'] == 6.95 and low['power_supply_health'] == battery.HEALTH_GOOD
    cutoff = fields(battery.CUTOFF, 6550)
    assert cutoff['present'] and cutoff['voltage'] == 6.55 and cutoff['power_supply_health'] == battery.HEALTH_DEAD
    adc = fields(battery.ADC_FAULT, 0)
    assert not adc['present'] and math.isnan(adc['voltage'])
    assert adc['power_supply_health'] == battery.HEALTH_UNSPEC_FAILURE
    removed = fields(battery.REMOVED_WHILE_DRIVING, 3)
    assert not removed['present'] and math.isnan(removed['voltage'])
    assert removed['power_supply_health'] == battery.HEALTH_UNKNOWN
    assert battery.state_name(5) == 'REMOVED_WHILE_DRIVING' and battery.state_name(9) == 'STATE_9'


def fitted_clock():
    fit = ClockFit()
    for n in range(500):                                   # 5 s, 3000 ppm fast, 2 ms latency
        fit.add(n * 10, 1000.0 + n * 0.010 / 1.003 + 0.002)
    return fit


def snapshot(**changes):
    values = {'status': bridge_fakes.status_fields(latch_reason=1, battery_state=0, vbat_mv=2, vdda_mv=3312,
                                                   reset_flags=12, rx_ok=3, rx_uart_errors=6),
              'hello_ack': dict(bridge_fakes.HELLO_ACK_FIELDS), 'state_age': 0.01, 'status_age': 0.05,
              'counts': {'frames': 1000, 'bad': 0, 'seq_gaps': 0, 't_ms_gaps': 0, 'dropped': 0, 'hello_sent': 1},
              'stale_text': 'none', 'read_gap_period': 0.012, 'read_gap_session': 0.056, 'clock': fitted_clock()}
    values.update(changes)
    return values


def by_name(reports):
    return {report['name'].split(': ')[1]: report for report in reports}


def values(report):
    return dict(report['values'])


def test_wall_power_on_the_bench_is_all_ok():
    reports = by_name(Diagnostics().build(snapshot()))
    assert set(reports) == {'motor', 'battery', 'link', 'clock', 'operator'}
    assert all(report['level'] == OK for report in reports.values()), reports
    motor = reports['motor']
    assert motor['message'] == 'LATCHED (BOOT)'
    assert values(motor)['build flags'] == 'none (normal build)' and values(motor)['reset flags'] == '12 (BOR+PIN)'
    assert values(motor)['fault reason'] == 'none' and values(motor)['imu ok'] == 'True'
    assert values(motor)['setpoint cps (board echo of last DRIVE)'] == '0' and values(motor)['speed cps (estimate)'] == 'n/a'
    assert values(by_name(Diagnostics().build(snapshot(speed_cps=2547)))['motor'])['speed cps (estimate)'] == '2547'
    assert reports['battery']['message'] == 'no pack connected (e.g. on wall power)'
    assert values(reports['battery'])['state'] == 'NOT_PRESENT' and values(reports['battery'])['vbat mV'] == '2'
    link = values(reports['link'])
    assert link['firmware rx_ok'] == '3' and link['firmware rx_uart_errors'] == '6'
    assert link['read gap max ms (period)'] == '12.00' and link['read gap max ms (session)'] == '56.00'
    assert link['stale before HELLO_ACK'] == 'none' and link['queue drops'] == '0'
    clock = values(reports['clock'])
    assert abs(float(clock['drift ppm (board fast +)']) - 3000) < 20 and float(clock['residual ms']) < 0.1
    assert values(reports['operator'])['heartbeat age'] == 'n/a'


def test_what_makes_driving_wrong_is_an_error():
    def motor(**status):
        report = by_name(Diagnostics().build(snapshot(status=bridge_fakes.status_fields(**status))))['motor']
        return report['level'], report['message']

    assert motor(motor_state=3, fault_reason=7) == (ERROR, 'FAULT (REPEATED STALL)')
    level, message = motor(imu_ok=0)
    assert level == ERROR and 'IMU failed at boot' in message
    test_build = by_name(Diagnostics().build(snapshot(hello_ack=dict(bridge_fakes.HELLO_ACK_FIELDS, build_flags=0x10))))
    assert test_build['motor']['level'] == ERROR and 'TEST BUILD' in test_build['motor']['message']
    assert values(test_build['motor'])['build flags'] == 'BENCH_TEXT_COMMANDS'
    for state, level in ((battery.LOW, WARN), (battery.CUTOFF, ERROR), (battery.ADC_FAULT, ERROR),
                         (battery.REMOVED_WHILE_DRIVING, ERROR), (battery.OK, OK)):
        report = by_name(Diagnostics().build(snapshot(status=bridge_fakes.status_fields(battery_state=state))))
        assert report['battery']['level'] == level, state


def test_new_losses_warn_once_then_ok():
    diagnostics = Diagnostics()
    assert by_name(diagnostics.build(snapshot()))['link']['level'] == OK
    worse = snapshot(counts={'frames': 1100, 'bad': 1, 'seq_gaps': 2, 't_ms_gaps': 1, 'dropped': 0, 'hello_sent': 1})
    link = by_name(diagnostics.build(worse))['link']
    assert link['level'] == WARN and link['message'] == 'new bad: +1; new seq gaps: +2; new t ms gaps: +1'
    assert by_name(diagnostics.build(worse))['link']['level'] == OK          # nothing new since
    slow = by_name(diagnostics.build(dict(worse, read_gap_period=0.6)))['link']
    assert slow['level'] == WARN and slow['message'] == 'read gap 600 ms'
    dropped = dict(worse, counts=dict(worse['counts'], dropped=5))
    assert by_name(diagnostics.build(dropped))['link']['message'] == 'new dropped: +5'


def test_silence_is_stale():
    reports = by_name(Diagnostics().build(snapshot(status=None, status_age=None, state_age=None, hello_ack=None)))
    assert reports['motor']['level'] == STALE and reports['battery']['level'] == STALE
    assert reports['link']['level'] == STALE and reports['link']['message'] == 'no HELLO_ACK yet'
    old = by_name(Diagnostics().build(snapshot(state_age=1.5, status_age=1.5)))
    assert old['link']['message'] == 'no STATE' and old['motor']['level'] == STALE


def test_clock_not_fitted_or_off_warns():
    assert by_name(Diagnostics().build(snapshot(clock=ClockFit())))['clock']['level'] == WARN
    bad = ClockFit()
    for n in range(500):                                   # a board 2 % slow: not a clock this one has
        bad.add(n * 10, 1000.0 + n * 0.010 * 1.02)
    report = by_name(Diagnostics().build(snapshot(clock=bad)))['clock']
    assert report['level'] == WARN and report['message'].startswith('fit looks wrong: drift -1')


def operator_values(core, now):
    report = by_name(Diagnostics().build(snapshot(safety=core.snapshot(now))))['operator']
    return report, values(report)


def test_operator_heartbeat_stops_and_gaps_since_arm():
    core = Safety(3.9336e-5, 0.133, 0.204)
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.0)
    core.on_status(bridge_fakes.status_fields(motor_state=2, latch_reason=1, battery_state=1, vbat_mv=7700), 10.0)
    core.on_cmd_vel(0.1, 0.0, 10.0)
    _report, before = operator_values(core, 10.0)
    assert before['heartbeat stops since arm'] == '0' and before['cmd_vel gap max ms (since arm)'] == 'n/a'
    assert before['cmd_vel last stop gap ms (since arm)'] == 'n/a'
    assert core.arm(10.0)[0]
    core.tick(10.02)
    core.on_status(bridge_fakes.status_fields(motor_state=0, latch_reason=0, battery_state=1, vbat_mv=7700), 10.03)
    core.tick(10.40)                                                       # stop
    core.on_cmd_vel(0.1, 0.0, 10.712)                                      # back after 712 ms
    _report, after = operator_values(core, 10.72)
    assert after['heartbeat stops'] == '1' and after['heartbeat stops since arm'] == '1'
    assert after['cmd_vel gap max ms (since arm)'] == '712.00'
    assert after['cmd_vel last stop gap ms (since arm)'] == '712.00'
    core.tick(11.05)                                                       # stop 2
    core.on_cmd_vel(0.1, 0.0, 11.112)                                      # back after 400 ms
    _report, second = operator_values(core, 11.12)
    assert second['heartbeat stops'] == '2' and second['heartbeat stops since arm'] == '2'
    assert second['cmd_vel gap max ms (since arm)'] == '712.00'           # the longest
    assert second['cmd_vel last stop gap ms (since arm)'] == '400.00'     # the last
    assert second['cmd_vel gaps >= 150 ms (since arm)'] == '2'


def test_the_last_event_has_an_age():
    # 2026-10-19: the 10 s line's "(last event)" read as current minutes after it happened.
    core = Safety(3.9336e-5, 0.133, 0.204)
    report, at_start = operator_values(core, 5.0)
    assert at_start['last event'] == 'started: DISARMED' and at_start['last event age s'] == 'n/a'
    assert report['message'] == 'DISARMED: started: DISARMED (at start)'
    core.estop(10.0, 'requested (/rover/estop)')
    report, later = operator_values(core, 22.34)
    assert later['last event'] == 'ESTOP x3: requested (/rover/estop)' and later['last event age s'] == '12.3'
    assert report['message'] == 'DISARMED: ESTOP x3: requested (/rover/estop) (12.3 s ago)'


def test_setpoint_echo_and_the_units_the_board_runs_on():
    # STATUS setpoint_cps is the DRIVE as received (link_rx.c); the controller quantizes it to
    # 50-cps units, nearest, ties toward zero, below 1000 -> 0 (LinkProtocol_CpsToUnits).
    def setpoints(cps):
        report = by_name(Diagnostics().build(snapshot(status=bridge_fakes.status_fields(setpoint_cps=cps))))['motor']
        return values(report)['setpoint cps (board echo of last DRIVE)'], values(report)['setpoint cps (board, 50-cps units)']

    assert setpoints(2542) == ('2542', '2550')
    assert setpoints(-2542) == ('-2542', '-2550')
    assert setpoints(1025) == ('1025', '1000')
    assert setpoints(1026) == ('1026', '1050')
    assert setpoints(999) == ('999', '0')
    assert setpoints(0) == ('0', '0')


def test_operator_command_asked_for_and_drive_being_sent():
    core = Safety(3.9336e-5, 0.133, 0.204)
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.0)
    core.on_status(bridge_fakes.status_fields(motor_state=2, latch_reason=1, battery_state=1, vbat_mv=7700), 10.0)
    core.on_cmd_vel(0.1, 0.0, 10.0)
    _report, disarmed = operator_values(core, 10.0)
    assert disarmed['command cps (from /cmd_vel)'] == '2542' and disarmed['drive cps (sent now)'] == 'n/a'
    assert core.arm(10.0)[0]
    core.tick(10.02)
    core.on_status(bridge_fakes.status_fields(motor_state=0, latch_reason=0, battery_state=1, vbat_mv=7700), 10.03)
    core.tick(10.04)
    assert operator_values(core, 10.04)[1]['drive cps (sent now)'] == '2542'
    core.tick(10.40)                                                       # heartbeat stop: DRIVE 0
    _report, stopped = operator_values(core, 10.40)
    assert stopped['command cps (from /cmd_vel)'] == '2542' and stopped['drive cps (sent now)'] == '0'


def test_a_seq_gap_the_board_dropped_says_so():
    # 2026-10-19: one STATE never sent (the board's TX queue was full at the text-to-binary switch).
    counts = {'frames': 1000, 'bad': 0, 'seq_gaps': 0, 't_ms_gaps': 0, 'dropped': 0, 'hello_sent': 1}
    diagnostics = Diagnostics()
    diagnostics.build(snapshot(counts=counts, status=bridge_fakes.status_fields(tx_dropped=0)))
    gap = dict(counts, frames=1100, seq_gaps=1)
    board = by_name(diagnostics.build(snapshot(counts=gap, status=bridge_fakes.status_fields(tx_dropped=1))))['link']
    assert board['level'] == WARN and board['message'] == 'new seq gaps: +1 (dropped by the board: tx_dropped +1)'
    wire = dict(gap, frames=1200, seq_gaps=2)
    lost = by_name(diagnostics.build(snapshot(counts=wire, status=bridge_fakes.status_fields(tx_dropped=1))))['link']
    assert lost['message'] == 'new seq gaps: +1'                           # the board sent it: lost on the way


def test_operator_slow_gaps_are_not_the_stops():
    core = Safety(3.9336e-5, 0.133, 0.204)
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.0)
    core.on_status(bridge_fakes.status_fields(motor_state=2, latch_reason=1, battery_state=1, vbat_mv=7700), 10.0)
    core.on_cmd_vel(0.1, 0.0, 10.0)
    assert core.arm(10.0)[0]
    core.tick(10.02)
    core.on_status(bridge_fakes.status_fields(motor_state=0, latch_reason=0, battery_state=1, vbat_mv=7700), 10.03)
    core.on_cmd_vel(0.1, 0.0, 10.25)                                       # 250 ms, no stop
    _report, slow = operator_values(core, 10.25)
    assert slow['cmd_vel gaps >= 150 ms (since arm)'] == '1' and slow['heartbeat stops since arm'] == '0'


def test_operator_shows_the_steering_and_warns_on_a_clamp_or_a_non_finite_command():
    core = Safety(3.9336e-5, 0.133, 0.204)
    core.on_hello_ack(dict(bridge_fakes.HELLO_ACK_FIELDS), 10.0)
    core.on_status(bridge_fakes.status_fields(motor_state=2, latch_reason=1, battery_state=1, vbat_mv=7700), 10.0)
    core.on_cmd_vel(0.1, 0.1, 10.0)
    diagnostics = Diagnostics()
    report = by_name(diagnostics.build(snapshot(safety=core.snapshot(10.0))))['operator']
    shown = values(report)
    assert shown['command steer % (from /cmd_vel)'] == '65' and shown['drive steer % (sent now)'] == 'n/a'
    assert report['level'] == OK
    core.on_cmd_vel(0.1, 0.5, 10.02)
    report = by_name(diagnostics.build(snapshot(safety=core.snapshot(10.02))))['operator']
    assert report['level'] == WARN and report['message'] == 'steering clamped at full lock'
    core.on_cmd_vel(0.1, math.nan, 10.04)
    report = by_name(diagnostics.build(snapshot(safety=core.snapshot(10.04))))['operator']
    assert report['level'] == WARN and report['message'] == 'non-finite commands steered to centre'
    assert values(report)['command steer % (from /cmd_vel)'] == '0'
    core.on_cmd_vel(0.0, 0.5, 10.06)                                      # held: normal, no WARN
    report = by_name(diagnostics.build(snapshot(safety=core.snapshot(10.06))))['operator']
    assert report['level'] == OK, report
