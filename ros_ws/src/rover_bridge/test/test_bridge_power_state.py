"""rover_bridge.power_state and the battery status in /diagnostics: what the bridge writes for the
host's shutdown watcher (tools/battery_watch.py), and the watcher's state read back. ROS-free (CI)."""
import json

import bridge_fakes
from rover_bridge import power_state
from rover_bridge.diagnostics import ERROR, OK, WARN, Diagnostics

import battery_watch                          # tools/ (rover_bridge._tools put it on the path)

BOOT = 'b00t'


def status(**changes):
    return bridge_fakes.status_fields(**dict({'motor_state': 3, 'latch_reason': 0, 'battery_state': 3,
                                              'vbat_mv': 6700}, **changes))


def test_the_battery_state_is_what_the_watcher_reads(tmp_path):
    path = tmp_path / 'battery.json'
    power_state.write_atomic(str(path), power_state.battery_state(status(), 123.4, BOOT, 1792700000.5))
    data = json.loads(path.read_text())
    assert data == {'boot_id': BOOT, 'status_monotonic': 123.4, 'battery_state': 3, 'vbat_mv': 6700,
                    'motor_state': 3, 'wall': 1792700000.5}
    assert not (tmp_path / 'battery.json.tmp').exists()        # renamed into place
    rule = battery_watch.Rule(BOOT)
    assert rule.update(battery_watch.read_json(str(path)), 123.6)['state'] == 'low'


def watch(state='low', at=50.0, boot=BOOT, **changes):
    data = {'state': state, 'text': 'x', 'held_s': 12.0, 'shutdown_in_s': 18.0 if state == 'low' else None,
            'threshold_mv': 6800, 'boot_id': boot, 'monotonic': at, 'action': 'shutdown'}
    data.update(changes)
    return data


def test_the_watch_file_is_read_only_when_fresh_and_from_this_boot(tmp_path):
    path = tmp_path / 'watch.json'
    assert power_state.read_watch(str(path), 50.0, BOOT) is None                 # no watcher
    path.write_text(json.dumps(watch()))
    assert power_state.read_watch(str(path), 52.0, BOOT)['state'] == 'low'
    assert power_state.read_watch(str(path), 55.01, BOOT) is None                # stopped
    assert power_state.read_watch(str(path), 52.0, 'other boot') is None
    path.write_text('{"state": ')
    assert power_state.read_watch(str(path), 52.0, BOOT) is None                 # half written


def test_the_watch_text():
    assert power_state.watch_text(None) == 'not running'
    assert power_state.watch_text(watch()) == 'Pi SHUTDOWN in 18 s (shutdown)'
    assert power_state.watch_text(watch(action='log')) == 'Pi SHUTDOWN in 18 s (log)'
    assert power_state.watch_text(watch('shutdown')) == 'Pi shutting down'
    assert power_state.watch_text(watch('shutdown', action='log')) == 'Pi would shut down (ACTION=log)'
    assert power_state.watch_text(watch('ok')) == 'ok'


def battery_report(fields, **snapshot_changes):
    snapshot = {'status': fields, 'hello_ack': dict(bridge_fakes.HELLO_ACK_FIELDS), 'state_age': 0.01,
                'status_age': 0.05, 'counts': {'frames': 1, 'bad': 0, 'seq_gaps': 0, 't_ms_gaps': 0, 'dropped': 0,
                                               'hello_sent': 1},
                'stale_text': 'none', 'read_gap_period': 0.01, 'read_gap_session': 0.01, 'clock': None}
    snapshot.update(snapshot_changes)
    report = Diagnostics()._battery(snapshot)
    return report, dict(report['values'])


def test_the_countdown_is_in_the_battery_status_and_raises_it_to_warn():
    report, values = battery_report(status(battery_state=1, vbat_mv=7850, motor_state=0),
                                    power_watch=watch(action='log'))
    assert report['level'] == WARN
    assert report['message'] == '7.85 V; Pi SHUTDOWN in 18 s (log)'
    assert values['Pi shutdown watch'] == 'Pi SHUTDOWN in 18 s (log)'
    report, _values = battery_report(status(), power_watch=watch())                    # CUTOFF stays ERROR
    assert report['level'] == ERROR and report['message'].endswith('; Pi SHUTDOWN in 18 s (shutdown)')
    report, _values = battery_report(status(), power_watch=watch('shutdown'))
    assert report['message'].endswith('; Pi shutting down')


def test_no_watcher_or_a_quiet_one_changes_nothing_but_the_value():
    report, values = battery_report(status(battery_state=1, vbat_mv=7850, motor_state=0), power_watch=None)
    assert report['level'] == OK and report['message'] == '7.85 V' and values['Pi shutdown watch'] == 'not running'
    report, values = battery_report(status(battery_state=1, vbat_mv=7850, motor_state=0), power_watch=watch('ok'))
    assert report['level'] == OK and values['Pi shutdown watch'] == 'ok'
    report, values = battery_report(status(battery_state=1, vbat_mv=7850, motor_state=0))   # key absent
    assert 'Pi shutdown watch' not in values
