"""Tests for battery_watch.py, the Pi's low-battery shutdown: the rule on an injected clock
(threshold, the 30 s hold at rest, resets, pauses, stale data, wall power), the settings, and the
watcher's log line before the shutdown command. No systemd, no files outside tmp_path."""
import json

import pytest

import battery_watch as bw
from battery_watch import ADC_FAULT, CUTOFF, LOW, NOT_PRESENT, OK, REMOVED_WHILE_DRIVING, Rule

BOOT = 'b00t'


def sample(at, vbat=6700, state=CUTOFF, motor=3, boot=BOOT):
    """A battery.json written at `at` for a STATUS received at `at` (motor 3: FAULT after a cutoff)."""
    return {'boot_id': boot, 'status_monotonic': at, 'battery_state': state, 'vbat_mv': vbat, 'motor_state': motor,
            'wall': 1792700000.0 + at}


def feed(rule, start, seconds, step=1.0, **fields):
    """One fresh sample per step from start (inclusive) for `seconds`: the last watch state."""
    watch = None
    count = int(round(seconds / step))
    for i in range(count + 1):
        at = start + i * step
        watch = rule.update(sample(at, **fields), at + 0.2)
    return watch


def test_30_s_at_rest_below_6800_shuts_down_and_not_29():
    rule = Rule(BOOT)
    watch = feed(rule, 100.0, 29.0)
    assert watch['state'] == 'low' and not rule.shutdown
    assert watch['shutdown_in_s'] == 1.0 and 'Pi shutdown in 1 s' in watch['text']
    watch = rule.update(sample(130.0), 130.2)
    assert rule.shutdown and watch['state'] == 'shutdown'
    assert watch['text'] == '6.70 V at rest for 30 s: shutting down'


def test_the_threshold_is_6800_and_at_it_is_not_low():
    rule = Rule(BOOT)
    assert feed(rule, 100.0, 60.0, vbat=6800)['state'] == 'ok' and not rule.shutdown
    assert feed(Rule(BOOT), 100.0, 30.0, vbat=6799)['state'] == 'shutdown'
    assert bw.THRESHOLD_MV == 6800 and bw.HOLD_SECONDS == 30.0


def test_ok_and_low_states_count_too_only_the_voltage_matters():
    for state in (OK, LOW):
        assert feed(Rule(BOOT), 100.0, 30.0, state=state, motor=0)['state'] == 'shutdown'


def test_a_dip_of_any_length_under_30_s_resets_when_the_pack_recovers_at_rest():
    rule = Rule(BOOT)
    feed(rule, 100.0, 25.0)                                    # 25 s low
    assert rule.update(sample(126.0, vbat=6850), 126.2)['state'] == 'ok'
    assert rule.held == 0.0
    watch = feed(rule, 127.0, 29.0)                            # a new hold from 0
    assert watch['state'] == 'low' and not rule.shutdown


def test_samples_while_the_motor_runs_dont_count_and_dont_reset():
    rule = Rule(BOOT)
    feed(rule, 100.0, 20.0)                                    # 20 s at rest
    watch = feed(rule, 121.0, 60.0, vbat=6000, motor=1)        # a minute driving, deep sag
    assert watch['state'] == 'running' and rule.held == pytest.approx(20.0) and not rule.shutdown
    feed(rule, 182.0, 9.0)                                     # at rest again: 9 more
    assert not rule.shutdown
    rule.update(sample(192.0), 192.2)
    assert rule.shutdown                                       # 20 + 10 at rest


@pytest.mark.parametrize('state', [NOT_PRESENT, REMOVED_WHILE_DRIVING])
def test_no_pack_never_counts_and_resets(state):
    rule = Rule(BOOT)
    assert feed(rule, 100.0, 120.0, vbat=3, state=state, motor=0)['state'] == 'no pack'
    assert not rule.shutdown
    feed(rule, 300.0, 20.0)
    rule.update(sample(321.0, vbat=4, state=state), 321.2)     # the pack unplugged mid-hold
    assert rule.held == 0.0


def test_an_adc_fault_pauses_and_never_shuts_down():
    rule = Rule(BOOT)
    feed(rule, 100.0, 20.0)
    watch = feed(rule, 121.0, 120.0, vbat=1000, state=ADC_FAULT)
    assert watch['state'] == 'adc fault' and not rule.shutdown and rule.held == pytest.approx(20.0)


def test_stale_data_pauses_the_hold_it_doesnt_reset_or_count():
    rule = Rule(BOOT)
    feed(rule, 100.0, 20.0)                                    # 20 s
    for now in range(121, 200):                                # the bridge stopped: the file stays at 120
        assert rule.update(sample(120.0), float(now))['state'] in ('low', 'stale')
    assert rule.update(sample(120.0), 199.0)['state'] == 'stale'
    assert rule.held == pytest.approx(20.0) and not rule.shutdown
    feed(rule, 200.0, 9.0)                                     # back: the gap adds nothing, 9 s more
    assert not rule.shutdown and rule.held == pytest.approx(29.0)
    rule.update(sample(210.0), 210.2)
    assert rule.shutdown


def test_the_same_sample_read_twice_adds_nothing():
    rule = Rule(BOOT)
    feed(rule, 100.0, 10.0)
    for now in (110.5, 111.0, 112.0, 113.0):                   # fresh (<= 5 s) but not new
        rule.update(sample(110.0), now)
    assert rule.held == pytest.approx(10.0)


def test_one_sample_adds_at_most_2_s():
    rule = Rule(BOOT)
    rule.update(sample(100.0), 100.1)
    rule.update(sample(104.0), 104.1)                          # 4 s apart, both fresh
    assert rule.held == pytest.approx(2.0)


@pytest.mark.parametrize('bad', [None, {}, {'boot_id': BOOT}, sample(100.0, boot='other boot'),
                                 dict(sample(100.0), vbat_mv='6700'), dict(sample(100.0), battery_state=None),
                                 sample(106.0)])
def test_missing_garbled_foreign_or_future_data_is_stale(bad):
    rule = Rule(BOOT)
    assert rule.update(bad, 100.0)['state'] == 'stale'         # sample(106.0) at 100.0: from the future


def test_data_older_than_5_s_is_stale():
    rule = Rule(BOOT)
    assert rule.update(sample(100.0), 105.0)['state'] == 'low'
    assert rule.update(sample(100.0), 105.01)['state'] == 'stale'


def test_the_shutdown_is_latched():
    rule = Rule(BOOT)
    feed(rule, 100.0, 30.0)
    assert rule.shutdown
    assert rule.update(sample(131.0, vbat=8000), 131.2)['state'] == 'shutdown'
    assert rule.update(None, 132.0)['state'] == 'shutdown'


def test_overrides_for_the_bench():
    rule = Rule(BOOT, threshold_mv=7900, hold_seconds=10.0)
    assert feed(rule, 100.0, 10.0, vbat=7850, state=OK, motor=0)['state'] == 'shutdown'


def test_settings_from_the_environment():
    assert bw.settings({}) == (6800, 30.0, 'shutdown', [])
    assert bw.settings({'THRESHOLD_MV': '7900', 'HOLD_S': '10', 'ACTION': 'log'}) == (7900, 10.0, 'log', [])
    threshold, hold, action, problems = bw.settings({'THRESHOLD_MV': '79OO', 'HOLD_S': '0', 'ACTION': 'reboot'})
    assert (threshold, hold, action) == (6800, 30.0, 'shutdown') and len(problems) == 3
    assert bw.settings({'THRESHOLD_MV': '12000'})[0] == 6800    # out of range: the default


# --- the watcher: files, log, the command ----------------------------------------------------

class Lines:
    def __init__(self):
        self.lines = []

    def __call__(self, text, sync=False):
        self.lines.append((text, sync))


def make_watcher(tmp_path, action='shutdown', **rule):
    runs = []
    log = Lines()
    watcher = bw.Watcher(Rule(BOOT, **rule), action, log, run=lambda command, check: runs.append(command),
                         state_file=str(tmp_path / 'battery.json'), watch_file=str(tmp_path / 'watch.json'))
    return watcher, log, runs


def write(tmp_path, data):
    (tmp_path / 'battery.json').write_text(json.dumps(data))


def test_the_watcher_logs_the_shutdown_line_synced_before_the_command(tmp_path):
    watcher, log, runs = make_watcher(tmp_path)
    for i in range(31):
        write(tmp_path, sample(100.0 + i))
        watcher.step(100.2 + i)
    assert runs == [['/usr/sbin/shutdown', '-h', 'now']]
    shutdown = [entry for entry in log.lines if entry[0].startswith('SHUTDOWN:')]
    assert shutdown == [('SHUTDOWN: 6.70 V at rest for 30 s: shutting down (threshold 6800 mV, hold 30 s, '
                         'action shutdown)', True)]
    assert log.lines.index(shutdown[0]) == len(log.lines) - 1
    write(tmp_path, sample(131.0))
    watcher.step(131.2)
    assert len(runs) == 1                                      # once
    watch = json.loads((tmp_path / 'watch.json').read_text())
    assert watch['state'] == 'shutdown' and watch['action'] == 'shutdown' and watch['boot_id'] == BOOT


def test_action_log_never_runs_the_command(tmp_path):
    watcher, log, runs = make_watcher(tmp_path, action='log', threshold_mv=7900, hold_seconds=10.0)
    for i in range(11):
        write(tmp_path, sample(100.0 + i, vbat=7850, state=OK, motor=0))
        watcher.step(100.2 + i)
    assert runs == []
    assert ('would shut down now (ACTION=log): nothing done', True) in log.lines


def test_the_watcher_logs_changes_and_the_countdown_every_10_s(tmp_path):
    watcher, log, _runs = make_watcher(tmp_path)
    watcher.step(99.0)                                         # no file yet
    for i in range(25):
        write(tmp_path, sample(100.0 + i))
        watcher.step(100.2 + i)
    texts = [text for text, _sync in log.lines]
    assert texts[0] == 'stale: no fresh battery data (bridge not running?)'
    assert texts[1].startswith('low: 6.70 V at rest, below 6.80 V: Pi shutdown in 30 s')
    assert [text.split('in ')[-1] for text in texts[2:]] == ['20 s', '10 s']
    watch = json.loads((tmp_path / 'watch.json').read_text())
    assert watch['state'] == 'low' and watch['shutdown_in_s'] == 6.0


def test_the_log_file_line_is_written_and_synced(tmp_path):
    class Clock:
        @staticmethod
        def time():
            return 1792700000.0

    out = []

    class Out:
        def write(self, text):
            out.append(text)

        def flush(self):
            pass

    log = bw.Log(path=str(tmp_path / 'watch.log'), out=Out(), clock=Clock)
    log('SHUTDOWN: test', sync=True)
    assert (tmp_path / 'watch.log').read_text().endswith(' SHUTDOWN: test\n')
    assert ''.join(out).endswith(' SHUTDOWN: test\n')
