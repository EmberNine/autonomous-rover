"""Mutation testing for the ROS bridge's driving side (rover_bridge group 3 follow-ups, 2026-10-19):
proves the tests notice the bugs that matter, by planting each one and requiring a test to fail.

Like run_link_mutants.py: each mutant is one textual change (old -> new, old must occur exactly
once) in a copy of the sources in a temporary directory; the working tree is never touched. The
unmutated copy must pass first. A mutant runs its test target (the whole rover_bridge test folder,
or tools/test_clock_step_probe.py for the probe, tools/test_battery_watch.py for the shutdown
watcher) with -x. KILLED when a test fails (pytest exit code 1), SURVIVED when all pass, INVALID
when the text doesn't occur exactly once, BROKEN on any other exit code: a mutant that breaks
the file itself (a collection error, 2026-10-22: V5 left half a comment behind and "killed" by
a SyntaxError) proves nothing about the tests.

The node tests run against a fake board in real time, so a kill by one of them must repeat: the
mutant runs again and only a second failure counts; a pass on the rerun is reported as FLAKY (a
timing failure, not a kill).

Run from the repo root, in WSL with ROS 2 Jazzy sourced (the node tests need rclpy; without it
they skip, and the mutants only they can kill survive):
    source /opt/ros/jazzy/setup.bash
    python3 tools/run_bridge_mutants.py          # all 97 (~2 h: each runs the whole folder, ~45 s)
    python3 tools/run_bridge_mutants.py R X D    # only ids starting with R, X or D (~25 min)
    python3 tools/run_bridge_mutants.py K X      # the teleop deadman and the control (~10 min)
    python3 tools/run_bridge_mutants.py V X      # the low-battery shutdown and the control (~3 min)

Exit code 0 only if every mutant was killed and every control (CONTROLS: a comment-only change,
X0) survived: a killed control means the runner, not the code, is broken. Its row is marked
"(control)". tools/test_run_bridge_mutants.py checks that each mutant's text still occurs exactly
once in its file, and that a control's change is only in a comment.
"""
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from run_link_mutants import Mutant

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
COPIED = ('tools', 'config', 'ros_ws/src/rover_bridge', 'ros_ws/docker')
BRIDGE_TESTS = 'ros_ws/src/rover_bridge/test'
PROBE_TESTS = 'tools/test_clock_step_probe.py'
WATCH_TESTS = 'tools/test_battery_watch.py'
RUN_TIMEOUT_SECONDS = 900

S = 'ros_ws/src/rover_bridge/rover_bridge/safety.py'
N = 'ros_ws/src/rover_bridge/rover_bridge/node.py'
D = 'ros_ws/src/rover_bridge/rover_bridge/diagnostics.py'
F = 'ros_ws/src/rover_bridge/test/bridge_fakes.py'
P = 'tools/clock_step_probe.py'
L = 'ros_ws/src/rover_bridge/rover_bridge/link.py'
K = 'ros_ws/src/rover_bridge/rover_bridge/teleop.py'
KN = 'ros_ws/src/rover_bridge/rover_bridge/teleop_node.py'
W = 'tools/battery_watch.py'

MUTANTS = [
    # --- G: the clock step probe
    Mutant('G1', P, 'abs(offset - previous) > self.threshold', 'abs(offset) > self.threshold',
           'a step is the offset itself, not its change'),
    Mutant('G2', P, 'abs(offset - previous) > self.threshold', 'offset - previous > self.threshold',
           'only forward steps found'),
    Mutant('G3', P, 'if after - before > slack:', 'if False:', 'interrupted reads kept'),
    Mutant('G4', P, 'steady - previous > self.threshold', 'steady - previous >= self.threshold - 0.02',
           'gaps of 130 ms reported'),
    # --- A: every safety event logged
    Mutant('A1', N, "        for warn, event in self.safety.take_events(self.clock.monotonic()):\n",
           "        for warn, event in [(w, e) for w, e in self.safety.take_events(self.clock.monotonic())][-1:]:\n"
           "            if event == getattr(self, '_logged', None):\n                continue\n"
           "            self._logged = event\n",
           'node: the old text dedup (polling last_event)'),
    Mutant('A2', N, 'for warn, event in self.safety.take_events(self.clock.monotonic()):',
           'for warn, event in self.safety.take_events(self.clock.monotonic())[-1:]:',
           'node: only the latest event per poll'),
    Mutant('A3', N, "        for warn, event in self.safety.take_events(self.clock.monotonic()):\n",
           "        for warn, event in self.safety.take_events(self.clock.monotonic()):\n"
           "            if event == getattr(self, '_logged', None):\n                continue\n"
           "            self._logged = event\n",
           'node: a repeated text skipped'),
    Mutant('A4', S, "RATE_LIMITED = {'heartbeat stop', 'heartbeat resume', 'slow gap'}",
           "RATE_LIMITED = {'heartbeat stop', 'heartbeat resume', 'slow gap', 'estop'}", 'ESTOPs rate-limited too'),
    Mutant('A5', S, '                    if held:\n', '                    if False:\n',
           'the held-back count never reported'),
    Mutant('A6', S, "self.counts['heartbeat stops'] += 1", "self.counts['heartbeat stops'] = 1",
           'the stop counter limited like the log'),
    Mutant('A7', S, '                    self.stopped_for_heartbeat = False\n                    self.last_stop_gap = gap',
           '                    self.last_stop_gap = gap', 'the stop flag not reset when /cmd_vel returns'),
    Mutant('A8', S, "WARN_EVENTS = {'refused', 'estop', 'heartbeat stop'}", "WARN_EVENTS = {'refused', 'estop'}",
           'a heartbeat stop logged as INFO'),
    Mutant('A9', S, 'if logged_at is not None and now - logged_at < EVENT_LOG_SECONDS:', 'if False:',
           'no rate limit at all'),
    # --- B: /cmd_vel gap counters
    Mutant('B1', S, 'self.stops_since_arm, self.gap_max, self.last_stop_gap = 0, None, None',
           'self.stops_since_arm, self.last_stop_gap = 0, None', 'gap max not reset at arm'),
    Mutant('B2', S, 'self.gap_max = gap if self.gap_max is None else max(self.gap_max, gap)',
           'self.gap_max = gap if self.gap_max is None else min(self.gap_max, gap)', 'gap max takes the min'),
    Mutant('B3', S, '                self.gap_max = gap if self.gap_max is None else max(self.gap_max, gap)\n',
           '                self.gap_max = gap if self.gap_max is None else max(self.gap_max, gap)\n'
           '                self.last_stop_gap = gap\n', 'last stop gap overwritten by every /cmd_vel'),
    Mutant('B4', S, 'if self.state in (ARMING, ARMED) and self.heartbeat_at is not None:',
           'if self.heartbeat_at is not None:', 'gaps counted while DISARMED'),
    Mutant('B5', S, '            if age is not None:\n', '            if False:\n', "the ESTOP's gap not counted"),
    Mutant('B6', S, 'self.stops_since_arm += 1', 'pass', 'stops since arm not counted'),
    Mutant('B7', S, 'self.stops_since_arm, self.gap_max, self.last_stop_gap = 0, None, None',
           'self.gap_max, self.last_stop_gap = None, None', 'stops since arm not reset at arm'),
    Mutant('B8', D, "_ms(safety['last_stop_gap'])", "_ms(safety['gap_max'])",
           'diagnostics: the last stop gap shows the max'),
    Mutant('B9', S, "f'/cmd_vel back after {gap * 1000:.0f} ms: driving again'",
           "f'/cmd_vel back after {self.gap_max * 1000:.0f} ms: driving again'",
           'the resume line reports the longest gap'),
    # --- C: the last event's age
    Mutant('C1', S, 'self.last_event, self.last_event_at = text, now',
           'self.last_event, self.last_event_at = text, (now if self.last_event_at is None else self.last_event_at)',
           'age counted from the first event'),
    Mutant('C2', S, 'self.last_event, self.last_event_at = text, now', 'self.last_event = text', 'age never recorded'),
    Mutant('C3', N, "(last event {age_text(safety['last_event_age'])}: ", '(', 'the 10 s line without the age'),
    Mutant('C4', D, "f\"{safety['state']}: {safety['last_event']} ({when})\"",
           "f\"{safety['state']}: {safety['last_event']}\"", 'the OK message without the age'),
    # --- D: setpoint echo vs units, command vs DRIVE sent
    Mutant('D1', D, "rover_link.cps_to_units(status_fields['setpoint_cps']) * rover_link.CPS_PER_UNIT",
           "status_fields['setpoint_cps']", 'the units key shows the raw echo'),
    Mutant('D2', D, "('drive cps (sent now)', 'n/a' if safety['drive_cps'] is None else safety['drive_cps'])",
           "('drive cps (sent now)', safety['command_cps'])", 'diagnostics: drive sent shows the command'),
    Mutant('D3', S, 'self.drive_cps = frames[0][1] if drive else None',
           'self.drive_cps = self.command_cps', 'tick records the command, not the DRIVE'),
    Mutant('D4', S, 'self.drive_cps = frames[0][1] if drive else None',
           'self.drive_cps = (frames[0][1] if drive else 0) if frames else None',
           'an ESTOP counted as a DRIVE 0'),
    Mutant('D5', F, "self.setpoint_cps = drive['speed_cps']",
           "self.setpoint_cps = rover_link.cps_to_units(drive['speed_cps']) * rover_link.CPS_PER_UNIT",
           'the fake board echoes the quantized setpoint again'),
    Mutant('D6', N, "sending = 'none' if safety['drive_cps'] is None else f\"{safety['drive_cps']}cps\"",
           "sending = f\"{safety['command_cps']}cps\"", 'the 10 s line: sending= shows the command'),
    # --- G (2): what the probe's gaps cover, monotonic vs the corrected wall clock
    Mutant('G5', P, 'self.received += 1', 'pass', 'the received count never counts'),
    Mutant('G6', P, 'return (steps[-1][0] - steps[0][0]) / (walls[-1] - walls[0]) - 1.0',
           'return (walls[-1] - walls[0]) / (steps[-1][0] - steps[0][0]) - 1.0', 'the monotonic rate inverted'),
    # --- S: /cmd_vel gaps of 150 ms or more
    Mutant('S1', S, 'SLOW_GAP_SECONDS = 0.15 ', 'SLOW_GAP_SECONDS = 0.30 ', 'slow gaps only from 300 ms'),
    Mutant('S2', S, '            self.slow_gaps_since_arm = 0\n', '', 'slow gaps not reset at arm'),
    Mutant('S3', S, 'elif gap >= SLOW_GAP_SECONDS:', 'if gap >= SLOW_GAP_SECONDS:', "a stop's gap logged as slow too"),
    Mutant('S4', S, 'if gap >= SLOW_GAP_SECONDS:\n                    self.slow_gaps_since_arm += 1',
           'if True:\n                    self.slow_gaps_since_arm += 1', 'every /cmd_vel counted as a slow gap'),
    Mutant('S5', S, "WARN_EVENTS = {'refused', 'estop', 'heartbeat stop'}",
           "WARN_EVENTS = {'refused', 'estop', 'heartbeat stop', 'slow gap'}", 'a slow gap logged as WARN'),
    Mutant('S6', S, "RATE_LIMITED = {'heartbeat stop', 'heartbeat resume', 'slow gap'}",
           "RATE_LIMITED = {'heartbeat stop', 'heartbeat resume'}", 'slow-gap lines not rate-limited'),
    Mutant('S7', D, "('cmd_vel gaps >= 150 ms (since arm)', safety['slow_gaps_since_arm'])",
           "('cmd_vel gaps >= 150 ms (since arm)', safety['stops_since_arm'])", 'diagnostics: slow gaps show the stops'),
    Mutant('S8', N, "gaps>=150ms {safety['slow_gaps_since_arm']}", "gaps>=150ms {safety['stops_since_arm']}",
           'the 10 s line: slow gaps show the stops'),
    # --- H: held-back lines said within a second
    Mutant('H1', S, 'if held and now - logged_at >= EVENT_LOG_SECONDS:', 'if False:', 'held-back lines never said'),
    Mutant('H2', S, 'if held and now - logged_at >= EVENT_LOG_SECONDS:', 'if held:',
           'held-back lines said before the second is over'),
    Mutant('H3', S, 'self.logged[kind] = (logged_at, 0)', 'pass', 'held-back lines said again and again'),
    # --- W: a seq gap the board dropped
    Mutant('W1', D, 'if board == new_gaps:', 'if board != new_gaps:', 'the board-drop note on the wrong gaps'),
    Mutant('W2', D, "problems[worse.index('seq_gaps')] += f' (dropped by the board: tx_dropped +{board})'", 'pass',
           'the board-drop note never shown'),
    Mutant('W3', D, "        self.previous_tx_dropped = snapshot['status']['tx_dropped'] if snapshot['status'] is not None else None\n",
           '', "the board's previous tx_dropped never kept"),
    Mutant('W4', N, "f'{self.link.stats_text()}{board_drops}; read_gap_max=", "f'{self.link.stats_text()}; read_gap_max=",
           "the 10 s line without the board's tx_dropped"),
    # --- T: the node's timers on a steady clock
    Mutant('T1', N, 'self.create_timer(PUBLISH_PERIOD_SECONDS, self.publish_pending, clock=steady)',
           'self.create_timer(PUBLISH_PERIOD_SECONDS, self.publish_pending)',
           'publish_pending back on the default (wall) clock'),
    Mutant('T2', N, 'self.create_timer(status_period, self.log_status, clock=steady)',
           'self.create_timer(status_period, self.log_status)', 'the 10 s summary back on the wall clock'),
    Mutant('T3', N, 'self.create_timer(diagnostics_period, self.publish_diagnostics, clock=steady)',
           'self.create_timer(diagnostics_period, self.publish_diagnostics)', 'diagnostics back on the wall clock'),
    # --- R: steering from /cmd_vel (2026-10-22)
    Mutant('R1', S, 'delta = math.atan(wheelbase * angular_z / linear_x)',
           'delta = math.atan(-wheelbase * angular_z / linear_x)', 'steering sign inverted'),
    Mutant('R2', S, 'delta = math.atan(wheelbase * angular_z / linear_x)',
           'delta = math.atan(wheelbase * angular_z / abs(linear_x))', 'reversing steers the same way'),
    Mutant('R3', S, 'if abs(delta) > max_angle:', 'if False:', 'no clamp at full lock'),
    Mutant('R4', S, "delta, what = math.copysign(max_angle, delta), 'clamped'",
           "delta, what = math.copysign(0.246, delta), 'clamped'", 'clamped to the stale 0.246'),
    Mutant('R5', S, 'return round(delta / max_angle * 100), what', 'return int(delta / max_angle * 100), what',
           'percent truncated, not rounded'),
    Mutant('R6', S, '            elif cps == 0:\n',
           '            elif cps == 0:\n                self.command_steer = CENTRE\n',
           'v ~ 0 centres instead of holding'),
    Mutant('R7', S, "self.counts['steering invalid'] += 1\n                self.command_steer = CENTRE\n",
           "self.counts['steering invalid'] += 1\n", 'a non-finite command holds the steering'),
    Mutant('R8', S, '        self.arm_bit_pending = False\n        self.command_steer = CENTRE\n\n    def _event',
           '        self.arm_bit_pending = False\n\n    def _event', 'the held steering survives a disarm'),
    Mutant('R9', S, "        self.command_steer = CENTRE\n        self._event('estop'", "        self._event('estop'",
           'the held steering survives an ESTOP'),
    Mutant('R10', S, '            self.arming_since = now\n            self.command_steer = CENTRE\n',
           '            self.arming_since = now\n', 'arming keeps the steering from before'),
    Mutant('R11', S, "                self.command_steer = CENTRE\n                return [('DRIVE', 0, CENTRE, False)]",
           "                return [('DRIVE', 0, self.command_steer, False)]", 'the heartbeat stop keeps the wheels turned'),
    Mutant('R12', S, "return [('DRIVE', 0, CENTRE, arm)]", "return [('DRIVE', 0, self.command_steer, arm)]",
           'ARMING steers'),
    Mutant('R13', S, '        return []\n\n    # --- helpers', "        return [('DRIVE', 0, CENTRE, False)]\n\n    # --- helpers",
           'DISARMED sends DRIVEs to centre (option 2)'),
    Mutant('R14', L, 'rover_link.pack_drive(speed, steer, steer_valid=True, arm=arm)',
           'rover_link.pack_drive(speed, steer, steer_valid=False, arm=arm)', 'DRIVE without steer_valid'),
    Mutant('R15', N, "max_steering_angle = conversions.vehicle_value(vehicle_cfg, 'max_steering_angle_radians')",
           'max_steering_angle = 0.246', 'the node ignores the cfg full lock'),
    Mutant('R16', S, 'self.drive_steer = frames[0][2] if drive else None', 'self.drive_steer = self.command_steer',
           'the steering sent shows the command'),
    Mutant('R17', D, "('drive steer % (sent now)', 'n/a' if safety['drive_steer'] is None else safety['drive_steer'])",
           "('drive steer % (sent now)', safety['command_steer'])", 'diagnostics: steering sent shows the command'),
    Mutant('R18', D, "'invalid', 'steering clamped', 'steering invalid')", "'invalid')",
           'no WARN on a steering clamp or a non-finite command'),
    # --- K: the teleop's keyboard-link deadman (2026-10-22)
    Mutant('K1', K, 'LINK_STOP_SECONDS = 0.5', 'LINK_STOP_SECONDS = 1.0', 'the link stop at 1 s, not 0.5 s'),
    Mutant('K2', K, 'LINK_ESTOP_SECONDS = 2.0', 'LINK_ESTOP_SECONDS = 3.0', 'the link ESTOP at 3 s, not 2 s'),
    Mutant('K3', K, "            self._stop()\n            self.events.append(f'no keyboard byte",
           "            self.events.append(f'no keyboard byte", 'the link stop does not stop'),
    Mutant('K4', K, '                self.link_stopped = False\n',
           '                self.link_stopped = False\n                self.level = SPEED_LEVELS.index(0.05)\n',
           'keepalives after a link stop drive again'),
    Mutant('K5', K, "            return self._finish(f'no keyboard byte for {age * 1000:.0f} ms')",
           "            self._finish(f'no keyboard byte for {age * 1000:.0f} ms')\n            return []",
           'no /rover/estop at 2 s'),
    Mutant('K6', K, "        return self._finish('keyboard link closed (end of input)')", '        return []',
           'end of input does nothing'),
    Mutant('K7', K, '            self.last_byte_at = now\n            if self.link_stopped:',
           '            if char != KEEPALIVE:\n                self.last_byte_at = now\n            if self.link_stopped:',
           'keepalives do not feed the deadman'),
    Mutant('K8', K, 'self.last_byte_at = now              # the deadman counts from the start too',
           'self.last_byte_at = math.inf', 'no deadman before the first byte'),
    Mutant('K9', K, '        if self.finished is not None:\n            return None\n',
           '        if False:\n            return None\n', 'still published once finished'),
    Mutant('K10', KN, 'self.act(self.teleop.on_eof(at) if text is None else self.teleop.on_bytes(text, at))',
           "self.act([] if text is None else self.teleop.on_bytes(text, at))", 'the node ignores end of input'),
    Mutant('K11', KN, '    node.publish_zero()                                   # the wheel stops even without /rover/estop\n',
           '', 'no /cmd_vel of 0 on the way out'),
    # --- V: the Pi's low-battery shutdown (2026-10-22): the watcher's rule, and the bridge's side
    Mutant('V1', W, 'THRESHOLD_MV = 6800', 'THRESHOLD_MV = 7000', 'the threshold 7000 mV, not 6800'),
    Mutant('V2', W, 'HOLD_SECONDS = 30.0', 'HOLD_SECONDS = 3.0', 'the hold 3 s, not 30 s'),
    Mutant('V3', W, '        if motor == RUNNING:\n', '        if False:\n', 'samples counted while the motor runs'),
    Mutant('V4', W, "            self._reset()\n            return self._watch(now, 'ok'",
           "            return self._watch(now, 'ok'", 'a recovery at rest does not reset the hold'),
    Mutant('V5', W, 'self.last_counted = None         # a pause: the next fresh sample adds nothing for the gap',
           'self._reset()', 'stale data resets the hold'),
    Mutant('V6', W, "0.0 <= now - sample['status_monotonic'] <= STALE_SECONDS",
           "0.0 <= now - sample['status_monotonic']", 'old data counts as fresh'),
    Mutant('V7', W, 'if state in (NOT_PRESENT, REMOVED_WHILE_DRIVING):', 'if state == REMOVED_WHILE_DRIVING:',
           'wall power (no pack) not recognised'),
    Mutant('V8', W, 'MEASURED = (OK, LOW, CUTOFF)', 'MEASURED = (OK, LOW, CUTOFF, ADC_FAULT)',
           'an ADC fault counts'),
    Mutant('V9', W, "return (sample is not None and sample['boot_id'] == self.boot_id",
           'return (sample is not None', 'a file from an earlier boot counts'),
    Mutant('V10', W, 'self.held += min(status_at - self.last_counted, MAX_STEP_SECONDS)',
           'self.held += status_at - self.last_counted', 'one sample adds any gap'),
    Mutant('V11', W, "            if self.action == 'shutdown':\n", '            if True:\n',
           'ACTION=log still shuts down'),
    Mutant('V12', N, 'self.last_status, self.last_status_at, self.boot_id, self.clock.time()))',
           'self.last_status, self.clock.monotonic(), self.boot_id, self.clock.time()))',
           'the bridge stamps the write time: a stale STATUS looks fresh'),
    Mutant('V13', W, "action {self.action})', sync=True)", "action {self.action})', sync=False)",
           'the SHUTDOWN line not synced before the command'),
    Mutant('V14', D, "level = max(report['level'], WARN)", "level = report['level']",
           'the countdown does not raise the battery status to WARN'),
    Mutant('V15', W, "        if self.shutdown:\n            return self._watch(now, 'shutdown', 'shutting down')\n", '',
           'the shutdown decision not latched'),
    # --- X: the control. A comment-only change: it must SURVIVE, or the runner itself is broken
    #     (a kill here would mean every kill above is suspect).
    Mutant('X0', S, 'CENTRE = 0                             # steering %, sent explicitly',
           'CENTRE = 0                             # steering %, sent explicitly (control mutant)',
           'no-op control: must survive'),
]
CONTROLS = {'X0'}


def test_target(mutant):
    return {P: PROBE_TESTS, W: WATCH_TESTS}.get(mutant.path, BRIDGE_TESTS)


def copy_tree(destination):
    for part in COPIED:
        shutil.copytree(REPOSITORY_ROOT / part, destination / part, ignore=shutil.ignore_patterns('__pycache__'))


def run(mutant, target):
    """('KILLED' | 'SURVIVED' | 'INVALID', detail). mutant None: the unmutated copy."""
    with tempfile.TemporaryDirectory(prefix='bridge_mutants_') as temporary:
        root = Path(temporary)
        copy_tree(root)
        if mutant is not None:
            path = root / mutant.path
            text = path.read_text(encoding='utf-8')
            if text.count(mutant.old) != 1:
                return 'INVALID', f'text occurs {text.count(mutant.old)} times'
            path.write_text(text.replace(mutant.old, mutant.new), encoding='utf-8', newline='')
        result = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-x', '-p', 'no:cacheprovider', target],
                                cwd=root, capture_output=True, text=True, timeout=RUN_TIMEOUT_SECONDS,
                                env=dict(os.environ))
    failed = [line[len('FAILED '):] for line in result.stdout.splitlines() if line.startswith('FAILED ')]
    last = (result.stdout.strip().splitlines() or [''])[-1]
    return verdict(result.returncode), (failed[0] if failed else last)


def verdict(returncode):
    """pytest's exit code -> the verdict: 1 tests failed, 0 all passed, anything else (2 a
    collection error or interruption, 3 internal, 4 usage, 5 no tests) broke the run itself."""
    return {0: 'SURVIVED', 1: 'KILLED'}.get(returncode, 'BROKEN')


def evaluate(mutant):
    verdict, detail = run(mutant, test_target(mutant))
    if verdict == 'KILLED' and 'test_bridge_node.py' in detail:          # real time: it must fail again
        again, detail_again = run(mutant, test_target(mutant))
        if again != 'KILLED':
            return 'FLAKY', detail + ' (passed on the rerun)'
        detail += ' (again: ' + detail_again.split('::')[-1] + ')'
    return verdict, detail


def main():
    selected = [m for m in MUTANTS if len(sys.argv) < 2 or m.id.startswith(tuple(sys.argv[1:]))]
    for target in sorted({test_target(m) for m in selected}):
        verdict, detail = run(None, target)
        if verdict != 'SURVIVED':
            print(f'BASELINE FAILS ({target}): {detail}')
            return 2
    print(f'baseline passes; {len(selected)} mutants')
    counts, wrong = {}, []
    for mutant in selected:
        started = time.monotonic()
        verdict, detail = evaluate(mutant)
        counts[verdict] = counts.get(verdict, 0) + 1
        expected = 'SURVIVED' if mutant.id in CONTROLS else 'KILLED'
        if verdict != expected:
            wrong.append(mutant.id)
        label = mutant.bug + (' (control)' if mutant.id in CONTROLS else '')
        print(f'{verdict:8s} {mutant.id:3s} {label:52s} {time.monotonic() - started:5.1f}s  {detail}', flush=True)
    print(', '.join(f'{count} {verdict.lower()}' for verdict, count in sorted(counts.items())) + f' of {len(selected)}'
          + (f'; not as expected: {" ".join(wrong)}' if wrong else ''))
    return 1 if wrong else 0


if __name__ == '__main__':
    sys.exit(main())
