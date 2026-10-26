#!/usr/bin/python3
"""Low-battery shutdown of the Pi (group 5): a host systemd service, not part of the container.

The Pi runs from the pack (buck #1, header pins 4/6, since 2026-10-16). After the firmware's
cutoff the motor stops, but the Pi keeps draining the pack. This watcher shuts the Pi down when
the pack is flat at rest. That slows the drain; it does not stop it (a halted Pi, buck #2 and the
servo, the divider and the Nucleo on the Pi's USB keep drawing): unplug the pack after a
shutdown, and never leave it plugged in unattended.

Input: /run/rover/battery.json, written by the ROS bridge once a second (rover_bridge/power_state.py):
boot_id, status_monotonic (CLOCK_MONOTONIC, shared by the container and the host, when the bridge
got the STATUS), battery_state, vbat_mv, motor_state. Output: /run/rover/watch.json (this
watcher's state; the bridge puts it in /diagnostics and the teleop status line shows the
countdown), log lines on stdout (the journal) and in /var/log/rover-battery-watch.log.

The rule (Rule.update):
  - a sample counts when it is fresh (this boot, a STATUS newer than the last one counted and
    at most STALE_SECONDS old), the pack is measured (OK, LOW or CUTOFF), the motor isn't
    RUNNING, and vbat < THRESHOLD_MV (6800 mV, 3.4 V per cell: above the firmware's 6.6 V cutoff,
    a margin for what a halted Pi still drains)
  - HOLD_SECONDS (30 s) of counting samples without a reset -> shut down (once: latched)
  - reset: a fresh at-rest sample at or above the threshold, or no pack (NOT_PRESENT: wall power,
    REMOVED_WHILE_DRIVING)
  - pause (the hold kept, not counted): stale or missing data (the bridge not running, e.g. a
    motor_console session; the board unplugged; a file from an earlier boot; garbled JSON), the
    motor RUNNING (sag under load is not the resting voltage; a resting pack under 6.6 V makes
    the firmware cut the motor off within ~3 s anyway), ADC_FAULT (the voltage can't be trusted)
  - never a shutdown on missing data
Before the shutdown: the SHUTDOWN line, fsynced, in the log file; then /usr/sbin/shutdown -h now.
The bridge's own way out (docker stop at the system shutdown) sends ESTOP x3.

Environment (EnvironmentFile /etc/default/rover-battery-watch): THRESHOLD_MV, HOLD_S, ACTION
(shutdown, the default, or log: say "would shut down" and do nothing; for the bench).
Install, uninstall and the bench test: docs/ros-bridge.md, Low-battery shutdown.
"""
import json
import os
import subprocess
import sys
import time

STATE_FILE = '/run/rover/battery.json'
WATCH_FILE = '/run/rover/watch.json'
LOG_FILE = '/var/log/rover-battery-watch.log'
BOOT_ID_FILE = '/proc/sys/kernel/random/boot_id'
SHUTDOWN_COMMAND = ['/usr/sbin/shutdown', '-h', 'now']
THRESHOLD_MV = 6800
HOLD_SECONDS = 30.0
STALE_SECONDS = 5.0
MAX_STEP_SECONDS = 2.0                 # one sample adds at most this to the hold (the bridge writes at 1 Hz)
POLL_SECONDS = 1.0
REPORT_EVERY_SECONDS = 10.0
# The firmware's BatteryState (battery_monitor.h) and MotorState (motor_control.h).
NOT_PRESENT, OK, LOW, CUTOFF, ADC_FAULT, REMOVED_WHILE_DRIVING = range(6)
MEASURED = (OK, LOW, CUTOFF)
RUNNING = 1


class Rule:
    """The shutdown rule, on an injected clock. update(sample, now) -> the watch state (a dict)."""

    def __init__(self, boot_id, threshold_mv=THRESHOLD_MV, hold_seconds=HOLD_SECONDS):
        self.boot_id = boot_id
        self.threshold_mv = threshold_mv
        self.hold_seconds = hold_seconds
        self.held = 0.0                      # s of counting samples since the last reset
        self.last_counted = None             # status_monotonic of the last sample that counted
        self.shutdown = False                # latched
        self.state = 'starting'

    def update(self, sample, now):
        """sample: the bridge's battery.json as a dict, or None (missing or unreadable)."""
        if self.shutdown:
            return self._watch(now, 'shutdown', 'shutting down')
        if not self._fresh(sample, now):
            self.last_counted = None         # a pause: the next fresh sample adds nothing for the gap
            return self._watch(now, 'stale', 'no fresh battery data (bridge not running?)')
        state, vbat, motor = sample['battery_state'], sample['vbat_mv'], sample['motor_state']
        if state in (NOT_PRESENT, REMOVED_WHILE_DRIVING):
            self._reset()
            return self._watch(now, 'no pack', 'no pack (wall power)')
        if state not in MEASURED:
            self.last_counted = None
            return self._watch(now, 'adc fault', 'the board reports an ADC fault: the voltage is not trusted')
        if motor == RUNNING:
            self.last_counted = None
            return self._watch(now, 'running', f'{vbat / 1000:.2f} V under load (not counted)')
        if vbat >= self.threshold_mv:
            self._reset()
            return self._watch(now, 'ok', f'{vbat / 1000:.2f} V at rest')
        status_at = sample['status_monotonic']
        if self.last_counted is not None:
            if status_at <= self.last_counted:
                return self._watch(now, 'low', self._low_text(vbat))   # the same sample again: no new evidence
            self.held += min(status_at - self.last_counted, MAX_STEP_SECONDS)
        self.last_counted = status_at
        if self.held >= self.hold_seconds:
            self.shutdown = True
            return self._watch(now, 'shutdown', f'{vbat / 1000:.2f} V at rest for {self.held:.0f} s: shutting down')
        return self._watch(now, 'low', self._low_text(vbat))

    def _fresh(self, sample, now):
        try:
            return (sample is not None and sample['boot_id'] == self.boot_id
                    and 0.0 <= now - sample['status_monotonic'] <= STALE_SECONDS
                    and all(isinstance(sample[key], int) for key in ('battery_state', 'vbat_mv', 'motor_state')))
        except (KeyError, TypeError):
            return False

    def _reset(self):
        self.held = 0.0
        self.last_counted = None

    def _low_text(self, vbat):
        return (f'{vbat / 1000:.2f} V at rest, below {self.threshold_mv / 1000:.2f} V: '
                f'Pi shutdown in {self.remaining():.0f} s')

    def remaining(self):
        return max(self.hold_seconds - self.held, 0.0)

    def _watch(self, now, state, text):
        self.state = state
        return {'state': state, 'text': text, 'held_s': round(self.held, 1),
                'shutdown_in_s': round(self.remaining(), 1) if state == 'low' else None,
                'threshold_mv': self.threshold_mv, 'boot_id': self.boot_id, 'monotonic': now}


def read_json(path):
    try:
        with open(path, encoding='utf-8') as file:
            data = json.load(file)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def write_atomic(path, data):
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as file:
        json.dump(data, file)
    os.chmod(temporary, 0o644)
    os.replace(temporary, path)


def settings(environ):
    """(threshold_mv, hold_seconds, action, problems): the environment's overrides, defaults for
    anything missing or invalid (each one named in problems)."""
    problems = []
    threshold, hold, action = THRESHOLD_MV, HOLD_SECONDS, 'shutdown'
    if environ.get('THRESHOLD_MV'):
        try:
            threshold = int(environ['THRESHOLD_MV'])
            if not 5000 <= threshold <= 9000:
                raise ValueError
        except ValueError:
            threshold = THRESHOLD_MV
            problems.append(f"THRESHOLD_MV={environ['THRESHOLD_MV']!r} invalid (5000-9000): {THRESHOLD_MV}")
    if environ.get('HOLD_S'):
        try:
            hold = float(environ['HOLD_S'])
            if not 1.0 <= hold <= 600.0:
                raise ValueError
        except ValueError:
            hold = HOLD_SECONDS
            problems.append(f"HOLD_S={environ['HOLD_S']!r} invalid (1-600): {HOLD_SECONDS:.0f}")
    if environ.get('ACTION'):
        if environ['ACTION'] in ('shutdown', 'log'):
            action = environ['ACTION']
        else:
            problems.append(f"ACTION={environ['ACTION']!r} invalid (shutdown or log): shutdown")
    return threshold, hold, action, problems


class Watcher:
    """One poll per step(): read, decide, write watch.json, log changes, act once."""

    def __init__(self, rule, action, log, run=subprocess.run, state_file=STATE_FILE, watch_file=WATCH_FILE):
        self.rule, self.action, self.log, self.run = rule, action, log, run
        self.state_file, self.watch_file = state_file, watch_file
        self.previous_state = None
        self.last_report = None
        self.acted = False

    def step(self, now):
        watch = self.rule.update(read_json(self.state_file), now)
        watch['action'] = self.action
        try:
            write_atomic(self.watch_file, watch)
        except OSError:
            pass                              # /run/rover missing: the rule still acts
        if watch['state'] != self.previous_state:
            self.log(f"{watch['state']}: {watch['text']}")
            self.previous_state, self.last_report = watch['state'], now
        elif watch['state'] == 'low' and now - self.last_report >= REPORT_EVERY_SECONDS:
            self.log(f"{watch['state']}: {watch['text']}")
            self.last_report = now
        if self.rule.shutdown and not self.acted:
            self.acted = True
            self.log(f"SHUTDOWN: {watch['text']} (threshold {self.rule.threshold_mv} mV, hold "
                     f'{self.rule.hold_seconds:.0f} s, action {self.action})', sync=True)
            if self.action == 'shutdown':
                self.run(SHUTDOWN_COMMAND, check=False)
            else:
                self.log('would shut down now (ACTION=log): nothing done', sync=True)
        return watch


class Log:
    """Lines to stdout (the journal) and appended to the log file; sync=True fsyncs the file."""

    def __init__(self, path=LOG_FILE, out=sys.stdout, clock=time):
        self.path, self.out, self.clock = path, out, clock

    def __call__(self, text, sync=False):
        line = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.clock.time())) + ' ' + text
        print(line, file=self.out, flush=True)
        try:
            with open(self.path, 'a', encoding='utf-8') as file:
                file.write(line + '\n')
                if sync:
                    file.flush()
                    os.fsync(file.fileno())
        except OSError:
            pass


def main():
    with open(BOOT_ID_FILE, encoding='ascii') as file:
        boot_id = file.read().strip()
    threshold, hold, action, problems = settings(os.environ)
    log = Log()
    for problem in problems:
        log('config: ' + problem)
    log(f'started: threshold {threshold} mV at rest, hold {hold:.0f} s, action {action}, reading {STATE_FILE}')
    watcher = Watcher(Rule(boot_id, threshold, hold), action, log)
    while True:
        watcher.step(time.monotonic())
        time.sleep(POLL_SECONDS)


if __name__ == '__main__':
    main()
