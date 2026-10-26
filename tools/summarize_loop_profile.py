#!/usr/bin/env python3
"""TEMPORARY: summarize the firmware's loop timing profile from a serial log.

Delete this file together with the LOOP_TIMING_PROFILE instrumentation in
firmware/Core/Src/main.c once the measurement is done.

With LOOP_TIMING_PROFILE = 1 the firmware sends, once per 100 ms window, an
extra line after the normal telemetry line:

  prof t_ms=... idle_n=... idle_enc=... ... spd_tot=... mot=... ctl_us=... est_us=... bld_tot=...

Every value except t_ms and *_n is the largest time (us) seen in that window
for that section of that kind of iteration; "mot" is the largest motor section
(command input + speed control + motor and steering state machines + pin
writes, and the 10 ms battery sample) over all iterations of the window;
"ctl_us" the largest speed control + stall check inside it, and "est_us" the
largest angle correction + corrected unwrap + speed estimate inside the
encoder section. Report iteration 4 formats the reset cause, steering and
battery fields; iteration 5 (telemetry v5) the speed control fields and
starts the UART. Telemetry lines of v1 to v5 all count as good
(serial_logger.classify). Profile lines from older firmware are not read by
this version: before the motor driver (tail_tx, no motr_* or mot), before
telemetry v3 (motr_tx, no strg_*) and before v5 (strg_tx, no spd_*, ctl_us,
est_us). This script reads a log
written by tools/serial_logger.py (read-only) and prints, per section:
  median  - the typical window maximum
  worst   - the largest window maximum in the whole log
  share   - median of the section / median of its iteration's total

serial_logger.py counts the prof lines as "bad" (it only knows telemetry and
boot lines). This script separates them out, so "other bad" below is the
number that has to be 0.

Usage:  py tools\\summarize_loop_profile.py C:\\dev\\rover-logs\\loop_profile.log
        python3 tools/summarize_loop_profile.py loop_profile.log
"""
import re
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from serial_logger import classify  # noqa: E402  (same good/boot/bad rules as the logger)

# Field order exactly as Profile_BuildLineStep() in main.c prints them.
PROFILE_KEYS = (
    't_ms',
    'idle_n', 'idle_enc', 'idle_imu', 'idle_oth', 'idle_tot',
    'busy_n', 'busy_enc', 'busy_imu', 'busy_oth', 'busy_tot',
    'hlth_enc', 'hlth_rd', 'hlth_imu', 'hlth_oth', 'hlth_tot',
    'head_enc', 'head_imu', 'head_fmt', 'head_oth', 'head_tot',
    'tail_enc', 'tail_imu', 'tail_fmt', 'tail_oth', 'tail_tot',
    'motr_enc', 'motr_imu', 'motr_fmt', 'motr_oth', 'motr_tot',
    'strg_enc', 'strg_imu', 'strg_fmt', 'strg_oth', 'strg_tot',
    'spd_enc', 'spd_imu', 'spd_fmt', 'spd_tx', 'spd_oth', 'spd_tot',
    'mot', 'ctl_us', 'est_us', 'bld_tot',
)
PROFILE_LINE = re.compile(
    rb'^prof ' + rb' '.join(key.encode() + rb'=(\d+)' for key in PROFILE_KEYS) + rb'\r\n$'
)

# (prefix, description, sections). Shares are relative to <prefix>_tot.
ITERATION_KINDS = (
    ('idle', 'normal iteration, UART idle', ('enc', 'imu', 'oth')),
    ('busy', 'normal iteration, UART sending', ('enc', 'imu', 'oth')),
    ('hlth', 'health iteration (t%100 == 50)', ('enc', 'rd', 'imu', 'oth')),
    ('head', 'report iteration 1 (encoder part)', ('enc', 'imu', 'fmt', 'oth')),
    ('tail', 'report iteration 2 (IMU part)', ('enc', 'imu', 'fmt', 'oth')),
    ('motr', 'report iteration 3 (motor part)', ('enc', 'imu', 'fmt', 'oth')),
    ('strg', 'report iteration 4 (reset cause + steering + battery part)', ('enc', 'imu', 'fmt', 'oth')),
    ('spd', 'report iteration 5 (speed control part + UART start)', ('enc', 'imu', 'fmt', 'tx', 'oth')),
    ('bld', 'profile-line build (instrumentation overhead)', ()),
)
SECTION_NAMES = {
    'enc': 'encoder (RAW ANGLE + MAGNITUDE)',
    'rd': 'health reads (STATUS + AGC)',
    'imu': 'IMU burst read',
    'fmt': 'line formatting',
    'tx': 'UART start (HAL_UART_Transmit_IT)',
    'oth': 'everything else (not the motor section)',
    'tot': 'whole iteration',
}


def read_log(path):
    lines = Path(path).read_bytes().split(b'\n')
    lines.pop()  # '' after the last '\n', or an unfinished line
    counts = {'good': 0, 'boot': 0, 'prof': 0, 'other bad': 0}
    profiles, loop_us_max_values = [], []
    examples = []
    for raw_line in lines:
        line = raw_line + b'\n'
        match = PROFILE_LINE.match(line)
        if match:
            counts['prof'] += 1
            profiles.append(dict(zip(PROFILE_KEYS, (int(value) for value in match.groups()))))
            continue
        kind = classify(line)
        if kind == 'good':
            counts['good'] += 1
            loop_us_max_values.append(int(re.search(rb' loop_us_max=(\d+)', line).group(1)))
        elif kind == 'boot':
            counts['boot'] += 1
        else:
            counts['other bad'] += 1
            if len(examples) < 3:
                examples.append(line)
    return counts, profiles, loop_us_max_values, examples


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    counts, profiles, loop_us_max_values, examples = read_log(sys.argv[1])

    print('Lines: ' + '  '.join(f'{name}={count}' for name, count in counts.items()))
    print('  (the logger reports bad = prof + other bad; only "other bad" is a real problem.'
          ' The first line of the log can be a cut-off partial line.)')
    for example in examples:
        print(f'  other bad example: {example!r}')
    if not profiles:
        print('No prof lines found. Was the firmware built with LOOP_TIMING_PROFILE = 1?')
        return 1

    if loop_us_max_values:
        print(f'\nTelemetry loop_us_max: median {statistics.median(loop_us_max_values):.0f} us, '
              f'worst {max(loop_us_max_values)} us ({len(loop_us_max_values)} lines)')

    motor_section_values = [profile['mot'] for profile in profiles]
    print(f'\nMotor section (every iteration: command input + speed control + motor and steering state machines'
          f' + pin writes + battery sample): '
          f'median of window maxima {statistics.median(motor_section_values):.0f} us, '
          f'worst {max(motor_section_values)} us')
    for key, description in (('ctl_us', 'speed control + stall check (part of the motor section)'),
                             ('est_us', 'angle correction + corrected unwrap + speed estimate (part of encoder)')):
        values = [profile[key] for profile in profiles]
        print(f'  {description}: median of window maxima {statistics.median(values):.0f} us, '
              f'worst {max(values)} us')

    for prefix, description, sections in ITERATION_KINDS:
        total_key = f'{prefix}_tot'
        # A window without this kind of iteration reports 0 (e.g. the first
        # window after a reset has no report iterations), so leave those out.
        windows = [profile for profile in profiles if profile[total_key] > 0]
        print(f'\n{description}: {len(windows)} windows', end='')
        if prefix in ('idle', 'busy') and windows:
            iterations = [profile[f'{prefix}_n'] for profile in windows]
            print(f', median {statistics.median(iterations):.0f} iterations per window', end='')
        print()
        if not windows:
            continue
        total_median = statistics.median(profile[total_key] for profile in windows)
        print(f'  {"section":<36} {"median":>7} {"worst":>7} {"share":>7}')
        for section in sections + ('tot',):
            values = [profile[f'{prefix}_{section}'] for profile in windows]
            median = statistics.median(values)
            share = f'{100.0 * median / total_median:.0f}%' if total_median else '-'
            print(f'  {SECTION_NAMES[section]:<36} {median:>7.0f} {max(values):>7} {share:>7}')

    print('\nNotes:')
    print('  - health and report iterations happen once per window, so their sections add up to')
    print('    their total. Normal iterations are ~40-60 per window and each section\'s maximum')
    print('    can come from a different iteration, so their shares need not add up to 100%.')
    print('  - "busy" minus "idle" is roughly what the UART send interrupts cost one iteration.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
