#!/usr/bin/env python3
"""Fit the encoder's 4-per-revolution angle error (H4) from step captures.

The AS5600 on this rover reads the wheel angle with an error that repeats 4
times per revolution, about +-44 counts (it looks like +-24 % speed
ripple). It is the same at every speed and in both directions, which points
to a sensor/magnet measurement error rather than the wheel: likely, not
proven (no rotation test is possible on this mount). The firmware removes it
from the raw angle (firmware/Core/Inc/angle_correction.h):
    eps(raw)  = A * sin(2 pi (raw + r0) / 1024)       counts
    corrected = raw - eps(raw)
This tool fits A and r0 from captures (serial logs of a SPEED_CAPTURE build,
see tools/fit_motor_step.py) and prints them as the firmware constants.

Per capture (its steady segment), the position is fitted as a straight line
plus harmonics 1..8 of the measured wheel angle phi = 2 pi raw / 4096, all
at once so none leaks into another. Only H4 goes into the correction: H1 is
partly real (a mechanical once-per-turn tight spot: it shrinks with speed),
and H2/H3/H8 aren't the same from capture to capture.

Then:
  - invariance: H4 should have the same amplitude (within 10 %) and phase
    (within 10 degrees) in every capture; if not, a WARNING (the board may
    have moved during the session, or a capture is bad);
  - the constants, from the capture with the most revolutions at steady
    speed, or the one named with --from (log file name, optionally :id);
  - the predicted residual per capture with those (rounded) constants:
    16-bin ripple, H4 and H1 in % of the mean speed, and the std of the
    speed over 20 ms (the speed estimator's window) and 100 ms (d100).

The correction is tied to how the encoder board is seated (on its pegs, not
glued): re-fit if ML or the magnitude at rest change, or if the 4-per-turn
ripple comes back.

Usage (WSL):  python3 tools/fit_angle_correction.py /mnt/c/dev/rover-logs/cap_*_1.log [--from cap_m30_c2_1.log]
"""
import argparse
import copy
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import fit_motor_step as captures_lib

COUNTS_PER_REV = 4096
H4_PERIOD = COUNTS_PER_REV // 4
MAX_HARMONIC = 8
MIN_REVOLUTIONS = captures_lib.MIN_RIPPLE_REVOLUTIONS   # 1.5 at steady speed
AMPLITUDE_SPREAD_LIMIT = 0.10       # (max - min) / mean of the H4 amplitudes
PHASE_SPREAD_LIMIT_DEGREES = 10.0


@dataclass
class HarmonicFit:
    name: str
    capture: object
    steady: object
    revolutions: float
    speed: float                    # counts/ms
    coefficients: dict              # n -> (C, S): eps contains C cos(n phi) + S sin(n phi)

    def amplitude(self, n):
        c, s = self.coefficients[n]
        return math.hypot(c, s)

    def phase_degrees(self, n):
        c, s = self.coefficients[n]
        return math.degrees(math.atan2(s, c))


def steady_points(capture, steady):
    positions = captures_lib.positions(capture)
    index = np.arange(steady.start, steady.end + 1)
    good = index[~np.isnan(positions[index])]
    return good * float(capture.period_ms), positions[good]


def fit_harmonics(name, capture):
    """Harmonics 1..MAX_HARMONIC of the steady segment, or None if it has
    none or covers under MIN_REVOLUTIONS (a once-per-turn term can't be told
    from the straight line over part of a turn)."""
    steady = captures_lib.steady_segment(capture, None)
    if steady is None:
        return None
    times, positions = steady_points(capture, steady)
    if abs(positions[-1] - positions[0]) / COUNTS_PER_REV < MIN_REVOLUTIONS:
        return None
    phi = 2 * math.pi * ((capture.raw0 + positions) % COUNTS_PER_REV) / COUNTS_PER_REV
    columns = [np.ones_like(times), times]
    for n in range(1, MAX_HARMONIC + 1):
        columns += [np.cos(n * phi), np.sin(n * phi)]
    solution, *_ = np.linalg.lstsq(np.column_stack(columns), positions, rcond=None)
    coefficients = {n: (float(solution[2 * n]), float(solution[2 * n + 1])) for n in range(1, MAX_HARMONIC + 1)}
    revolutions = abs(positions[-1] - positions[0]) / COUNTS_PER_REV
    return HarmonicFit(name, capture, steady, revolutions, steady.speed, coefficients)


def constants_from(fit):
    """(amplitude_q4, phase_counts) for eps = A sin(2 pi (raw + r0) / 1024):
    A sin(4 phi + psi) = A cos(psi) sin(4 phi) + A sin(psi) cos(4 phi), so
    S4 = A cos(psi), C4 = A sin(psi), r0 = psi / (2 pi) * 1024."""
    c4, s4 = fit.coefficients[4]
    amplitude = math.hypot(c4, s4)
    phase_counts = math.atan2(c4, s4) / (2 * math.pi) * H4_PERIOD
    return round(amplitude * 16), round(phase_counts)


def correction_counts(raw, amplitude_q4, phase_counts):
    """eps(raw) in counts, as the firmware computes it (rounded to whole counts)."""
    raw = np.asarray(raw, dtype=float)
    return np.round(amplitude_q4 / 16.0 * np.sin(2 * math.pi * (raw + phase_counts) / H4_PERIOD))


def corrected_capture(capture, amplitude_q4, phase_counts):
    """A copy of the capture whose positions are the corrected angle, unwrapped."""
    positions = captures_lib.positions(capture)
    absolute = capture.raw0 + positions
    raw = np.mod(absolute, COUNTS_PER_REV)
    corrected_absolute = absolute - correction_counts(raw, amplitude_q4, phase_counts)
    start = capture.raw0 - correction_counts(capture.raw0, amplitude_q4, phase_counts)
    result = copy.deepcopy(capture)
    result.raw0 = int(start) % COUNTS_PER_REV
    relative = corrected_absolute - start
    # Same convention as the firmware's capture: a missing sample stays
    # missing, and the next good delta covers the gap.
    result.deltas = np.full(capture.sample_count, np.nan)
    last = 0.0
    for index in range(capture.usable):
        if not np.isnan(relative[index]):
            result.deltas[index] = relative[index] - last
            last = relative[index]
    return result


@dataclass
class Residual:
    ripple_percent: float           # 16-bin peak-to-peak
    h4_percent: float
    h1_percent: float
    std_20ms: float                 # % of mean
    std_100ms: float


def residual_of(capture, steady):
    ripple = captures_lib.ripple(capture, steady)
    if ripple is None:
        return None
    profile = 100.0 * np.array(ripple.bin_speeds) / ripple.mean_speed
    spectrum = np.fft.rfft(profile - 100.0)
    times, positions = steady_points(capture, steady)

    def window_std(window_ms):
        step = max(1, int(round(window_ms / capture.period_ms)))
        speeds = (positions[step:] - positions[:-step]) / (times[step:] - times[:-step])
        return 100.0 * float(np.std(speeds)) / abs(float(np.mean(speeds)))

    return Residual(ripple.percent, 2 * abs(spectrum[4]) / 16, 2 * abs(spectrum[1]) / 16,
                    window_std(20), window_std(100))


def invariance_warnings(fits):
    if len(fits) < 2:
        return ['only one capture: H4 invariance across speeds not checked']
    amplitudes = [fit.amplitude(4) for fit in fits]
    phases = [fit.phase_degrees(4) for fit in fits]
    reference = phases[0]
    deviations = [((phase - reference + 180.0) % 360.0) - 180.0 for phase in phases]
    warnings = []
    spread = (max(amplitudes) - min(amplitudes)) / float(np.mean(amplitudes))
    if spread > AMPLITUDE_SPREAD_LIMIT:
        warnings.append(f'WARNING: H4 amplitude differs by {spread:.0%} between captures (limit '
                        f'{AMPLITUDE_SPREAD_LIMIT:.0%}): not speed-invariant, so maybe not (only) the sensor, '
                        'or the board moved between captures')
    phase_spread = max(deviations) - min(deviations)
    if phase_spread > PHASE_SPREAD_LIMIT_DEGREES:
        warnings.append(f'WARNING: H4 phase differs by {phase_spread:.0f} degrees between captures (limit '
                        f'{PHASE_SPREAD_LIMIT_DEGREES:.0f}): the board may have moved')
    return warnings


def choose_source(fits, from_name):
    if from_name is None:
        return max(fits, key=lambda fit: fit.revolutions)
    wanted_log, _, wanted_id = from_name.partition(':')
    for fit in fits:
        log, capture_id = fit.name.rsplit(':', 1)
        if (log == wanted_log or Path(log).stem == Path(wanted_log).stem) and (not wanted_id or capture_id == wanted_id):
            return fit
    raise SystemExit(f'--from {from_name}: no capture with a steady segment by that name')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--from', dest='from_name', help='capture to take the constants from (log name[:id])')
    args = parser.parse_args(argv)

    fits = []
    for path in args.logs:
        for capture in captures_lib.parse_captures(path.read_bytes(), path.name):
            fit = fit_harmonics(f'{path.name}:{capture.capture_id}', capture)
            if fit is None:
                print(f'{path.name}:{capture.capture_id}: no steady segment of {MIN_REVOLUTIONS} revolutions or more, '
                      'skipped (at m 10 capture with c 4)')
            else:
                fits.append(fit)
    if not fits:
        print('No capture with a steady segment.')
        return 1

    print('Harmonics of the steady segments (amplitude in counts @ phase in degrees; % = the speed ripple it looks like):')
    for fit in fits:
        parts = []
        for n in (1, 2, 3, 4, 8):
            amplitude = fit.amplitude(n)
            percent = amplitude * n * 2 * math.pi / COUNTS_PER_REV * 100
            parts.append(f'H{n} {amplitude:5.1f}@{fit.phase_degrees(n):4.0f} ({percent:4.1f}%)')
        print(f'  {fit.name:22s} {fit.speed * 1000:6.0f} c/s {fit.revolutions:4.1f} rev  ' + '  '.join(parts))
    for warning in invariance_warnings(fits):
        print('  ' + warning)

    source = choose_source(fits, args.from_name)
    amplitude_q4, phase_counts = constants_from(source)
    print(f'Constants from {source.name} ({source.revolutions:.1f} revolutions):')
    print(f'  #define ANGLE_H4_AMPLITUDE_Q4   {amplitude_q4}    /* {amplitude_q4 / 16:.2f} counts */')
    print(f'  #define ANGLE_H4_PHASE_COUNTS   {phase_counts}')

    print('Predicted with these constants (before -> after): 16-bin ripple p-p, H4, H1, speed std over 20 ms and 100 ms:')
    for fit in fits:
        before = residual_of(fit.capture, fit.steady)
        after_capture = corrected_capture(fit.capture, amplitude_q4, phase_counts)
        after = residual_of(after_capture, captures_lib.steady_segment(after_capture, None))
        if before is None or after is None:
            print(f'  {fit.name:22s} under 1.5 revolutions at steady speed: no residual')
            continue
        print(f'  {fit.name:22s} p-p {before.ripple_percent:4.0f} -> {after.ripple_percent:4.0f} %   '
              f'H4 ±{before.h4_percent:4.1f} -> ±{after.h4_percent:4.1f} %   H1 ±{before.h1_percent:4.1f} -> '
              f'±{after.h1_percent:4.1f} %   20 ms {before.std_20ms:4.1f} -> {after.std_20ms:4.1f} %   '
              f'100 ms {before.std_100ms:4.1f} -> {after.std_100ms:4.1f} %')
    return 0


if __name__ == '__main__':
    sys.exit(main())
