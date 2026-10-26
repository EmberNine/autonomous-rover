#!/usr/bin/env python3
"""TEMPORARY: fit a drive-motor model to the firmware's step captures.

Reads serial logs (written by tools/serial_logger.py or motor_console.py) of a
capture build (SPEED_CAPTURE 1, firmware/Core/Inc/speed_capture.h): one
"caph" header line and then "cap" data lines per capture, mixed in with the
telemetry lines. Every capture in every log given is fitted separately, then
all of them are compared.

Model (per sample, h = the capture's sample period in ms), driven by the
duty the firmware actually applied (so the 2 ticks/ms ramp is part of the
input, not an error):
    f(u)  = sign(u) * max(|u| - u0, 0)          dead zone u0 (ticks)
    w[k]  = a * w[k-1] + (1 - a) * K * f(u[k-1-theta])   while driving,
            a = exp(-h / tau)
    w[k]  = ac * w[k-1], ac = exp(-h / tau_coast)      once the driver is off
    p[k]  = p[k-1] + h * w[k]                    position, counts
w is the wheel speed in counts/ms, K the gain in counts/ms per tick, theta
the dead time. The fit compares POSITIONS (cumulative counts), not per-ms
speeds: with 1-3 counts per ms the speeds are mostly quantisation, while a
position is exact to +-1 count. Samples whose angle read failed ("x") are
missing: they are left out, NOT treated as zero motion (the next good delta
covers the gap, so later positions stay exact).

Per capture it prints the fit (gain, dead zone, time constant, dead time,
coast time constant, residual), the steady speed, and the once-per-revolution
ripple at steady state: the speed in 16 bins of wheel angle (raw angle at the
start + counts, mod 4096), as peak-to-peak % of the mean. Then, over all
captures: the static map speed = K * (duty - u0) from the steady speeds (needs
two or more duties), the ripple comparison between speeds, and PI gains for
the speed controller (lambda tuning, lambda = 2 tau).

Ripple, how to read it (compare m 10 with m 30):
  - about the same % at both speeds: the encoder's angle error (a mis-centred
    magnet) - the wheel isn't really speeding up and slowing down;
  - smaller % at the higher speed: mechanical (binding once per turn), which
    the inertia smooths out more the faster it turns.

Usage (WSL):  python3 tools/fit_motor_step.py /mnt/c/dev/rover-logs/cap_m20_1.log [more logs ...]
"""
import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

COUNTS_PER_REV = 4096
TICKS_PER_PERCENT = 16              # 1600 ticks = 100 % (firmware motor_control.h)
COUNTS_PER_MS_PER_UNIT = 0.05       # 1 speed unit = 1 count per 20 ms = 50 counts/s
CONTROLLER_PERIOD_MS = 10           # the speed controller's planned rate
RIPPLE_BINS = 16
MIN_RIPPLE_REVOLUTIONS = 1.5
RIPPLE_NOISE_FLOOR_PERCENT = 3.0    # estimate: below this the ripple is quantisation noise
SAME_RIPPLE_RATIO = 0.75            # fast/slow ripple at or above this: "about the same"
SMALLER_RIPPLE_RATIO = 0.5          # at or below this: "smaller at the higher speed"
MIN_SPEED_RATIO = 1.5               # captures must differ this much in speed to compare

CAPH = re.compile(rb'^caph id=(\d+) per=(\d+) n=(\d+) t0=(\d+) count0=(-?\d+) raw0=(\d+) vbat=(\d+) '
                  rb'prev=(-?\d+) target=(-?\d+)$')
CAP = re.compile(rb'^cap id=(\d+) i=(\d+) d=([-0-9x,]+) u=([-0-9,]+)$')


@dataclass
class Capture:
    source: str
    capture_id: int
    period_ms: int
    sample_count: int
    t0_ms: int
    count0: int
    raw0: int
    vbat_mv: int
    previous_target: int
    target: int
    deltas: np.ndarray = None          # float, NaN = missing ("x" or never received)
    duties: np.ndarray = None          # float, NaN = never received
    received: np.ndarray = None        # bool per sample
    usable: int = 0                    # samples 0..usable-1 all received
    notes: list = field(default_factory=list)


def parse_captures(data, source='log'):
    """All captures in a log (bytes), in order. A capture's data lines are
    the "cap" lines with its id after its "caph" line."""
    captures = []
    current = None
    for raw_line in data.split(b'\n'):
        line = raw_line.rstrip(b'\r')
        header = CAPH.match(line)
        if header:
            values = [int(group) for group in header.groups()]
            current = Capture(source, values[0], values[1], values[2], values[3], values[4], values[5],
                              values[6], values[7], values[8])
            current.deltas = np.full(current.sample_count, np.nan)
            current.duties = np.full(current.sample_count, np.nan)
            current.received = np.zeros(current.sample_count, dtype=bool)
            captures.append(current)
            continue
        data_line = CAP.match(line)
        if not data_line or current is None or int(data_line.group(1)) != current.capture_id:
            continue
        first = int(data_line.group(2))
        deltas = data_line.group(3).split(b',')
        duties = data_line.group(4).split(b',')
        if len(deltas) != len(duties) or first + len(deltas) > current.sample_count:
            current.notes.append(f'data line i={first} malformed, ignored')
            continue
        for offset, (delta, duty) in enumerate(zip(deltas, duties)):
            index = first + offset
            current.deltas[index] = np.nan if delta == b'x' else float(int(delta))
            current.duties[index] = float(int(duty))
            current.received[index] = True
    for capture in captures:
        missing = np.flatnonzero(~capture.received)
        capture.usable = int(missing[0]) if missing.size else capture.sample_count
        if capture.usable < capture.sample_count:
            capture.notes.append(f'samples from {capture.usable} on were not received (a lost line or a cut log): '
                                 f'fitted on the first {capture.usable} of {capture.sample_count}')
    return captures


def positions(capture):
    """Position in counts at each usable sample, NaN where the sample is
    missing. The delta after a missing sample covers the gap, so a plain sum
    of the good deltas is exact at every good sample."""
    deltas = capture.deltas[:capture.usable]
    good = ~np.isnan(deltas)
    result = np.cumsum(np.where(good, deltas, 0.0))
    result[~good] = np.nan
    return result


def dead_zone(duties, u0):
    """f(u) for an array of duties and an array of dead zones: shape (len(duties), len(u0))."""
    magnitude = np.maximum(np.abs(duties)[:, None] - np.asarray(u0, dtype=float)[None, :], 0.0)
    return np.sign(duties)[:, None] * magnitude


def _drive_end(duties, theta_samples):
    nonzero = np.flatnonzero(duties != 0)
    if nonzero.size == 0:
        return 0
    return min(len(duties), int(nonzero[-1]) + 2 + theta_samples)


def _fit_drive_grid(duties, measured, h, w0, taus, u0s, theta_samples, end):
    """Least squares over a grid of (tau, u0) for one dead time; K in closed
    form. Returns (sse, K, tau, u0) of the best point."""
    taus = np.asarray(taus, dtype=float)
    a = np.exp(-h / taus)[:, None]                      # (nT, 1)
    drive = dead_zone(duties, u0s)                      # (N, nU)
    w1 = np.zeros((taus.size, len(u0s)))                # response to the duty, K = 1
    w0_state = np.full((taus.size, 1), w0)              # response to the initial speed
    p1 = np.zeros_like(w1)
    p0 = np.zeros_like(w0_state)
    s_pd = np.zeros_like(w1)
    s_pp = np.zeros_like(w1)
    s_dd = np.zeros_like(w0_state)
    for k in range(1, end):
        j = k - 1 - theta_samples
        g = drive[j] if j >= 0 else 0.0
        w1 = a * w1 + (1.0 - a) * g
        w0_state = a * w0_state
        p1 = p1 + h * w1
        p0 = p0 + h * w0_state
        y = measured[k]
        if not math.isnan(y):
            d = y - p0
            s_pd += p1 * d
            s_pp += p1 * p1
            s_dd += d * d
    gain = np.where(s_pp > 0, s_pd / np.where(s_pp > 0, s_pp, 1.0), 0.0)
    sse = s_dd - gain * s_pd
    best = np.unravel_index(int(np.argmin(sse)), sse.shape)
    return float(sse[best]), float(gain[best]), float(taus[best[0]]), float(u0s[best[1]])


def simulate(duties, h, gain, u0, tau, theta_samples, tau_coast, w0=0.0):
    """Positions (counts) of the model for one parameter set."""
    a = math.exp(-h / tau)
    ac = math.exp(-h / tau_coast) if tau_coast else 1.0
    drive = dead_zone(duties, [u0])[:, 0]
    w, p = w0, 0.0
    result = np.zeros(len(duties))
    for k in range(1, len(duties)):
        j = k - 1 - theta_samples
        u = duties[j] if j >= 0 else 0.0
        if u != 0 or j < 0:
            w = a * w + (1.0 - a) * gain * (drive[j] if j >= 0 else 0.0)
        else:
            w = ac * w
        p += h * w
        result[k] = p
    return result


@dataclass
class Fit:
    gain: float          # counts/ms per tick
    u0: float            # ticks
    tau_ms: float
    theta_ms: float
    tau_coast_ms: float  # None if the capture has no coast
    rms_counts: float


def fit_capture(capture, max_theta_ms=10):
    """Fits the model to one capture (the usable part). None if too short."""
    h = float(capture.period_ms)
    duties = capture.duties[:capture.usable]
    measured = positions(capture)
    if capture.usable < 50 or not np.any(duties != 0):
        return None
    good_first = [d for d in capture.deltas[1:6] if not math.isnan(d)]
    w0 = (sum(good_first) / len(good_first) / h) if good_first else 0.0

    best = None
    theta_grid = sorted({int(round(theta / h)) for theta in range(0, max_theta_ms + 1)})
    coarse_taus = np.geomspace(5.0, 1000.0, 30)
    coarse_u0s = np.arange(0.0, 201.0, 5.0)
    for theta_samples in theta_grid:
        end = _drive_end(duties, theta_samples)
        sse, gain, tau, u0 = _fit_drive_grid(duties, measured, h, w0, coarse_taus, coarse_u0s, theta_samples, end)
        if best is None or sse < best[0]:
            best = (sse, gain, tau, u0, theta_samples)
    # Refine around the best point, twice, each time on a finer grid (and
    # the neighbouring dead times, which trade off against the dead zone).
    for tau_span, u0_span, u0_step in ((1.4, 8.0, 1.0), (1.06, 2.0, 0.25)):
        _, _, tau, u0, theta_samples = best
        fine_taus = tau * np.geomspace(1.0 / tau_span, tau_span, 21)
        fine_u0s = np.arange(max(0.0, u0 - u0_span), u0 + u0_span + u0_step / 2, u0_step)
        best = None
        for candidate_theta in sorted({max(0, theta_samples - 1), theta_samples, theta_samples + 1}):
            end = _drive_end(duties, candidate_theta)
            sse, gain, tau, u0 = _fit_drive_grid(duties, measured, h, w0, fine_taus, fine_u0s, candidate_theta, end)
            if best is None or sse < best[0]:
                best = (sse, gain, tau, u0, candidate_theta)
    _, gain, tau, u0, theta_samples = best

    # Coast after the driver turned off, if the capture has one.
    end = _drive_end(duties, theta_samples)
    tau_coast = None
    coast_good = ~np.isnan(measured[end:]) if end < capture.usable else np.zeros(0, dtype=bool)
    if coast_good.sum() >= 20:
        best_coast = None
        for grid in (np.geomspace(10.0, 20000.0, 60), None):
            if grid is None:
                grid = best_coast[1] * np.geomspace(1 / 1.15, 1.15, 31)  # refine around the coarse best
            for candidate in grid:
                residual = simulate(duties, h, gain, u0, tau, theta_samples, candidate, w0) - measured
                sse = float(np.nansum(residual[end:] ** 2))
                if best_coast is None or sse < best_coast[0]:
                    best_coast = (sse, candidate)
        tau_coast = best_coast[1]
    model = simulate(duties, h, gain, u0, tau, theta_samples, tau_coast, w0)
    residual = model - measured
    rms = float(np.sqrt(np.nanmean(residual ** 2)))
    return Fit(gain, u0, tau, theta_samples * h, tau_coast, rms)


@dataclass
class Steady:
    duty: float               # ticks
    speed: float              # counts/ms, signed
    start: int                # first sample of the steady segment
    end: int


def steady_segment(capture, fit):
    """The plateau: the longest run of one constant non-zero duty, from when
    the speed had settled (5 time constants, at least 300 ms, after the duty
    got there) to its end. None if under 200 ms of it."""
    h = capture.period_ms
    duties = capture.duties[:capture.usable]
    best_start, best_last, run_start = None, None, 0
    for index in range(1, len(duties) + 1):
        if index == len(duties) or duties[index] != duties[run_start]:
            if duties[run_start] != 0 and (best_start is None or index - run_start > best_last + 1 - best_start):
                best_start, best_last = run_start, index - 1
            run_start = index
    if best_start is None:
        return None
    start, last, final = best_start, best_last, duties[best_start]
    settle_ms = max(5.0 * (fit.tau_ms if fit else 100.0), 300.0)
    begin = start + int(math.ceil(settle_ms / h))
    if (last - begin) * h < 200:
        return None
    measured = positions(capture)
    good = np.flatnonzero(~np.isnan(measured[begin:last + 1])) + begin
    if good.size < 2:
        return None
    first, final_index = int(good[0]), int(good[-1])
    speed = (measured[final_index] - measured[first]) / ((final_index - first) * h)
    return Steady(float(final), float(speed), first, final_index)


@dataclass
class Ripple:
    percent: float            # peak-to-peak / mean, %
    revolutions: float
    mean_speed: float         # counts/ms, magnitude
    bin_speeds: list


def ripple(capture, steady):
    """Speed per 1/16 of a wheel turn over the steady segment, averaged over
    the revolutions in it. None if under MIN_RIPPLE_REVOLUTIONS."""
    measured = positions(capture)
    index = np.arange(steady.start, steady.end + 1)
    good = index[~np.isnan(measured[index])]
    times = good * float(capture.period_ms)
    direction = 1.0 if steady.speed >= 0 else -1.0
    angle = direction * (capture.raw0 + measured[good])      # increasing, in counts
    angle = np.maximum.accumulate(angle)                     # +-1 count jitter must not run backwards
    revolutions = (angle[-1] - angle[0]) / COUNTS_PER_REV
    if revolutions < MIN_RIPPLE_REVOLUTIONS:
        return None
    width = COUNTS_PER_REV // RIPPLE_BINS
    levels = np.arange(math.ceil(angle[0] / width) * width, angle[-1] + 1, width)
    crossing = np.interp(levels, angle, times)
    sums = np.zeros(RIPPLE_BINS)
    counts = np.zeros(RIPPLE_BINS)
    for lower, t_lower, t_upper in zip(levels[:-1], crossing[:-1], crossing[1:]):
        if t_upper <= t_lower:
            continue
        wheel_angle = (direction * lower) % COUNTS_PER_REV if direction > 0 else (-(lower + width)) % COUNTS_PER_REV
        bin_index = int(wheel_angle // width)
        sums[bin_index] += width / (t_upper - t_lower)
        counts[bin_index] += 1
    if np.any(counts == 0):
        return None
    speeds = sums / counts
    mean = float(np.mean(speeds))
    return Ripple(100.0 * (float(np.max(speeds)) - float(np.min(speeds))) / mean, float(revolutions), mean,
                  [float(s) for s in speeds])


def ripple_verdict(results):
    """Compares the ripple of the slowest and fastest captures that have one.
    results: list of Ripple. Returns one line of text."""
    usable = [r for r in results if r is not None]
    if len(usable) < 2:
        return 'Ripple: need captures at two speeds (e.g. m 10 with c 4, and m 30) to tell the cause.'
    slow = min(usable, key=lambda r: r.mean_speed)
    fast = max(usable, key=lambda r: r.mean_speed)
    if fast.mean_speed < MIN_SPEED_RATIO * slow.mean_speed:
        return (f'Ripple: the speeds are too close ({slow.mean_speed * 1000:.0f} and {fast.mean_speed * 1000:.0f} '
                'counts/s) to tell the cause; compare m 10 with m 30.')
    ratio = fast.percent / slow.percent if slow.percent > 0 else float('inf')
    head = (f'Ripple: {slow.percent:.1f} % at {slow.mean_speed * 1000:.0f} counts/s vs {fast.percent:.1f} % at '
            f'{fast.mean_speed * 1000:.0f} counts/s (ratio {ratio:.2f}). ')
    if max(slow.percent, fast.percent) < RIPPLE_NOISE_FLOOR_PERCENT:
        return head + 'Both are at the noise floor: no once-per-revolution ripple to explain.'
    if ratio >= SAME_RIPPLE_RATIO:
        return head + ('About the same % at both speeds: most likely ANGLE ERROR (a mis-centred magnet), '
                       'not real speed changes.')
    if ratio <= SMALLER_RIPPLE_RATIO:
        return head + ('Smaller at the higher speed: most likely MECHANICAL (binding once per turn), '
                       'which the inertia smooths out more at speed.')
    return head + 'In between: inconclusive (could be both); repeat, or compare a wider speed range.'


def static_map(steadies):
    """speed = K * (|duty| - u0) fitted to the steady speeds; None with
    fewer than two distinct duties."""
    points = {}
    for steady in steadies:
        points.setdefault(abs(steady.duty), []).append(abs(steady.speed))
    if len(points) < 2:
        return None
    duties = np.array(sorted(points))
    speeds = np.array([np.mean(points[d]) for d in duties])
    slope, intercept = np.polyfit(duties, speeds, 1)
    return float(slope), float(-intercept / slope)


def pi_gains(gain, tau_ms, theta_ms, lambda_factor=2.0):
    """PI gains for the speed controller (lambda tuning): kp in ticks per
    speed unit, ki in ticks per unit per controller period."""
    gain_units = gain / COUNTS_PER_MS_PER_UNIT          # units per tick
    lam = lambda_factor * tau_ms
    kp = tau_ms / (gain_units * (lam + theta_ms))
    ki = kp * CONTROLLER_PERIOD_MS / tau_ms
    return kp, ki, lam


def describe(capture, fit, steady, ripple_result):
    lines = [f'== {capture.source}: capture {capture.capture_id} ==',
             f'  {capture.sample_count} samples every {capture.period_ms} ms, vbat {capture.vbat_mv / 1000:.2f} V, '
             f'target {capture.previous_target} -> {capture.target} ticks '
             f'({capture.previous_target / TICKS_PER_PERCENT:.0f} -> {capture.target / TICKS_PER_PERCENT:.0f} %)']
    missing = int(np.isnan(capture.deltas[:capture.usable]).sum())
    if missing:
        lines.append(f'  {missing} samples missing (angle read failed): left out of the fit')
    lines += [f'  note: {note}' for note in capture.notes]
    if fit is None:
        lines.append('  too short to fit')
        return lines
    lines.append(f'  time constant {fit.tau_ms:.0f} ms, dead time {fit.theta_ms:.0f} ms')
    lines.append(f'  gain {fit.gain * 1000:.1f} counts/s per tick ({fit.gain * 1000 * TICKS_PER_PERCENT:.0f} per %), '
                 f'dead zone {fit.u0:.0f} ticks ({fit.u0 / TICKS_PER_PERCENT:.1f} %): from one duty these two '
                 'trade off; the static map over several duties (below) is the pair to use')
    lines.append(f'  coast time constant {fit.tau_coast_ms:.0f} ms' if fit.tau_coast_ms
                 else '  no coast in this capture (send m 0 within it to get one)')
    lines.append(f'  fit residual {fit.rms_counts:.1f} counts rms')
    if steady:
        lines.append(f'  steady: {steady.speed * 1000:.0f} counts/s = {steady.speed / COUNTS_PER_MS_PER_UNIT:.1f} units '
                     f'at {steady.duty:.0f} ticks')
    if ripple_result:
        lines.append(f'  once-per-revolution ripple: {ripple_result.percent:.1f} % peak-to-peak over '
                     f'{ripple_result.revolutions:.1f} revolutions')
    elif steady:
        lines.append(f'  ripple: under {MIN_RIPPLE_REVOLUTIONS} revolutions at steady speed '
                     '(at m 10 use c 4: 12 s)')
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('logs', nargs='+', type=Path)
    args = parser.parse_args(argv)

    fits, steadies, ripples = [], [], []
    for path in args.logs:
        captures = parse_captures(path.read_bytes(), path.name)
        if not captures:
            print(f'== {path.name}: no capture in this log ==')
        for capture in captures:
            fit = fit_capture(capture)
            steady = steady_segment(capture, fit) if fit else None
            ripple_result = ripple(capture, steady) if steady else None
            print('\n'.join(describe(capture, fit, steady, ripple_result)))
            if fit:
                fits.append(fit)
            if steady:
                steadies.append(steady)
            ripples.append(ripple_result)

    if not fits:
        print('No capture could be fitted.')
        return 1
    print('== All captures ==')
    mapping = static_map(steadies)
    if mapping:
        gain, u0 = mapping
        print(f'  static map from {len(steadies)} steady speeds: {gain * 1000:.1f} counts/s per tick above '
              f'{u0:.0f} ticks ({u0 / TICKS_PER_PERCENT:.1f} %)')
    else:
        gain = float(np.median([fit.gain for fit in fits]))
        print('  static map: needs steady speeds at two or more duties; using the median fitted gain')
    tau = float(np.median([fit.tau_ms for fit in fits]))
    theta = float(np.median([fit.theta_ms for fit in fits]))
    kp, ki, lam = pi_gains(gain, tau, theta)
    print(f'  median time constant {tau:.0f} ms, dead time {theta:.0f} ms')
    print(f'  PI (lambda = {lam:.0f} ms): kp {kp:.2f} ticks per unit (Q8 {round(kp * 256)}), '
          f'ki {ki:.3f} ticks per unit per {CONTROLLER_PERIOD_MS} ms (Q8 {round(ki * 256)})')
    print('  ' + ripple_verdict(ripples))
    return 0


if __name__ == '__main__':
    sys.exit(main())
