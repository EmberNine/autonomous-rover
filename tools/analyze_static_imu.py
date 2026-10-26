#!/usr/bin/env python3
"""Analyze a static IMU log: gyro bias, drift, noise and Allan deviation.

Input is the 10 Hz telemetry written by the STM32 firmware and captured with
tools/serial_logger.py (format described in firmware/README.md). The output is
a plain-text summary plus two PNG plots, written NEXT TO THE LOG (never into
the repo), and the numbers needed to set the EKF's gyro noise parameters.

Usage:
    python3 tools/analyze_static_imu.py LOG [--output-dir DIR] [--skip-start-seconds N]

References used below:
  [IEEE952]  IEEE Std 952-1997 (R2008), "IEEE Standard Specification Format
             Guide and Test Procedure for Single-Axis Interferometric Fiber
             Optic Gyros", Annex C (Allan variance method).
  [NIST1065] W. J. Riley, "Handbook of Frequency Stability Analysis",
             NIST Special Publication 1065 (2008), sec. 5.2.4 (overlapping
             Allan variance).
  [DS]       TDK InvenSense ICM-42688-P datasheet DS-000347 rev 1.2, as cited
             in firmware/README.md ("IMU fields" table).
"""
import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# --- Scale factors (firmware/README.md "IMU fields"; set in firmware/Core/Src/icm42688.c) ---
GYRO_LSB_PER_DEGREE_PER_SECOND = 65.5   # +/-500 dps full scale [DS 3.1]
ACCEL_LSB_PER_G = 8192.0                # +/-4 g full scale [DS 3.2]
TEMPERATURE_LSB_PER_DEGREE_C = 132.48   # deg C = raw / 132.48 + 25 [DS 14.5-14.6]
TEMPERATURE_OFFSET_DEGREES_C = 25.0
RADIANS_PER_DEGREE = math.pi / 180.0

# --- Telemetry timing (firmware/README.md) ---
REPORT_PERIOD_MS = 100                  # one line per 100 ms window
EXPECTED_SAMPLES_PER_WINDOW = 100       # 1 kHz gyro samples summed per window
BASE_AVERAGING_TIME_SECONDS = REPORT_PERIOD_MS / 1000.0  # tau_0 for the Allan deviation

# Every key the firmware prints, in order (firmware/Core/Src/main.c). A line
# is only "good" if it has exactly these keys, each with an integer value,
# optionally followed by exactly MOTOR_KEYS (firmware with the motor driver
# appends them; older logs don't have them), optionally followed in turn by
# exactly STEERING_KEYS (telemetry v3: reset cause + steering servo), then
# by exactly BATTERY_KEYS (telemetry v4), and then by exactly SPEED_KEYS
# (telemetry v5: closed-loop speed control). Only TELEMETRY_KEYS become
# columns, so old and new logs (or a mix) are analyzed the same way.
TELEMETRY_KEYS = (
    't_ms', 'raw', 'count', 'd100', 'MD', 'ML', 'MH', 'AGC', 'mag_min', 'mag_max',
    'i2c_err', 'overrun', 'resync', 'skipped', 'i2c_us_max', 'loop_us_max',
    'imu_ok', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp',
    'gsum_x', 'gsum_y', 'gsum_z', 'n_imu', 'imu_err', 'spi_us_max',
)
MOTOR_KEYS = ('cmd_pct', 'duty', 'mstate', 'wd_trip', 'cmd_ok', 'cmd_bad', 'cmd_clamp', 'rx_err')
STEERING_KEYS = ('rst', 's_pct', 's_us', 'sstate', 's_ok', 's_wd')
BATTERY_KEYS = ('vbat', 'bstate', 'vdda')
SPEED_KEYS = ('vset', 'vest', 'ctl', 'vok', 'vi', 'stall_n', 'enc_bad', 'mfault')
ACCEPTED_KEY_SEQUENCES = (
    TELEMETRY_KEYS,
    TELEMETRY_KEYS + MOTOR_KEYS,
    TELEMETRY_KEYS + MOTOR_KEYS + STEERING_KEYS,
    TELEMETRY_KEYS + MOTOR_KEYS + STEERING_KEYS + BATTERY_KEYS,
    TELEMETRY_KEYS + MOTOR_KEYS + STEERING_KEYS + BATTERY_KEYS + SPEED_KEYS,
)
BOOT_LINE_PREFIX = 'imu_init='
# Error counters are cumulative since boot (firmware/README.md), so what
# matters is how much they grew during the log, not their absolute value.
CUMULATIVE_ERROR_COUNTERS = ('i2c_err', 'overrun', 'resync', 'skipped', 'imu_err')

AXES = ('x', 'y', 'z')

# Allan-deviation reading rules [IEEE952 Annex C]:
#   angle random walk:  sigma(tau) = N / sqrt(tau)        -> N = sigma at tau = 1 s
#   bias instability:   flat floor sigma = B * sqrt(2 ln2 / pi) = 0.664 * B
#   rate random walk:   sigma(tau) = K * sqrt(tau / 3)    -> K = sigma at tau = 3 s
BIAS_INSTABILITY_FLOOR_FACTOR = math.sqrt(2.0 * math.log(2.0) / math.pi)  # 0.6643
ANGLE_RANDOM_WALK_READ_TAU_SECONDS = 1.0
RATE_RANDOM_WALK_READ_TAU_SECONDS = 3.0
# A log-log slope this close to -1/2 or +1/2 counts as "on that line".
SLOPE_TOLERANCE = 0.15
# The curve bends smoothly through its minimum, so fitting the +1/2 line
# right from the minimum would drag the slope down; start a bit later.
RISING_TAIL_START_FACTOR = 2.0

WARM_UP_SECONDS = 600.0          # "first 10 minutes" for drift and warm-up checks
TEMPERATURE_BLOCK_SECONDS = 60.0  # block-average before regressing bias on temperature

# Colors: the first three slots of the validated categorical palette (dataviz
# skill reference palette, light mode) - one fixed color per axis everywhere.
AXIS_COLORS = {'x': '#2a78d6', 'y': '#eb6834', 'z': '#1baf7a'}
INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
REFERENCE_LINE_COLOR = '#8a8984'
SURFACE_COLOR = '#fcfcfb'


# ----------------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------------

def classify_line(line_text):
    """Return ('telemetry', {key: int}), ('boot', None), ('blank', None) or ('malformed', None)."""
    stripped = line_text.strip()  # also removes the '\r' of '\r\n' line endings
    if not stripped:
        return 'blank', None
    if stripped.startswith(BOOT_LINE_PREFIX):
        return 'boot', None
    fields = {}
    for token in stripped.split(' '):
        key, separator, value = token.partition('=')
        if not separator or key in fields:
            return 'malformed', None
        try:
            fields[key] = int(value)
        except ValueError:
            return 'malformed', None
    # Exact key set and order: a line cut short, or two lines glued together
    # after a dropped '\n', must not sneak through with plausible numbers.
    if tuple(fields) not in ACCEPTED_KEY_SEQUENCES:
        return 'malformed', None
    return 'telemetry', fields


@dataclass
class ParsedLog:
    columns: dict                  # key -> np.ndarray(int64), one entry per good line
    good_line_count: int = 0
    boot_line_count: int = 0
    blank_line_count: int = 0
    malformed_line_count: int = 0
    partial_last_line: bool = False  # file did not end in '\n' (log still being written / cut)
    malformed_examples: list = field(default_factory=list)


def parse_log_text(log_text):
    """Parse the whole log. The last line only counts if it ends in a newline."""
    lines = log_text.split('\n')
    # split() leaves '' after a final '\n'; anything else is an unfinished line.
    last_fragment = lines.pop()
    parsed = ParsedLog(columns={})
    parsed.partial_last_line = last_fragment.strip() != ''

    records = []
    for line_text in lines:
        kind, fields = classify_line(line_text)
        if kind == 'telemetry':
            records.append(fields)
        elif kind == 'boot':
            parsed.boot_line_count += 1
        elif kind == 'blank':
            parsed.blank_line_count += 1
        else:
            parsed.malformed_line_count += 1
            if len(parsed.malformed_examples) < 3:
                parsed.malformed_examples.append(line_text.strip()[:120])
    parsed.good_line_count = len(records)
    parsed.columns = {
        key: np.array([record[key] for record in records], dtype=np.int64) for key in TELEMETRY_KEYS
    }
    return parsed


def find_contiguous_segments(board_time_ms, samples_per_window):
    """Split the good lines into runs of back-to-back windows.

    The Allan deviation assumes evenly spaced, back-to-back averages, so a run
    breaks wherever t_ms does not step by exactly 100 ms (a lost/corrupt line,
    or a board reset), and at any window with no IMU samples (no mean exists).
    Returns (list of (start, stop) index pairs, list of (index, step_ms) gaps).
    """
    if len(board_time_ms) == 0:
        return [], []
    time_steps_ms = np.diff(board_time_ms)
    gaps = [(int(index) + 1, int(time_steps_ms[index]))
            for index in np.flatnonzero(time_steps_ms != REPORT_PERIOD_MS)]
    break_before = np.zeros(len(board_time_ms), dtype=bool)
    break_before[1:] = time_steps_ms != REPORT_PERIOD_MS
    usable = samples_per_window > 0
    segments = []
    segment_start = None
    for index in range(len(board_time_ms)):
        if not usable[index]:
            if segment_start is not None:
                segments.append((segment_start, index))
            segment_start = None
            continue
        if segment_start is None:
            segment_start = index
        elif break_before[index]:
            segments.append((segment_start, index))
            segment_start = index
    if segment_start is not None:
        segments.append((segment_start, len(board_time_ms)))
    return segments, gaps


# ----------------------------------------------------------------------------
# Allan deviation
# ----------------------------------------------------------------------------

def log_spaced_cluster_sizes(sample_count, points_per_decade=10, max_fraction=1.0 / 9.0):
    """Cluster sizes m = 1 ... sample_count/9, roughly evenly spaced on a log axis.

    Stopping at 1/9 of the record keeps at least ~8 independent clusters at the
    longest tau, so the last points aren't pure noise.
    """
    largest_cluster_size = max(1, int(sample_count * max_fraction))
    decades = math.log10(largest_cluster_size)
    candidates = np.logspace(0, decades, max(2, int(decades * points_per_decade) + 1))
    return np.unique(np.round(candidates).astype(int))


def overlapping_allan_deviation(rate_samples, base_interval_seconds, cluster_sizes):
    """Overlapping Allan deviation of a rate signal [NIST1065 sec. 5.2.4; IEEE952 Annex C].

    rate_samples: back-to-back averages of the rate, each over base_interval_seconds.
    For every cluster size m (averaging time tau = m * tau_0):
      1. Integrate the rate into angle: theta_k = tau_0 * (y_0 + ... + y_{k-1}).
      2. The mean rate over any window of length tau is (theta_end - theta_start) / tau,
         so the difference between two ADJACENT tau-windows is
         (theta_{k+2m} - 2 theta_{k+m} + theta_k) / tau.
      3. Allan variance = half the mean square of that difference, averaged over
         every start k ("overlapping": windows may share samples, which uses the
         data better than chopping it into disjoint pieces).
    Returns (taus, allan_deviation, one_sigma_error, cluster_counts).
    """
    rate_samples = np.asarray(rate_samples, dtype=float)
    sample_count = len(rate_samples)
    angle = np.concatenate(([0.0], np.cumsum(rate_samples) * base_interval_seconds))
    taus, deviations, errors, cluster_counts = [], [], [], []
    for cluster_size in cluster_sizes:
        if 2 * cluster_size > sample_count:
            break
        tau = cluster_size * base_interval_seconds
        second_difference = (angle[2 * cluster_size:]
                             - 2.0 * angle[cluster_size:-cluster_size]
                             + angle[:-2 * cluster_size])
        allan_variance = np.mean(second_difference ** 2) / (2.0 * tau ** 2)
        independent_clusters = sample_count // cluster_size
        taus.append(tau)
        deviations.append(math.sqrt(allan_variance))
        # 1-sigma relative uncertainty with K independent clusters,
        # 1 / sqrt(2 (K - 1)) [IEEE952 Annex C, accuracy of the estimate]. It is for
        # the non-overlapping estimator, so it slightly overstates the error
        # of the overlapping one - the conservative side.
        errors.append(math.sqrt(allan_variance) / math.sqrt(2.0 * max(independent_clusters - 1, 1)))
        cluster_counts.append(independent_clusters)
    return np.array(taus), np.array(deviations), np.array(errors), np.array(cluster_counts)


def _log_interpolate(taus, deviations, target_tau):
    return float(np.exp(np.interp(math.log(target_tau), np.log(taus), np.log(deviations))))


def _log_log_slopes(taus, deviations):
    """Slope between neighboring points on the log-log plot."""
    return np.diff(np.log(deviations)) / np.diff(np.log(taus))


@dataclass
class NoiseParameters:
    """Noise terms read off one Allan-deviation curve, in the curve's rate unit."""
    angle_random_walk_read_at_1s: float     # N, rate*sqrt(s): sigma(1 s) * sqrt(1 s)
    angle_random_walk_fit: float            # N fitted on the -1/2 region
    angle_random_walk_fit_tau_range: tuple
    allan_minimum: float
    allan_minimum_tau: float
    bias_instability: float                 # B = minimum / 0.664
    minimum_is_at_end: bool                 # curve still falling -> B is only an upper bound
    rising_tail_slope: float                # log-log slope after the minimum (nan if none)
    rate_random_walk_resolved: bool
    rate_random_walk: float                 # K if resolved, else its upper bound; rate/sqrt(s)
    rate_random_walk_bound_tau: float       # where the bound was taken, or where the +1/2 fit starts


def allan_deviation_upper_confidence_factor(independent_clusters, standard_normal_quantile=1.645):
    """Factor for a one-sided upper confidence limit on a measured Allan deviation.

    The Allan variance estimate follows a chi-squared distribution [NIST1065
    sec. 5.3]: true <= measured * dof / chi2_low(dof). We take dof = K - 1
    for K independent clusters (the non-overlapping count; simple and on
    the cautious side) and the 5 % chi-squared quantile from the
    Wilson-Hilferty approximation (E. B. Wilson & M. M. Hilferty, PNAS 17,
    1931), so no scipy is needed. 1.645 -> 95 % one-sided.
    """
    degrees_of_freedom = max(int(independent_clusters) - 1, 1)
    cube_root_term = (1.0 - 2.0 / (9.0 * degrees_of_freedom)
                      - standard_normal_quantile * math.sqrt(2.0 / (9.0 * degrees_of_freedom)))
    lower_chi_squared = degrees_of_freedom * max(cube_root_term, 1e-3) ** 3
    return math.sqrt(degrees_of_freedom / lower_chi_squared)


def extract_noise_parameters(taus, deviations, independent_cluster_counts):
    """Read N, B and K off an Allan-deviation curve [IEEE952 Annex C]."""
    slopes = _log_log_slopes(taus, deviations)
    minimum_index = int(np.argmin(deviations))

    # N: read directly at tau = 1 s, and also fit sigma*sqrt(tau) = N over the
    # longest run of neighbors (before the minimum) whose slope stays near
    # -1/2. Averaging several points is less noisy than one reading, and not
    # insisting the run starts at tau_0 tolerates a small bump at the
    # shortest taus (e.g. a few-Hz disturbance).
    angle_random_walk_read = _log_interpolate(taus, deviations, ANGLE_RANDOM_WALK_READ_TAU_SECONDS) \
        * math.sqrt(ANGLE_RANDOM_WALK_READ_TAU_SECONDS)
    on_white_line = np.abs(slopes[:minimum_index] + 0.5) <= SLOPE_TOLERANCE
    best_run_start, best_run_length, run_start = 0, 0, None
    for slope_index, on_line in enumerate(np.append(on_white_line, False)):
        if on_line and run_start is None:
            run_start = slope_index
        elif not on_line and run_start is not None:
            if slope_index - run_start > best_run_length:
                best_run_start, best_run_length = run_start, slope_index - run_start
            run_start = None
    if best_run_length >= 1:
        # A run of k slopes spans k + 1 points.
        white_points = slice(best_run_start, best_run_start + best_run_length + 1)
        angle_random_walk_fit = float(np.exp(np.mean(
            np.log(deviations[white_points]) + 0.5 * np.log(taus[white_points]))))
        fit_tau_range = (float(taus[white_points.start]), float(taus[white_points.stop - 1]))
    else:
        angle_random_walk_fit = float('nan')
        fit_tau_range = (float('nan'), float('nan'))

    # B: the floor of the curve divided by 0.664.
    allan_minimum = float(deviations[minimum_index])
    minimum_is_at_end = minimum_index >= len(deviations) - 1

    # K: only if the curve rises after its minimum with a slope near +1/2 over
    # at least half a decade of tau. Then fit a slope-+1/2 line through those
    # points and read it at tau = 3 s.
    tail_start_index = int(np.searchsorted(taus, RISING_TAIL_START_FACTOR * taus[minimum_index]))
    tail = slice(tail_start_index, len(deviations))
    tail_taus, tail_deviations = taus[tail], deviations[tail]
    if len(tail_taus) >= 3:
        rising_tail_slope = float(np.polyfit(np.log(tail_taus), np.log(tail_deviations), 1)[0])
    else:
        rising_tail_slope = float('nan')
    tail_span_decades = math.log10(tail_taus[-1] / tail_taus[0]) if len(tail_taus) >= 2 else 0.0
    rate_random_walk_resolved = (not math.isnan(rising_tail_slope)
                                 and abs(rising_tail_slope - 0.5) <= SLOPE_TOLERANCE
                                 and tail_span_decades >= 0.5)
    if rate_random_walk_resolved:
        rate_random_walk = float(np.exp(np.mean(
            np.log(tail_deviations) - 0.5 * np.log(tail_taus / RATE_RANDOM_WALK_READ_TAU_SECONDS))))
        bound_tau = float(tail_taus[0])
    else:
        # Upper bound: a rate random walk of strength K adds K^2 * tau / 3 to the
        # Allan variance at EVERY tau, so it can never exceed the true curve:
        # K <= sigma_true(tau) * sqrt(3 / tau). Use the longest tau (where a
        # random walk would stand out most) and replace the measured sigma by
        # its one-sided 95% upper confidence limit. Picking the lowest point of
        # the curve instead would be biased low: among ~40 noisy points, some
        # are low by chance (a Monte Carlo check showed a 1-sigma margin there
        # undershoots the true K in ~1/3 of runs).
        upper_deviation = deviations[-1] * allan_deviation_upper_confidence_factor(
            independent_cluster_counts[-1])
        rate_random_walk = float(upper_deviation * math.sqrt(RATE_RANDOM_WALK_READ_TAU_SECONDS / taus[-1]))
        bound_tau = float(taus[-1])

    return NoiseParameters(
        angle_random_walk_read_at_1s=angle_random_walk_read,
        angle_random_walk_fit=angle_random_walk_fit,
        angle_random_walk_fit_tau_range=fit_tau_range,
        allan_minimum=allan_minimum,
        allan_minimum_tau=float(taus[minimum_index]),
        bias_instability=allan_minimum / BIAS_INSTABILITY_FLOOR_FACTOR,
        minimum_is_at_end=minimum_is_at_end,
        rising_tail_slope=rising_tail_slope,
        rate_random_walk_resolved=rate_random_walk_resolved,
        rate_random_walk=rate_random_walk,
        rate_random_walk_bound_tau=bound_tau,
    )


# ----------------------------------------------------------------------------
# Per-segment analysis
# ----------------------------------------------------------------------------

@dataclass
class AxisResult:
    mean_bias: float                       # deg/s
    first_window_bias: float               # deg/s, first WARM_UP_SECONDS
    last_window_bias: float                # deg/s, last WARM_UP_SECONDS
    trend_per_hour: float                  # deg/s per hour, linear fit over the run
    snapshot_sample_std: float             # deg/s, one 1 kHz sample
    window_mean_std: float                 # deg/s, plain std of the 100 ms means (includes drift)
    allan_deviation_at_base_tau: float     # deg/s, ADEV(0.1 s): drift-insensitive noise of a mean
    white_noise_ratio: float               # ADEV(0.1 s) / (sample std / sqrt(n)); 1.0 if white
    taus: np.ndarray
    allan_deviation: np.ndarray            # deg/s
    allan_error: np.ndarray                # deg/s
    noise: NoiseParameters                 # in deg/s units


def gyro_window_mean_degrees_per_second(columns, axis):
    """Mean rate of each 100 ms window: gsum / n_imu, then LSB -> deg/s (firmware/README.md)."""
    return columns[f'gsum_{axis}'] / columns['n_imu'] / GYRO_LSB_PER_DEGREE_PER_SECOND


def analyze_axis(columns, axis):
    window_mean_rate_degrees_per_second = gyro_window_mean_degrees_per_second(columns, axis)
    snapshot_rate_degrees_per_second = columns[f'g{axis}'] / GYRO_LSB_PER_DEGREE_PER_SECOND
    elapsed_seconds = np.arange(len(window_mean_rate_degrees_per_second)) * BASE_AVERAGING_TIME_SECONDS
    warm_up_windows = int(WARM_UP_SECONDS / BASE_AVERAGING_TIME_SECONDS)

    trend_per_second = np.polyfit(elapsed_seconds, window_mean_rate_degrees_per_second, 1)[0]

    taus, allan_deviation, allan_error, independent_cluster_counts = overlapping_allan_deviation(
        window_mean_rate_degrees_per_second, BASE_AVERAGING_TIME_SECONDS,
        log_spaced_cluster_sizes(len(window_mean_rate_degrees_per_second)))

    snapshot_sample_std = float(np.std(snapshot_rate_degrees_per_second, ddof=1))
    mean_samples_per_window = float(np.mean(columns['n_imu']))
    # If the 1 kHz samples were independent, a 100-sample mean would have
    # std = sample std / sqrt(100). ADEV(tau_0) estimates the std of one mean
    # but ignores slow drift (it only compares neighbors), so it is the fair
    # thing to compare with.
    expected_white_mean_std = snapshot_sample_std / math.sqrt(mean_samples_per_window)

    return AxisResult(
        mean_bias=float(np.mean(window_mean_rate_degrees_per_second)),
        first_window_bias=float(np.mean(window_mean_rate_degrees_per_second[:warm_up_windows])),
        last_window_bias=float(np.mean(window_mean_rate_degrees_per_second[-warm_up_windows:])),
        trend_per_hour=float(trend_per_second * 3600.0),
        snapshot_sample_std=snapshot_sample_std,
        window_mean_std=float(np.std(window_mean_rate_degrees_per_second, ddof=1)),
        allan_deviation_at_base_tau=float(allan_deviation[0]),
        white_noise_ratio=float(allan_deviation[0] / expected_white_mean_std),
        taus=taus,
        allan_deviation=allan_deviation,
        allan_error=allan_error,
        noise=extract_noise_parameters(taus, allan_deviation, independent_cluster_counts),
    )


def temperature_degrees_c(raw_temperature):
    return raw_temperature / TEMPERATURE_LSB_PER_DEGREE_C + TEMPERATURE_OFFSET_DEGREES_C


def block_means(values, block_length):
    """Average non-overlapping blocks of block_length samples (drops the remainder)."""
    block_count = len(values) // block_length
    return values[:block_count * block_length].reshape(block_count, block_length).mean(axis=1)


@dataclass
class TemperatureResult:
    minimum_block_mean_c: float
    maximum_block_mean_c: float
    snapshot_min_c: float
    snapshot_max_c: float
    z_bias_slope_degrees_per_second_per_c: float
    z_bias_slope_standard_error: float
    correlation: float


def analyze_temperature(columns):
    temperature_c = temperature_degrees_c(columns['temp'].astype(float))
    z_rate = gyro_window_mean_degrees_per_second(columns, 'z')
    # Each line's temperature is ONE noisy snapshot (+/-0.1 C). Regressing on
    # those directly would bias the slope toward zero (noise in x "dilutes" the
    # fit), so average both into 60 s blocks first.
    block_length = int(TEMPERATURE_BLOCK_SECONDS / BASE_AVERAGING_TIME_SECONDS)
    block_temperature = block_means(temperature_c, block_length)
    block_z_rate = block_means(z_rate, block_length)
    if len(block_temperature) >= 3 and np.ptp(block_temperature) > 0:
        coefficients, covariance = np.polyfit(block_temperature, block_z_rate, 1, cov=True)
        slope, slope_error = float(coefficients[0]), float(math.sqrt(covariance[0, 0]))
        correlation = float(np.corrcoef(block_temperature, block_z_rate)[0, 1])
    else:
        slope = slope_error = correlation = float('nan')
    return TemperatureResult(
        minimum_block_mean_c=float(np.min(block_temperature)) if len(block_temperature) else float('nan'),
        maximum_block_mean_c=float(np.max(block_temperature)) if len(block_temperature) else float('nan'),
        snapshot_min_c=float(np.min(temperature_c)),
        snapshot_max_c=float(np.max(temperature_c)),
        z_bias_slope_degrees_per_second_per_c=slope,
        z_bias_slope_standard_error=slope_error,
        correlation=correlation,
    )


def analyze_accelerometer(columns):
    """Mean and per-sample std per axis in g, and tilt from the gravity direction."""
    accel_g = {axis: columns[f'a{axis}'] / ACCEL_LSB_PER_G for axis in AXES}
    means = {axis: float(np.mean(values)) for axis, values in accel_g.items()}
    stds = {axis: float(np.std(values, ddof=1)) for axis, values in accel_g.items()}
    # At rest the accelerometer reads the reaction to gravity: +1 g "up".
    # REP 103 (X forward, Y left, Z up): roll about X is positive when the left
    # side is up (ay > 0); pitch about Y is positive nose-down (ax < 0).
    roll_degrees = math.degrees(math.atan2(means['y'], means['z']))
    pitch_degrees = math.degrees(math.atan2(-means['x'], math.hypot(means['y'], means['z'])))
    magnitude = math.sqrt(sum(value ** 2 for value in means.values()))
    tilt_degrees = math.degrees(math.acos(means['z'] / magnitude))
    return means, stds, magnitude, roll_degrees, pitch_degrees, tilt_degrees


def slice_columns(columns, start, stop):
    return {key: values[start:stop] for key, values in columns.items()}


# ----------------------------------------------------------------------------
# Plots
# ----------------------------------------------------------------------------

def _style_axes(axes):
    axes.set_facecolor(SURFACE_COLOR)
    for spine in ('top', 'right'):
        axes.spines[spine].set_visible(False)
    for spine in ('left', 'bottom'):
        axes.spines[spine].set_color(INK_SECONDARY)
    axes.tick_params(colors=INK_SECONDARY, labelcolor=INK_SECONDARY)
    axes.xaxis.label.set_color(INK_PRIMARY)
    axes.yaxis.label.set_color(INK_PRIMARY)


def plot_allan_deviation(axis_results, output_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(9, 6.5), facecolor=SURFACE_COLOR)
    _style_axes(axes)
    axes.grid(True, which='major', color='#e4e3df', linewidth=0.8)
    axes.grid(True, which='minor', color='#f0efec', linewidth=0.5)

    for axis, result in axis_results.items():
        color = AXIS_COLORS[axis]
        axes.fill_between(result.taus, result.allan_deviation - result.allan_error,
                          result.allan_deviation + result.allan_error, color=color, alpha=0.15, linewidth=0)
        axes.plot(result.taus, result.allan_deviation, color=color, linewidth=2, label=f'{axis.upper()} gyro')
        # Direct label at the right end of each curve.
        axes.annotate(axis.upper(), (result.taus[-1], result.allan_deviation[-1]), xytext=(6, 0),
                      textcoords='offset points', color=INK_PRIMARY, va='center', fontsize=10)

    # Reference slopes and N/B/K points for Z (the axis the EKF uses).
    z_result = axis_results['z']
    noise = z_result.noise
    tau_line = np.array([z_result.taus[0], z_result.taus[-1]])
    axes.plot(tau_line, noise.angle_random_walk_read_at_1s / np.sqrt(tau_line), linestyle='--',
              color=REFERENCE_LINE_COLOR, linewidth=1.2, label='slope -1/2 through Z at 1 s (white noise)')
    axes.plot(tau_line, noise.rate_random_walk * np.sqrt(tau_line / RATE_RANDOM_WALK_READ_TAU_SECONDS),
              linestyle=':', color=REFERENCE_LINE_COLOR, linewidth=1.5,
              label='slope +1/2: Z rate random walk ' + ('fit' if noise.rate_random_walk_resolved
                                                         else 'upper bound'))
    axes.axhline(noise.allan_minimum, color=REFERENCE_LINE_COLOR, linewidth=0.8, linestyle='-.')

    marker_style = dict(markersize=9, markerfacecolor=SURFACE_COLOR, markeredgewidth=2,
                        markeredgecolor=INK_PRIMARY, linestyle='none')
    axes.plot([1.0], [noise.angle_random_walk_read_at_1s], marker='o', **marker_style)
    axes.annotate(f'N = {noise.angle_random_walk_read_at_1s:.2e} deg/s/sqrt(Hz)\n(Z, tau = 1 s)',
                  (1.0, noise.angle_random_walk_read_at_1s), xytext=(10, 10), textcoords='offset points',
                  color=INK_PRIMARY, fontsize=9)
    axes.plot([noise.allan_minimum_tau], [noise.allan_minimum], marker='s', **marker_style)
    axes.annotate(f'min {noise.allan_minimum:.2e} deg/s at {noise.allan_minimum_tau:.0f} s\n'
                  f'B = min / 0.664 = {noise.bias_instability:.2e} deg/s',
                  (noise.allan_minimum_tau, noise.allan_minimum), xytext=(0.60, 0.10),
                  textcoords='axes fraction', color=INK_PRIMARY, fontsize=9,
                  arrowprops=dict(arrowstyle='-', color=INK_SECONDARY, linewidth=0.8))
    k_label_value = noise.rate_random_walk
    axes.plot([RATE_RANDOM_WALK_READ_TAU_SECONDS], [k_label_value], marker='^', **marker_style)
    axes.annotate(('K = ' if noise.rate_random_walk_resolved else 'K < ')
                  + f'{k_label_value:.2e} deg/s/sqrt(s)\n(Z, read at tau = 3 s'
                  + (')' if noise.rate_random_walk_resolved else '; 95% upper bound)'),
                  (RATE_RANDOM_WALK_READ_TAU_SECONDS, k_label_value), xytext=(10, -22),
                  textcoords='offset points', color=INK_PRIMARY, fontsize=9)

    axes.set_xscale('log')
    axes.set_yscale('log')
    axes.set_xlabel('averaging time tau (s)')
    axes.set_ylabel('Allan deviation (deg/s)')
    axes.set_title('Gyro Allan deviation, static log (band = 1-sigma estimate error)',
                   color=INK_PRIMARY, loc='left')
    legend = axes.legend(loc='lower left', frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(INK_PRIMARY)
    figure.tight_layout()
    figure.savefig(output_path, dpi=130, facecolor=SURFACE_COLOR)
    plt.close(figure)


def plot_z_rate_and_temperature(columns, start_board_time_ms, output_path):
    """Z window-mean rate and temperature over time.

    Two stacked panels sharing the time axis rather than one chart with two
    y-axes: twin y-scales invite reading meaning into where the lines cross,
    which depends only on how each axis was scaled.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    elapsed_minutes = (columns['t_ms'] - start_board_time_ms) / 60000.0
    z_rate = gyro_window_mean_degrees_per_second(columns, 'z')
    temperature_c = temperature_degrees_c(columns['temp'].astype(float))
    smoothing_length = int(TEMPERATURE_BLOCK_SECONDS / BASE_AVERAGING_TIME_SECONDS)
    smoothing_kernel = np.ones(smoothing_length) / smoothing_length
    valid = slice(smoothing_length - 1, None)  # 'valid' convolution drops the first block-1 points

    figure, (rate_axes, temperature_axes) = plt.subplots(
        2, 1, sharex=True, figsize=(10, 6.5), facecolor=SURFACE_COLOR, gridspec_kw={'height_ratios': [3, 2]})
    for axes in (rate_axes, temperature_axes):
        _style_axes(axes)
        axes.grid(True, color='#e4e3df', linewidth=0.8)

    rate_axes.plot(elapsed_minutes, z_rate, color=AXIS_COLORS['z'], alpha=0.25, linewidth=0.5,
                   label='100 ms window mean')
    rate_axes.plot(elapsed_minutes[valid], np.convolve(z_rate, smoothing_kernel, mode='valid'),
                   color=INK_PRIMARY, linewidth=2, label='60 s moving average')
    rate_axes.set_ylabel('Z rate (deg/s)')
    rate_axes.set_title('Z gyro (yaw) while static, and IMU temperature', color=INK_PRIMARY, loc='left')
    legend = rate_axes.legend(loc='upper right', frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(INK_PRIMARY)

    temperature_axes.plot(elapsed_minutes, temperature_c, color=INK_SECONDARY, alpha=0.25, linewidth=0.5,
                          label='snapshot')
    temperature_axes.plot(elapsed_minutes[valid],
                          np.convolve(temperature_c, smoothing_kernel, mode='valid'),
                          color=INK_PRIMARY, linewidth=2, label='60 s moving average')
    temperature_axes.set_ylabel('IMU temperature (deg C)')
    temperature_axes.set_xlabel('minutes since start of analyzed segment')
    legend = temperature_axes.legend(loc='upper right', frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(INK_PRIMARY)

    figure.tight_layout()
    figure.savefig(output_path, dpi=130, facecolor=SURFACE_COLOR)
    plt.close(figure)


# ----------------------------------------------------------------------------
# Report
# ----------------------------------------------------------------------------

def _to_rad(degrees_value):
    return degrees_value * RADIANS_PER_DEGREE


def format_axis_table(axis_results):
    """Rows: quantity; columns: X, Y, Z in deg/s-based units, then Z in rad/s-based units."""
    lines = []
    header = f"{'quantity':<46}{'unit':<16}{'X':>12}{'Y':>12}{'Z':>12}   {'Z (rad units)':>14}  rad unit"
    lines.append(header)
    lines.append('-' * len(header))

    def row(label, unit, rad_unit, getter, scale_to_rad=RADIANS_PER_DEGREE):
        values = [getter(axis_results[axis]) for axis in AXES]
        z_rad = values[2] * scale_to_rad if rad_unit else None
        text = f'{label:<46}{unit:<16}' + ''.join(f'{value:>12.4g}' for value in values)
        if rad_unit:
            text += f'   {z_rad:>14.4g}  {rad_unit}'
        lines.append(text)

    row('mean bias (whole segment)', 'deg/s', 'rad/s', lambda r: r.mean_bias)
    row('bias, first 10 min', 'deg/s', 'rad/s', lambda r: r.first_window_bias)
    row('bias, last 10 min', 'deg/s', 'rad/s', lambda r: r.last_window_bias)
    row('bias change last - first', 'deg/s', 'rad/s', lambda r: r.last_window_bias - r.first_window_bias)
    row('linear trend', 'deg/s per h', 'rad/s per h', lambda r: r.trend_per_hour)
    row('std of one 1 kHz sample (gx/gy/gz)', 'deg/s', 'rad/s', lambda r: r.snapshot_sample_std)
    row('std of 100 ms means (incl. drift)', 'deg/s', 'rad/s', lambda r: r.window_mean_std)
    row('ADEV(0.1 s) = noise of one 100 ms mean', 'deg/s', 'rad/s', lambda r: r.allan_deviation_at_base_tau)
    row('ratio ADEV(0.1 s) / (sample std/sqrt(n))', '1 = white', None, lambda r: r.white_noise_ratio)
    row('N, ARW read at tau = 1 s', 'deg/s/sqrt(Hz)', 'rad/s/sqrt(Hz)',
        lambda r: r.noise.angle_random_walk_read_at_1s)
    row('N, ARW fitted on -1/2 region', 'deg/s/sqrt(Hz)', 'rad/s/sqrt(Hz)', lambda r: r.noise.angle_random_walk_fit)
    row('N, same in deg/sqrt(h)  (x 60)', 'deg/sqrt(h)', None, lambda r: r.noise.angle_random_walk_read_at_1s * 60)
    row('Allan minimum', 'deg/s', 'rad/s', lambda r: r.noise.allan_minimum)
    row('tau at Allan minimum', 's', None, lambda r: r.noise.allan_minimum_tau)
    row('B, bias instability = min / 0.664', 'deg/s', 'rad/s', lambda r: r.noise.bias_instability)
    row('B in deg/h  (x 3600)', 'deg/h', None, lambda r: r.noise.bias_instability * 3600)
    row('log-log slope after the minimum', '+0.5 = RRW', None, lambda r: r.noise.rising_tail_slope)
    row('K, rate random walk: fit, or 95% upper bound', 'deg/s/sqrt(s)', 'rad/s/sqrt(s)',
        lambda r: r.noise.rate_random_walk)
    row('  ... tau where K bound / fit start taken', 's', None, lambda r: r.noise.rate_random_walk_bound_tau)
    lines.append(f"{'K resolved (+1/2 region visible)?':<46}{'':<16}"
                 + ''.join(f"{('yes' if axis_results[axis].noise.rate_random_walk_resolved else 'no, bound'):>12}"
                           for axis in AXES))
    lines.append(f"{'N fit tau range':<46}{'s':<16}"
                 + ''.join(f"{'%.1f-%.1f' % axis_results[axis].noise.angle_random_walk_fit_tau_range:>12}"
                           for axis in AXES))
    return lines


def build_report(log_path, parsed, segments, gaps, segment_index, skip_start_seconds,
                 analyzed_columns, axis_results, temperature_result, accel_summary,
                 warm_up_comparison, plot_paths):
    lines = []
    add = lines.append
    add(f'Static IMU analysis of {log_path}')
    add('=' * 78)
    add('')
    add('PARSING')
    add(f'  good telemetry lines:        {parsed.good_line_count}')
    add(f'  boot lines (imu_init=...):   {parsed.boot_line_count}')
    add(f'  malformed lines skipped:     {parsed.malformed_line_count}')
    for example in parsed.malformed_examples:
        add(f'      e.g. {example!r}')
    add(f'  blank lines:                 {parsed.blank_line_count}')
    add(f'  unfinished last line:        {"yes (skipped)" if parsed.partial_last_line else "no"}')
    add(f'  t_ms gaps (step != 100 ms):  {len(gaps)}')
    for gap_index, step in gaps[:10]:
        add(f'      before line #{gap_index}: step {step} ms')
    columns = parsed.columns
    off_count_windows = int(np.sum(columns['n_imu'] != EXPECTED_SAMPLES_PER_WINDOW))
    add(f'  windows with n_imu != 100:   {off_count_windows}'
        + (f'  (n_imu range {columns["n_imu"].min()}..{columns["n_imu"].max()})' if off_count_windows else ''))
    imu_ok_zero = int(np.sum(columns['imu_ok'] != 1))
    add(f'  lines with imu_ok != 1:      {imu_ok_zero}')
    add('  error counters (cumulative since boot; increase during the log):')
    for counter in CUMULATIVE_ERROR_COUNTERS:
        values = columns[counter]
        add(f'      {counter:<8} first {values[0]:>6}  last {values[-1]:>6}  increase {values[-1] - values[0]:>6}')
    add(f'  contiguous segments:         {len(segments)}  (lengths in s: '
        + ', '.join(f'{(stop - start) * BASE_AVERAGING_TIME_SECONDS:.1f}' for start, stop in segments[:10])
        + (' ...' if len(segments) > 10 else '') + ')')
    start, stop = segments[segment_index]
    add(f'  ANALYZED: longest segment #{segment_index}, t_ms {columns["t_ms"][start]}..{columns["t_ms"][stop - 1]}'
        f' ({(stop - start) * BASE_AVERAGING_TIME_SECONDS / 60:.1f} min)'
        + (f', first {skip_start_seconds:.0f} s skipped' if skip_start_seconds else '')
        + f' -> {len(analyzed_columns["t_ms"])} windows'
        f' ({len(analyzed_columns["t_ms"]) * BASE_AVERAGING_TIME_SECONDS / 60:.1f} min)')
    add('  Note: the IMU (1 kHz, own clock) and the firmware loop (1 kHz, MCU clock) are not')
    add('  synchronized, so a window can hold a duplicated or a skipped sample. That changes a')
    add('  100-sample mean by at most ~1/100 of one sample\'s noise - negligible here.')
    add('')

    add('GYRO (scale 65.5 LSB per deg/s, firmware/README.md; rad = deg * pi/180)')
    add('  N: angle random walk, B: bias instability, K: rate random walk [IEEE Std 952-1997 Annex C].')
    add('  deg/s/sqrt(Hz) is the same thing as deg/sqrt(s).')
    lines.extend('  ' + line for line in format_axis_table(axis_results))
    add('')

    add('TEMPERATURE (deg C = raw / 132.48 + 25, firmware/README.md)')
    add(f'  range of 60 s means:  {temperature_result.minimum_block_mean_c:.2f} .. '
        f'{temperature_result.maximum_block_mean_c:.2f} C'
        f'  (span {temperature_result.maximum_block_mean_c - temperature_result.minimum_block_mean_c:.2f} C)')
    add(f'  range of snapshots:   {temperature_result.snapshot_min_c:.2f} .. {temperature_result.snapshot_max_c:.2f} C')
    add(f'  Z bias vs temperature (60 s blocks): {temperature_result.z_bias_slope_degrees_per_second_per_c:.4g}'
        f' +/- {temperature_result.z_bias_slope_standard_error:.2g} deg/s per C'
        f'  (= {_to_rad(temperature_result.z_bias_slope_degrees_per_second_per_c):.3g} rad/s per C),'
        f' correlation r = {temperature_result.correlation:.2f}')
    add('  (A slope from a small temperature span is weak evidence; see the span above.)')
    add('')

    means, stds, magnitude, roll_degrees, pitch_degrees, tilt_degrees = accel_summary
    add('ACCELEROMETER (scale 8192 LSB per g, firmware/README.md; not used by the EKF yet)')
    for axis in AXES:
        add(f'  {axis.upper()}: mean {means[axis]:+.4f} g   std of one sample {stds[axis] * 1000:.2f} mg')
    add(f'  |mean| = {magnitude:.4f} g   roll {roll_degrees:+.2f} deg (left side up +), '
        f'pitch {pitch_degrees:+.2f} deg (nose down +), tilt from vertical {tilt_degrees:.2f} deg')
    add('  (Accel offsets of a few mg are normal, so tilt here is uncertain by roughly 0.5 deg.)')
    add('')

    add('WARM-UP CHECK (Z axis, same segment: all of it vs. first 600 s removed)')
    if warm_up_comparison is None:
        add('  segment too short to compare')
    else:
        with_warm_up, without_warm_up = warm_up_comparison
        for label, getter, unit in (
                ('mean bias', lambda r: r.mean_bias, 'deg/s'),
                ('ADEV(0.1 s)', lambda r: r.allan_deviation_at_base_tau, 'deg/s'),
                ('linear trend', lambda r: r.trend_per_hour, 'deg/s per h'),
                ('N at 1 s', lambda r: r.noise.angle_random_walk_read_at_1s, 'deg/s/sqrt(Hz)'),
                ('Allan minimum', lambda r: r.noise.allan_minimum, 'deg/s'),
                ('tau at minimum', lambda r: r.noise.allan_minimum_tau, 's'),
                ('B', lambda r: r.noise.bias_instability, 'deg/s'),
                ('K (fit or bound)', lambda r: r.noise.rate_random_walk, 'deg/s/sqrt(s)'),
                ('slope after minimum', lambda r: r.noise.rising_tail_slope, '')):
            add(f'  {label:<22}{getter(with_warm_up):>12.4g}{getter(without_warm_up):>12.4g}   {unit}')
        add(f"  {'':<22}{'all':>12}{'skip 600 s':>12}")
    add('')
    add('PLOTS')
    for plot_path in plot_paths:
        add(f'  {plot_path}')
    return lines


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def resolve_output_directory(log_path, requested_output_directory):
    output_directory = (Path(requested_output_directory) if requested_output_directory
                        else log_path.parent / f'{log_path.stem}_analysis').resolve()
    # Logs, plots and results never belong in the repo.
    if output_directory == REPOSITORY_ROOT or REPOSITORY_ROOT in output_directory.parents:
        raise SystemExit(f'Refusing to write results inside the repository: {output_directory}\n'
                         'Pass --output-dir somewhere outside it (default is next to the log).')
    return output_directory


def main(argv=None):
    argument_parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    argument_parser.add_argument('log_path', type=Path)
    argument_parser.add_argument('--output-dir', default=None,
                                 help='where to write the summary and plots (default: next to the log)')
    argument_parser.add_argument('--skip-start-seconds', type=float, default=0.0,
                                 help='drop this much from the start of the analyzed segment (warm-up)')
    arguments = argument_parser.parse_args(argv)

    log_path = arguments.log_path.resolve()
    output_directory = resolve_output_directory(log_path, arguments.output_dir)
    # errors='replace': a corrupted byte turns into U+FFFD, which then fails
    # int() and the line is counted as malformed instead of crashing.
    parsed = parse_log_text(log_path.read_bytes().decode('ascii', errors='replace'))
    if parsed.good_line_count == 0:
        raise SystemExit('No good telemetry lines found.')

    segments, gaps = find_contiguous_segments(parsed.columns['t_ms'], parsed.columns['n_imu'])
    segment_index = int(np.argmax([stop - start for start, stop in segments]))
    segment_start, segment_stop = segments[segment_index]
    skip_windows = int(round(arguments.skip_start_seconds / BASE_AVERAGING_TIME_SECONDS))
    analyzed_columns = slice_columns(parsed.columns, segment_start + skip_windows, segment_stop)
    analyzed_window_count = len(analyzed_columns['t_ms'])
    minimum_windows = int(2 * WARM_UP_SECONDS / BASE_AVERAGING_TIME_SECONDS)
    if analyzed_window_count < minimum_windows:
        raise SystemExit(f'Analyzed segment has only {analyzed_window_count} windows; need at least '
                         f'{minimum_windows} (2 x {WARM_UP_SECONDS:.0f} s) for the first/last-10-min drift.')

    axis_results = {axis: analyze_axis(analyzed_columns, axis) for axis in AXES}
    temperature_result = analyze_temperature(analyzed_columns)
    accel_summary = analyze_accelerometer(analyzed_columns)

    # Warm-up check always compares the WHOLE segment with the segment minus
    # its first 600 s, independent of --skip-start-seconds.
    whole_segment_columns = slice_columns(parsed.columns, segment_start, segment_stop)
    warm_up_windows = int(WARM_UP_SECONDS / BASE_AVERAGING_TIME_SECONDS)
    if segment_stop - segment_start >= warm_up_windows + minimum_windows:
        warm_up_comparison = (
            analyze_axis(whole_segment_columns, 'z'),
            analyze_axis(slice_columns(whole_segment_columns, warm_up_windows, None), 'z'),
        )
    else:
        warm_up_comparison = None

    output_directory.mkdir(parents=True, exist_ok=True)
    suffix = f'_skip{int(arguments.skip_start_seconds)}s' if arguments.skip_start_seconds else ''
    allan_plot_path = output_directory / f'{log_path.stem}{suffix}_allan_deviation.png'
    rate_plot_path = output_directory / f'{log_path.stem}{suffix}_z_rate_temperature.png'
    summary_path = output_directory / f'{log_path.stem}{suffix}_summary.txt'
    plot_allan_deviation(axis_results, allan_plot_path)
    plot_z_rate_and_temperature(analyzed_columns, analyzed_columns['t_ms'][0], rate_plot_path)

    report_lines = build_report(log_path, parsed, segments, gaps, segment_index, arguments.skip_start_seconds,
                                analyzed_columns, axis_results, temperature_result, accel_summary,
                                warm_up_comparison, [allan_plot_path, rate_plot_path, summary_path])
    report_text = '\n'.join(report_lines) + '\n'
    summary_path.write_text(report_text)
    print(report_text, end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
