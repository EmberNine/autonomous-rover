"""Tests for analyze_static_imu.py. Run from the repo root: python3 -m pytest tools/

Statistical tests use a fixed seed, so they are deterministic. The tolerances
are not tuned to that seed; they come from the estimator's known scatter:
  - The 1-sigma relative error of an Allan deviation with K independent
    clusters is 1/sqrt(2(K-1)) [IEEE Std 952-1997 Annex C]. A Monte Carlo run
    (200 seeds, every tau of a 100k-sample record) found the real scatter of
    the overlapping estimator to be 0.74-1.19 x that formula, and the worst
    single deviation 3.9 x formula. Allowing 5 x formula per point is
    therefore >= 4.2 real standard deviations.
  - The log-log slope fitted over all taus scattered with std 0.014 (white)
    and 0.031 (random walk) over 200 seeds; the tolerances below are 7 and 5
    of those standard deviations.
So any seed would pass; a failure means the code is wrong, not unlucky.
"""
import math

import numpy as np
import pytest

import analyze_static_imu as analysis

GOOD_LINE = (
    't_ms=273200 raw=2030 count=-48 d100=0 MD=1 ML=1 MH=0 AGC=128 mag_min=1532 mag_max=1535 '
    'i2c_err=0 overrun=0 resync=0 skipped=0 i2c_us_max=523 loop_us_max=901 imu_ok=1 ax=-45 ay=237 '
    'az=8224 gx=-9 gy=-33 gz=-5 temp=-364 gsum_x=-1100 gsum_y=-3258 gsum_z=-515 n_imu=100 imu_err=0 '
    'spi_us_max=186'
)

BASE_INTERVAL_SECONDS = 0.1
SAMPLE_COUNT = 100_000
SEED = 20261004


def make_line(board_time_ms, samples_per_window=100):
    return (GOOD_LINE.replace('t_ms=273200', f't_ms={board_time_ms}')
            .replace('n_imu=100', f'n_imu={samples_per_window}'))


# --- Parser -----------------------------------------------------------------

def test_good_line_with_crlf_is_parsed_into_integers():
    kind, fields = analysis.classify_line(GOOD_LINE + '\r')  # '\n' already split off
    assert kind == 'telemetry'
    assert fields['t_ms'] == 273200
    assert fields['gz'] == -5
    assert fields['gsum_z'] == -515
    assert fields['n_imu'] == 100
    assert tuple(fields) == analysis.TELEMETRY_KEYS


MOTOR_PART = 'cmd_pct=20 duty=320 mstate=1 wd_trip=0 cmd_ok=57 cmd_bad=1 cmd_clamp=0 rx_err=0'


def test_line_with_motor_fields_is_parsed():
    kind, fields = analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + '\r')
    assert kind == 'telemetry'
    assert tuple(fields) == analysis.TELEMETRY_KEYS + analysis.MOTOR_KEYS
    assert fields['gz'] == -5
    assert fields['duty'] == 320


def test_motor_fields_are_all_or_nothing():
    # Cut inside the motor part, motor fields in the wrong order, or an extra field.
    assert analysis.classify_line(GOOD_LINE + ' cmd_pct=20 duty=320')[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART.replace('cmd_pct=20 duty=320', 'duty=320 cmd_pct=20'))[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + ' extra=1')[0] == 'malformed'


STEERING_PART = 'rst=36 s_pct=-40 s_us=1380 sstate=1 s_ok=12 s_wd=0'


def test_line_with_steering_fields_is_parsed():
    kind, fields = analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART + '\r')
    assert kind == 'telemetry'
    assert tuple(fields) == analysis.TELEMETRY_KEYS + analysis.MOTOR_KEYS + analysis.STEERING_KEYS
    assert fields['gz'] == -5
    assert fields['rst'] == 36
    assert fields['s_pct'] == -40


def test_steering_fields_are_all_or_nothing_and_only_after_the_motor_part():
    with_motor = GOOD_LINE + ' ' + MOTOR_PART
    # Cut inside the steering part, steering without or before the motor part,
    # fields in the wrong order, an extra field, or the part twice.
    assert analysis.classify_line(with_motor + ' rst=36 s_pct=-40')[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + STEERING_PART)[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + STEERING_PART + ' ' + MOTOR_PART)[0] == 'malformed'
    assert analysis.classify_line(with_motor + ' ' + STEERING_PART.replace('s_us=1380 sstate=1', 'sstate=1 s_us=1380'))[0] == 'malformed'
    assert analysis.classify_line(with_motor + ' ' + STEERING_PART + ' extra=1')[0] == 'malformed'
    assert analysis.classify_line(with_motor + ' ' + STEERING_PART + ' ' + STEERING_PART)[0] == 'malformed'


BATTERY_PART = 'vbat=7921 bstate=1 vdda=3299'


def test_line_with_battery_fields_is_parsed():
    kind, fields = analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART + ' ' + BATTERY_PART + '\r')
    assert kind == 'telemetry'
    assert tuple(fields) == analysis.TELEMETRY_KEYS + analysis.MOTOR_KEYS + analysis.STEERING_KEYS + analysis.BATTERY_KEYS
    assert fields['vbat'] == 7921
    assert fields['bstate'] == 1
    assert fields['vdda'] == 3299


def test_battery_fields_are_all_or_nothing_and_only_after_the_steering_part():
    with_steering = GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART
    assert analysis.classify_line(with_steering + ' vbat=7921 bstate=1')[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + ' ' + BATTERY_PART)[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE + ' ' + MOTOR_PART + ' ' + BATTERY_PART + ' ' + STEERING_PART)[0] == 'malformed'
    assert analysis.classify_line(with_steering + ' ' + BATTERY_PART.replace('vbat=7921 bstate=1', 'bstate=1 vbat=7921'))[0] == 'malformed'
    assert analysis.classify_line(with_steering + ' ' + BATTERY_PART + ' extra=1')[0] == 'malformed'
    assert analysis.classify_line(with_steering + ' ' + BATTERY_PART + ' ' + BATTERY_PART)[0] == 'malformed'


SPEED_PART = 'vset=50 vest=49 ctl=1 vok=1 vi=307 stall_n=0 enc_bad=0 mfault=0'


def test_line_with_speed_fields_is_parsed():
    kind, fields = analysis.classify_line(
        GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART + ' ' + BATTERY_PART + ' ' + SPEED_PART + '\r')
    assert kind == 'telemetry'
    assert tuple(fields) == (analysis.TELEMETRY_KEYS + analysis.MOTOR_KEYS + analysis.STEERING_KEYS
                             + analysis.BATTERY_KEYS + analysis.SPEED_KEYS)
    assert fields['vest'] == 49 and fields['vi'] == 307


def test_speed_fields_are_all_or_nothing_and_only_after_the_battery_part():
    with_battery = GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART + ' ' + BATTERY_PART
    with_steering = GOOD_LINE + ' ' + MOTOR_PART + ' ' + STEERING_PART
    assert analysis.classify_line(with_battery + ' vset=50 vest=49')[0] == 'malformed'
    assert analysis.classify_line(with_steering + ' ' + SPEED_PART)[0] == 'malformed'
    assert analysis.classify_line(with_steering + ' ' + SPEED_PART + ' ' + BATTERY_PART)[0] == 'malformed'
    assert analysis.classify_line(with_battery + ' ' + SPEED_PART.replace('vset=50 vest=49', 'vest=49 vset=50'))[0] == 'malformed'
    assert analysis.classify_line(with_battery + ' ' + SPEED_PART + ' extra=1')[0] == 'malformed'
    assert analysis.classify_line(with_battery + ' ' + SPEED_PART + ' ' + SPEED_PART)[0] == 'malformed'


def test_speed_keys_match_the_logger():
    import serial_logger
    assert analysis.SPEED_KEYS == serial_logger.SPEED_FIELDS


def test_old_and_new_lines_in_one_log_give_the_same_columns():
    log_text = '\r\n'.join([
        make_line(1000),
        make_line(1100) + ' ' + MOTOR_PART,
        'imu_init=ok who=0x47',
        make_line(1200) + ' ' + MOTOR_PART,
        make_line(1300) + ' ' + MOTOR_PART + ' ' + STEERING_PART,
        make_line(1400) + ' ' + MOTOR_PART + ' ' + STEERING_PART + ' ' + BATTERY_PART,
    ]) + '\r\n'
    parsed = analysis.parse_log_text(log_text)
    assert parsed.good_line_count == 5
    assert parsed.malformed_line_count == 0
    assert tuple(parsed.columns) == analysis.TELEMETRY_KEYS
    assert list(parsed.columns['t_ms']) == [1000, 1100, 1200, 1300, 1400]


def test_boot_line_is_recognized():
    assert analysis.classify_line('imu_init=ok who=0x47\r') == ('boot', None)
    assert analysis.classify_line('imu_init=err_who_am_i who=0x00') == ('boot', None)


def test_truncated_line_is_malformed():
    # Cut at a token boundary (missing keys) and in the middle of a value.
    assert analysis.classify_line(GOOD_LINE[:120].rsplit(' ', 1)[0])[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE[: GOOD_LINE.index('gsum_z=') + 9])[0] == 'malformed'


def test_glued_lines_and_garbage_are_malformed():
    # Two lines run together after a lost newline repeat every key.
    assert analysis.classify_line(GOOD_LINE + ' ' + GOOD_LINE)[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE.replace('gz=-5', 'gz=-�5'))[0] == 'malformed'
    assert analysis.classify_line(GOOD_LINE.replace('gz=-5', 'gz-5'))[0] == 'malformed'


def test_parse_log_counts_each_kind_and_drops_unfinished_last_line():
    log_text = '\r\n'.join([
        GOOD_LINE[40:],            # partial first line (logger started mid-line)
        'imu_init=ok who=0x47',
        make_line(1000),
        make_line(1100),
        '',
        make_line(1200),
    ]) + '\r\n' + make_line(1300)[:50]  # still being written: no newline yet
    parsed = analysis.parse_log_text(log_text)
    assert parsed.good_line_count == 3
    assert parsed.boot_line_count == 1
    assert parsed.malformed_line_count == 1
    assert parsed.blank_line_count == 1
    assert parsed.partial_last_line
    assert list(parsed.columns['t_ms']) == [1000, 1100, 1200]


def test_complete_last_line_is_kept():
    parsed = analysis.parse_log_text(make_line(1000) + '\n' + make_line(1100) + '\n')
    assert parsed.good_line_count == 2
    assert not parsed.partial_last_line


def test_gap_splits_into_contiguous_segments():
    board_time_ms = np.array([100, 200, 300, 500, 600, 700, 800])
    samples_per_window = np.full(len(board_time_ms), 100)
    segments, gaps = analysis.find_contiguous_segments(board_time_ms, samples_per_window)
    assert gaps == [(3, 200)]
    assert segments == [(0, 3), (3, 7)]


def test_board_reset_and_empty_window_also_split_segments():
    board_time_ms = np.array([100, 200, 300, 400, 100, 200, 300])  # reset after index 3
    samples_per_window = np.array([100, 100, 0, 100, 100, 99, 100])  # no mean at index 2
    segments, gaps = analysis.find_contiguous_segments(board_time_ms, samples_per_window)
    assert gaps == [(4, -300)]
    assert segments == [(0, 2), (3, 4), (4, 7)]


# --- Allan deviation --------------------------------------------------------

def allan_curve(rate_samples):
    return analysis.overlapping_allan_deviation(
        rate_samples, BASE_INTERVAL_SECONDS, analysis.log_spaced_cluster_sizes(len(rate_samples)))


def predicted_relative_error(cluster_counts):
    return 1.0 / np.sqrt(2.0 * (cluster_counts - 1))


def log_log_slope(taus, deviations):
    return np.polyfit(np.log(taus), np.log(deviations), 1)[0]


def test_white_noise_gives_sigma_over_sqrt_m_and_slope_minus_half():
    sample_sigma = 0.7
    rate = np.random.default_rng(SEED).normal(0.3, sample_sigma, SAMPLE_COUNT)  # offset must not matter
    taus, deviations, _, cluster_counts = allan_curve(rate)
    # Averaging m independent samples divides the std by sqrt(m), and the
    # Allan deviation of white noise equals the std of one m-sample mean.
    cluster_sizes = taus / BASE_INTERVAL_SECONDS
    expected = sample_sigma / np.sqrt(cluster_sizes)
    relative_deviation = np.abs(deviations / expected - 1.0)
    assert np.all(relative_deviation <= 5.0 * predicted_relative_error(cluster_counts))
    assert log_log_slope(taus, deviations) == pytest.approx(-0.5, abs=0.1)
    assert taus[0] == pytest.approx(BASE_INTERVAL_SECONDS)
    assert taus[-1] <= SAMPLE_COUNT * BASE_INTERVAL_SECONDS / 9.0


def test_random_walk_gives_slope_plus_half():
    step_sigma = 1e-3
    rate = np.cumsum(np.random.default_rng(SEED).normal(0.0, step_sigma, SAMPLE_COUNT))
    taus, deviations, _, cluster_counts = allan_curve(rate)
    cluster_sizes = taus / BASE_INTERVAL_SECONDS
    # Exact Allan variance of a discrete random walk averaged over m steps:
    # step_sigma^2 (2 m^2 + 1) / (6 m). For large m that is step_sigma^2 m / 3,
    # i.e. K sqrt(tau / 3) with K = step_sigma / sqrt(tau_0) [IEEE 952 Annex C].
    expected = step_sigma * np.sqrt((2.0 * cluster_sizes ** 2 + 1.0) / (6.0 * cluster_sizes))
    relative_deviation = np.abs(deviations / expected - 1.0)
    assert np.all(relative_deviation <= 5.0 * predicted_relative_error(cluster_counts))
    long_taus = cluster_sizes >= 10  # where the +1/2 asymptote applies
    assert log_log_slope(taus[long_taus], deviations[long_taus]) == pytest.approx(0.5, abs=0.15)


def test_allan_deviation_matches_direct_definition_for_small_case():
    # Independent check of the cumulative-sum shortcut on a hand-sized input:
    # AVAR(m) = 1 / (2 (M - 2m + 1)) * sum_j (mean_{j+m..j+2m-1} - mean_{j..j+m-1})^2
    rate = np.array([1.0, 3.0, 2.0, 5.0, 4.0, 4.0, 1.0])
    for cluster_size in (1, 2, 3):
        differences = [rate[start + cluster_size:start + 2 * cluster_size].mean()
                       - rate[start:start + cluster_size].mean()
                       for start in range(len(rate) - 2 * cluster_size + 1)]
        expected = math.sqrt(0.5 * np.mean(np.square(differences)))
        _, deviations, _, _ = analysis.overlapping_allan_deviation(rate, 0.1, [cluster_size])
        assert deviations[0] == pytest.approx(expected, rel=1e-12)


# --- Reading N, B, K off the curve -----------------------------------------

def test_noise_parameters_from_white_noise_only():
    sample_sigma = 1.0
    rate = np.random.default_rng(SEED).normal(0.0, sample_sigma, SAMPLE_COUNT)
    taus, deviations, _, cluster_counts = allan_curve(rate)
    noise = analysis.extract_noise_parameters(taus, deviations, cluster_counts)
    true_angle_random_walk = sample_sigma * math.sqrt(BASE_INTERVAL_SECONDS)  # sigma(1 s) * sqrt(1 s)
    # At tau = 1 s there are 10,000 clusters: 1-sigma error 0.7 %, so 5 % is ~7 sigma.
    assert noise.angle_random_walk_read_at_1s == pytest.approx(true_angle_random_walk, rel=0.05)
    assert noise.angle_random_walk_fit == pytest.approx(true_angle_random_walk, rel=0.05)
    assert not noise.rate_random_walk_resolved
    assert noise.rate_random_walk > 0.0  # an upper bound always exists


def test_rate_random_walk_is_found_and_bound_holds_when_too_small_to_see():
    rng = np.random.default_rng(SEED)
    sample_sigma, step_sigma = 1.0, 0.003
    white = rng.normal(0.0, sample_sigma, SAMPLE_COUNT)
    walk = np.cumsum(rng.normal(0.0, step_sigma, SAMPLE_COUNT))
    true_rate_random_walk = step_sigma / math.sqrt(BASE_INTERVAL_SECONDS)

    taus, deviations, _, cluster_counts = allan_curve(white + walk)
    noise = analysis.extract_noise_parameters(taus, deviations, cluster_counts)
    # The fitted K scattered with std 0.126 (relative) over 200 Monte Carlo
    # seeds, and the +1/2 region was detected in 69 % of them. When it is
    # not detected, the fallback must still be a bound >= the true K.
    if noise.rate_random_walk_resolved:
        assert noise.rate_random_walk == pytest.approx(true_rate_random_walk, rel=0.5)
    else:
        assert noise.rate_random_walk >= true_rate_random_walk

    # A walk 30x weaker is invisible under the white noise in this record;
    # the 95 % upper bound must still sit above it.
    weak_walk = walk / 30.0
    taus, deviations, _, cluster_counts = allan_curve(white + weak_walk)
    noise = analysis.extract_noise_parameters(taus, deviations, cluster_counts)
    assert not noise.rate_random_walk_resolved
    assert noise.rate_random_walk >= true_rate_random_walk / 30.0


def test_upper_confidence_factor_shrinks_with_more_data():
    # Wilson-Hilferty vs. the exact chi-squared 5 % quantile for 10 dof
    # (3.9403, standard tables): sqrt(10 / 3.9403) = 1.593.
    assert analysis.allan_deviation_upper_confidence_factor(11) == pytest.approx(1.593, abs=0.01)
    assert analysis.allan_deviation_upper_confidence_factor(1000) < 1.06
    assert analysis.allan_deviation_upper_confidence_factor(1) > 1.0


# --- Output location ---------------------------------------------------------

def test_output_inside_repository_is_refused(tmp_path):
    with pytest.raises(SystemExit):
        analysis.resolve_output_directory(tmp_path / 'x.log', analysis.REPOSITORY_ROOT / 'results')
    default = analysis.resolve_output_directory(tmp_path / 'gyro.log', None)
    assert default == (tmp_path / 'gyro_analysis').resolve()
