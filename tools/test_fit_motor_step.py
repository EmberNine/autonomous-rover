"""Tests for fit_motor_step.py. Run from the repo root: python3 -m pytest tools/

Synthetic captures in the firmware's exact line format (speed_capture.c):
a known motor (gain, dead zone, time constant, dead time, coast) driven by
the duty the firmware would apply (2 ticks/ms ramp from rest, then m 0),
quantised to whole counts, with some failed angle reads. The fitter has to
recover the parameters, keep missing samples out, and tell angle error from
mechanical ripple.
"""
import math

import numpy as np
import pytest

import fit_motor_step as fitter

TRUE = dict(gain=0.011, u0=70.0, tau=80.0, theta=3, tau_coast=400.0)  # counts/ms per tick, ticks, ms, ms, ms


def firmware_duty(target, total_ms, off_at_ms=None):
    """Applied duty per ms as motor_control.c makes it: wake-up ms at 0, a
    2 ticks/ms ramp, and after m 0 a ramp down and then the driver off."""
    duty, applied = [], 0
    for t in range(total_ms):
        goal = 0 if (off_at_ms is not None and t >= off_at_ms) else target
        if t > 0:
            step = 2 if goal > applied else (-2 if goal < applied else 0)
            applied = goal if abs(goal - applied) < 2 else applied + step
        duty.append(applied)
    return np.array(duty, dtype=float)


def true_positions(duty, params, angle_error=0.0, torque_ripple=0.0, raw0=0):
    """Exact model at 1 ms, with optional once-per-revolution angle error
    (counts added to the measured position) or torque ripple (counts/ms
    added to the drive, so the speed really varies)."""
    a = math.exp(-1.0 / params['tau'])
    ac = math.exp(-1.0 / params['tau_coast'])
    w, p = 0.0, 0.0
    result = np.zeros(len(duty))
    for k in range(1, len(duty)):
        j = k - 1 - params['theta']
        u = duty[j] if j >= 0 else 0.0
        angle = 2 * math.pi * ((raw0 + p) % 4096) / 4096
        if u != 0 or j < 0:
            drive = math.copysign(max(abs(u) - params['u0'], 0.0), u) * params['gain']
            if drive != 0:
                drive += torque_ripple * math.sin(angle)
            w = a * w + (1 - a) * drive
        else:
            w = ac * w
        p += w
        result[k] = p
    measured = result + angle_error * np.sin(2 * math.pi * ((raw0 + result) % 4096) / 4096)
    return measured


def capture_log(duty, measured, period=1, capture_id=1, raw0=1000, missing=(), lost_lines=(), count0=-500,
                vbat=7900, prev=0):
    """The log text the firmware would send for this capture, between
    telemetry-like and garbage lines. measured: true position per ms."""
    sample_ms = np.arange(0, len(duty), period)[:3000]
    counts = np.floor(measured[sample_ms] + 0.37).astype(int)      # what the encoder reads
    deltas, last_good = [], counts[0]
    for index, count in enumerate(counts):
        if index in missing:
            deltas.append('x')
        else:
            deltas.append(str(int(count - last_good)) if index else '0')
            last_good = count
    duties = [str(int(duty[t])) for t in sample_ms]
    target = int(max(duty, key=abs))
    lines = ['t_ms=100 raw=1 count=2 d100=0 garbage line for the parser to skip',
             f'caph id={capture_id} per={period} n={len(sample_ms)} t0=12345 count0={count0} raw0={raw0} '
             f'vbat={vbat} prev={prev} target={target}']
    for line_number, first in enumerate(range(0, len(sample_ms), 20)):
        if line_number in lost_lines:
            continue
        lines.append(f'cap id={capture_id} i={first} d={",".join(deltas[first:first + 20])} '
                     f'u={",".join(duties[first:first + 20])}')
        if line_number % 7 == 0:
            lines.append('imu_init=ok who=0x47')
    return ('\r\n'.join(lines) + '\r\n').encode(), counts


def one_capture(data):
    captures = fitter.parse_captures(data, 'synthetic')
    assert len(captures) == 1
    return captures[0]


# --- Parsing -----------------------------------------------------------------------

def test_parse_header_samples_and_missing_reads():
    duty = firmware_duty(320, 3000, off_at_ms=1500)
    data, counts = capture_log(duty, true_positions(duty, TRUE), missing={5, 6, 40, 1999})
    capture = one_capture(data)
    assert (capture.capture_id, capture.period_ms, capture.sample_count, capture.raw0, capture.vbat_mv) == \
        (1, 1, 3000, 1000, 7900)
    assert capture.usable == 3000 and not capture.notes
    assert np.array_equal(capture.duties, duty)
    positions = fitter.positions(capture)
    # Missing samples are missing (not "no motion"), and every good sample
    # is exactly where the encoder was: the delta after a gap covers it.
    for index in (5, 6, 40, 1999):
        assert math.isnan(positions[index])
    good = ~np.isnan(positions)
    assert np.array_equal(positions[good], (counts - counts[0])[good])


def test_a_lost_line_limits_the_capture_to_what_came_before():
    duty = firmware_duty(320, 3000, off_at_ms=1500)
    data, _ = capture_log(duty, true_positions(duty, TRUE), lost_lines={60})
    capture = one_capture(data)
    assert capture.usable == 1200
    assert any('not received' in note for note in capture.notes)
    fit = fitter.fit_capture(capture)
    assert fit is not None and abs(fit.tau_ms - TRUE['tau']) / TRUE['tau'] < 0.15


def test_several_captures_and_other_lines_in_one_log():
    duty = firmware_duty(320, 3000, off_at_ms=1500)
    first, _ = capture_log(duty, true_positions(duty, TRUE), capture_id=1)
    second, _ = capture_log(duty, true_positions(duty, TRUE), capture_id=2)
    captures = fitter.parse_captures(first + b'prof t_ms=1 idle_n=2\r\n' + second, 'two')
    assert [c.capture_id for c in captures] == [1, 2]
    assert all(c.usable == 3000 for c in captures)


# --- Fitting ---------------------------------------------------------------------------

def test_fit_recovers_the_motor_from_a_step_with_coast():
    duty = firmware_duty(320, 3000, off_at_ms=1500)
    data, _ = capture_log(duty, true_positions(duty, TRUE), missing={7, 300, 301, 2500})
    fit = fitter.fit_capture(one_capture(data))
    assert abs(fit.gain - TRUE['gain']) / TRUE['gain'] < 0.05, fit
    assert abs(fit.u0 - TRUE['u0']) <= 6, fit
    assert abs(fit.tau_ms - TRUE['tau']) / TRUE['tau'] < 0.10, fit
    assert abs(fit.theta_ms - TRUE['theta']) <= 1, fit
    assert abs(fit.tau_coast_ms - TRUE['tau_coast']) / TRUE['tau_coast'] < 0.20, fit
    assert fit.rms_counts < 1.5, fit


def test_fit_works_in_reverse():
    duty = firmware_duty(-320, 3000, off_at_ms=1500)
    data, _ = capture_log(duty, true_positions(duty, TRUE))
    fit = fitter.fit_capture(one_capture(data))
    assert abs(fit.gain - TRUE['gain']) / TRUE['gain'] < 0.05 and abs(fit.tau_ms - TRUE['tau']) / TRUE['tau'] < 0.10


def test_steady_speed_and_static_map_over_three_duties_and_sample_periods():
    steadies = []
    for target, period in ((160, 4), (320, 1), (480, 2)):
        total = 3000 * period
        duty = firmware_duty(target, total)
        data, _ = capture_log(duty, true_positions(duty, TRUE), period=period)
        capture = one_capture(data)
        fit = fitter.fit_capture(capture)
        steady = fitter.steady_segment(capture, fit)
        expected = TRUE['gain'] * (target - TRUE['u0'])          # counts/ms
        assert abs(steady.speed - expected) / expected < 0.01, (target, period, steady.speed, expected)
        steadies.append(steady)
    gain, u0 = fitter.static_map(steadies)
    assert abs(gain - TRUE['gain']) / TRUE['gain'] < 0.02 and abs(u0 - TRUE['u0']) < 3


def test_pi_gains_follow_lambda_tuning():
    kp, ki, lam = fitter.pi_gains(gain=0.011, tau_ms=80.0, theta_ms=3.0)
    gain_units = 0.011 / 0.05
    assert lam == 160.0
    assert kp == pytest.approx(80.0 / (gain_units * 163.0))
    assert ki == pytest.approx(kp * 10.0 / 80.0)


# --- Once-per-revolution ripple ------------------------------------------------------------

def ripple_at(target, period, total_ms, **disturbance):
    duty = firmware_duty(target, total_ms)
    data, _ = capture_log(duty, true_positions(duty, TRUE, raw0=1000, **disturbance), period=period, raw0=1000)
    capture = one_capture(data)
    fit = fitter.fit_capture(capture)
    return fitter.ripple(capture, fitter.steady_segment(capture, fit))


def test_angle_error_gives_the_same_percent_at_both_speeds():
    slow = ripple_at(160, 4, 12000, angle_error=20.0)          # m 10 with c 4: ~1 count/ms, 12 s
    fast = ripple_at(480, 1, 3000, angle_error=20.0)           # m 30 with c 1
    assert slow.revolutions >= 1.5 and fast.revolutions >= 1.5
    assert slow.percent > 4.0 and fast.percent > 4.0
    assert 0.75 <= fast.percent / slow.percent <= 1.33
    assert 'ANGLE ERROR' in fitter.ripple_verdict([slow, fast])


def test_mechanical_ripple_is_smaller_at_the_higher_speed():
    slow = ripple_at(160, 4, 12000, torque_ripple=0.15)
    fast = ripple_at(480, 1, 3000, torque_ripple=0.15)
    assert slow.percent > 2 * fast.percent
    assert 'MECHANICAL' in fitter.ripple_verdict([slow, fast])


def test_no_disturbance_is_at_the_noise_floor():
    slow = ripple_at(160, 4, 12000)
    fast = ripple_at(480, 1, 3000)
    assert slow.percent < fitter.RIPPLE_NOISE_FLOOR_PERCENT and fast.percent < fitter.RIPPLE_NOISE_FLOOR_PERCENT
    assert 'noise floor' in fitter.ripple_verdict([slow, fast])


def test_too_few_revolutions_and_too_close_speeds_are_reported():
    assert ripple_at(160, 1, 3000) is None                     # m 10 at c 1: under one revolution
    fast = ripple_at(480, 1, 3000, angle_error=20.0)
    assert 'two speeds' in fitter.ripple_verdict([fast, None])
    assert 'too close' in fitter.ripple_verdict([fast, fast])


# --- Command line ----------------------------------------------------------------------------

def test_main_prints_every_part(tmp_path, capsys):
    paths = []
    for target, period, total, name in ((160, 4, 12000, 'm10.log'), (480, 1, 3000, 'm30.log')):
        duty = firmware_duty(target, total, off_at_ms=total - 600)
        data, _ = capture_log(duty, true_positions(duty, TRUE, raw0=1000, angle_error=20.0), period=period)
        path = tmp_path / name
        path.write_bytes(data)
        paths.append(str(path))
    empty = tmp_path / 'empty.log'
    empty.write_bytes(b't_ms=1\r\n')
    assert fitter.main(paths + [str(empty)]) == 0
    out = capsys.readouterr().out
    for expected in ('capture 1', 'time constant', 'dead zone', 'coast time constant', 'steady:',
                     'once-per-revolution ripple', 'static map from 2 steady speeds', 'PI (lambda',
                     'ANGLE ERROR', 'no capture in this log'):
        assert expected in out, expected


def test_main_without_captures_fails(tmp_path):
    empty = tmp_path / 'empty.log'
    empty.write_bytes(b'')
    assert fitter.main([str(empty)]) == 1
