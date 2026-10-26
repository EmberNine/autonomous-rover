"""Tests for fit_angle_correction.py. Run from the repo root: python3 -m pytest tools/

Synthetic captures in the firmware's line format: a wheel whose real speed
has a mechanical once- and twice-per-turn variation (torque ripple, smaller
in % at higher speed), read by a sensor whose angle carries the 4-per-turn
error the firmware will remove. The measured angle is the exact inverse of
the firmware's correction (corrected = raw - eps(raw)), so the fitted
constants mean what angle_correction.c uses.
"""
import math

import numpy as np

import fit_angle_correction as tool
import fit_motor_step as captures_lib
import test_fit_motor_step as synthetic

TRUE_AMPLITUDE = 43.92      # counts
TRUE_PHASE = -8.6           # counts


def measured_angle(true_absolute, amplitude, phase):
    """Solves m - eps(m) = true for m (eps has slope < 1, so this converges)."""
    m = np.array(true_absolute, dtype=float)
    for _ in range(60):
        m = true_absolute + amplitude * np.sin(2 * math.pi * (m + phase) / 1024)
    return m


def capture_bytes(target, period, total_ms, amplitude=TRUE_AMPLITUDE, phase=TRUE_PHASE, h1=0.12, h2=0.05,
                  raw_start=1000, capture_id=1):
    duty = synthetic.firmware_duty(target, total_ms)
    params = synthetic.TRUE
    a = math.exp(-1.0 / params['tau'])
    w, p = 0.0, 0.0
    true = np.zeros(total_ms)
    for k in range(1, total_ms):
        j = k - 1 - params['theta']
        u = duty[j] if j >= 0 else 0.0
        drive = math.copysign(max(abs(u) - params['u0'], 0.0), u) * params['gain']
        if drive != 0:
            angle = 2 * math.pi * ((raw_start + p) % 4096) / 4096
            drive += math.copysign(1.0, u) * (h1 * math.sin(angle + 0.4) + h2 * math.sin(2 * angle + 1.1))
        w = a * w + (1 - a) * drive
        p += w
        true[k] = p
    measured = measured_angle(raw_start + true, amplitude, phase)
    raw0 = int(math.floor(measured[0] + 0.37)) % 4096
    data, _ = synthetic.capture_log(duty, measured - measured[0], period=period, raw0=raw0, capture_id=capture_id)
    return data


def fits_of(*datas):
    fits = []
    for index, data in enumerate(datas):
        for capture in captures_lib.parse_captures(data, f'log{index}.log'):
            fit = tool.fit_harmonics(f'log{index}.log:{capture.capture_id}', capture)
            assert fit is not None
            fits.append(fit)
    return fits


# --- Recovery --------------------------------------------------------------------------

def test_recovers_amplitude_and_phase_at_several_speeds_and_both_directions():
    for target, period, total in ((480, 2, 6000), (320, 2, 6000), (-320, 2, 6000), (160, 4, 12000)):
        (fit,) = fits_of(capture_bytes(target, period, total))
        amplitude_q4, phase_counts = tool.constants_from(fit)
        assert abs(amplitude_q4 / 16 - TRUE_AMPLITUDE) <= 1.0, (target, amplitude_q4 / 16)
        assert abs(phase_counts - TRUE_PHASE) <= 2.0, (target, phase_counts)
        # H1 is real (mechanical) and is reported, but it is not part of the constants.
        assert fit.amplitude(1) > 3.0


def test_h1_and_h2_do_not_leak_into_h4():
    (plain,) = fits_of(capture_bytes(480, 2, 6000, h1=0.0, h2=0.0))
    (rough,) = fits_of(capture_bytes(480, 2, 6000, h1=0.4, h2=0.3))
    assert abs(plain.amplitude(4) - rough.amplitude(4)) < 1.0
    assert abs(plain.phase_degrees(4) - rough.phase_degrees(4)) < 3.0


def test_constants_convention_matches_the_firmware_formula():
    # A fit with C4, S4 must give constants whose eps reproduces C4 cos + S4 sin.
    (fit,) = fits_of(capture_bytes(480, 2, 6000))
    amplitude_q4, phase_counts = tool.constants_from(fit)
    c4, s4 = fit.coefficients[4]
    raw = np.arange(4096)
    phi = 2 * math.pi * raw / 4096
    from_fit = c4 * np.cos(4 * phi) + s4 * np.sin(4 * phi)
    firmware = tool.correction_counts(raw, amplitude_q4, phase_counts)
    assert np.max(np.abs(from_fit - firmware)) < 1.0


# --- Invariance ----------------------------------------------------------------------------

def test_consistent_captures_give_no_warning():
    fits = fits_of(capture_bytes(480, 2, 6000), capture_bytes(320, 2, 6000), capture_bytes(-320, 2, 6000))
    assert tool.invariance_warnings(fits) == []


def test_a_different_amplitude_or_phase_warns():
    good = capture_bytes(480, 2, 6000)
    weaker = capture_bytes(320, 2, 6000, amplitude=30.0)
    moved = capture_bytes(320, 2, 6000, phase=TRUE_PHASE + 40)     # 14 degrees of H4 phase
    amplitude_warnings = tool.invariance_warnings(fits_of(good, weaker))
    phase_warnings = tool.invariance_warnings(fits_of(good, moved))
    assert any('amplitude' in w for w in amplitude_warnings) and not any('phase' in w for w in amplitude_warnings)
    assert any('phase' in w for w in phase_warnings) and not any('amplitude' in w for w in phase_warnings)
    assert 'only one capture' in tool.invariance_warnings(fits_of(good))[0]


# --- Residual ---------------------------------------------------------------------------------

def test_correction_removes_h4_and_leaves_h1():
    (fit,) = fits_of(capture_bytes(480, 2, 6000, h1=0.25))
    amplitude_q4, phase_counts = tool.constants_from(fit)
    before = tool.residual_of(fit.capture, fit.steady)
    corrected = tool.corrected_capture(fit.capture, amplitude_q4, phase_counts)
    after = tool.residual_of(corrected, captures_lib.steady_segment(corrected, None))
    assert before.h4_percent > 20.0 and after.h4_percent < 2.0, (before.h4_percent, after.h4_percent)
    assert abs(after.h1_percent - before.h1_percent) < 1.5        # the real H1 stays
    assert after.std_20ms < before.std_20ms / 2 and after.std_100ms < before.std_100ms / 2


def test_corrected_capture_keeps_missing_samples_missing():
    data = capture_bytes(480, 2, 6000)
    lines = data.split(b'\r\n')
    for i, line in enumerate(lines):
        if line.startswith(b'cap id=1 i=1000 '):
            head, d, u = line.split(b' d=')[0], line.split(b' d=')[1].split(b' u=')[0], line.split(b' u=')[1]
            values = d.split(b',')
            values[3] = b'x'
            values[4] = str(int(values[4]) + int(lines[i].split(b' d=')[1].split(b',')[3])).encode()
            lines[i] = head + b' d=' + b','.join(values) + b' u=' + u
    capture = captures_lib.parse_captures(b'\r\n'.join(lines), 'gap')[0]
    corrected = tool.corrected_capture(capture, 703, -9)
    assert np.isnan(corrected.deltas[1003]) and not np.isnan(corrected.deltas[1004])
    raw_positions = captures_lib.positions(capture)
    corrected_positions = captures_lib.positions(corrected)
    assert np.nanmax(np.abs(corrected_positions - raw_positions)) <= 2 * 44 + 1   # never more than 2 A apart


# --- Command line ---------------------------------------------------------------------------------

def write_logs(tmp_path):
    paths = []
    for name, args in (('cap_m30_c2_1.log', (480, 2, 6000)), ('cap_m20_c2_1.log', (320, 2, 6000)),
                       ('cap_m-20_c1_1.log', (-320, 1, 3000))):
        path = tmp_path / name
        path.write_bytes(capture_bytes(*args))
        paths.append(str(path))
    return paths


def test_main_prints_constants_from_the_capture_with_most_revolutions(tmp_path, capsys):
    paths = write_logs(tmp_path)
    assert tool.main(paths) == 0
    out = capsys.readouterr().out
    assert 'Constants from cap_m30_c2_1.log:1' in out
    assert '#define ANGLE_H4_AMPLITUDE_Q4' in out and '#define ANGLE_H4_PHASE_COUNTS' in out
    amplitude = int(out.split('#define ANGLE_H4_AMPLITUDE_Q4')[1].split()[0])
    phase = int(out.split('#define ANGLE_H4_PHASE_COUNTS')[1].split()[0])
    assert abs(amplitude / 16 - TRUE_AMPLITUDE) <= 1.0 and abs(phase - TRUE_PHASE) <= 2.0
    assert 'WARNING' not in out
    assert out.count(' -> ') >= 15          # the residual table: 5 columns for each of 3 captures
    assert 'H1' in out and 'H4' in out and 'H8' in out


def test_main_from_option(tmp_path, capsys):
    paths = write_logs(tmp_path)
    assert tool.main(paths + ['--from', 'cap_m-20_c1_1.log']) == 0
    assert 'Constants from cap_m-20_c1_1.log:1' in capsys.readouterr().out
    assert tool.main(paths + ['--from', 'cap_m20_c2_1:1']) == 0
    assert 'Constants from cap_m20_c2_1.log:1' in capsys.readouterr().out
    try:
        tool.main(paths + ['--from', 'nothing.log'])
        raise AssertionError('an unknown --from must stop')
    except SystemExit as stop:
        assert 'nothing.log' in str(stop)


def test_under_one_and_a_half_revolutions_is_skipped(tmp_path, capsys):
    # m 10 at c 1: ~1 count/ms over ~2.5 s of steady speed = under one turn.
    short = capture_bytes(160, 1, 3000)
    capture = captures_lib.parse_captures(short, 'short.log')[0]
    assert tool.fit_harmonics('short.log:1', capture) is None
    paths = write_logs(tmp_path)
    path = tmp_path / 'cap_m10_c1_1.log'
    path.write_bytes(short)
    assert tool.main(paths + [str(path)]) == 0
    out = capsys.readouterr().out
    assert 'cap_m10_c1_1.log:1: no steady segment of 1.5 revolutions or more, skipped' in out
    assert 'WARNING' not in out          # the short capture doesn't spoil the invariance check


def test_main_without_usable_captures(tmp_path, capsys):
    empty = tmp_path / 'empty.log'
    empty.write_bytes(b't_ms=1\r\n')
    assert tool.main([str(empty)]) == 1
