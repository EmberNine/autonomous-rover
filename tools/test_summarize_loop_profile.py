"""Tests for summarize_loop_profile.py. Run from the repo root: python3 -m pytest tools/

Checks that the prof line layout matches the one main.c's Profile_BuildLineStep
prints (with the motor-report and steering-report kinds and the motor section;
tools/test_firmware_sources.py also checks it against main.c itself), and that telemetry
lines with motor fields (v2), steering fields (v3), battery fields (v4) and
speed control fields (v5) count as good, not as "other bad".
"""
import summarize_loop_profile as summary
from serial_logger import MOTOR_FIELDS, TELEMETRY_FIELDS_V1, TELEMETRY_FIELDS_V3, TELEMETRY_FIELDS_V4, TELEMETRY_FIELDS_V5

# Key order exactly as main.c prints it: step 1..9 of Profile_BuildLineStep.
FIRMWARE_PROF_KEYS = (
    't_ms', 'idle_n', 'idle_enc', 'idle_imu', 'idle_oth', 'idle_tot',
    'busy_n', 'busy_enc', 'busy_imu', 'busy_oth', 'busy_tot',
    'hlth_enc', 'hlth_rd', 'hlth_imu', 'hlth_oth', 'hlth_tot',
    'head_enc', 'head_imu', 'head_fmt', 'head_oth', 'head_tot',
    'tail_enc', 'tail_imu', 'tail_fmt', 'tail_oth', 'tail_tot',
    'motr_enc', 'motr_imu', 'motr_fmt', 'motr_oth', 'motr_tot',
    'strg_enc', 'strg_imu', 'strg_fmt', 'strg_oth', 'strg_tot',
    'spd_enc', 'spd_imu', 'spd_fmt', 'spd_tx', 'spd_oth', 'spd_tot',
    'mot', 'ctl_us', 'est_us', 'bld_tot',
)


def telemetry_line(t_ms, loop_us_max):
    values = {name: 1 for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS}
    values.update(t_ms=t_ms, loop_us_max=loop_us_max)
    return ' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V1 + MOTOR_FIELDS) + '\r\n'


def prof_line(t_ms, base):
    return 'prof ' + ' '.join(f'{key}={t_ms if key == "t_ms" else base + index}'
                              for index, key in enumerate(FIRMWARE_PROF_KEYS)) + '\r\n'


def test_profile_keys_match_the_firmware():
    assert summary.PROFILE_KEYS == FIRMWARE_PROF_KEYS
    for prefix, _description, sections in summary.ITERATION_KINDS:
        for section in sections + ('tot',):
            assert f'{prefix}_{section}' in summary.PROFILE_KEYS


def test_read_log_with_motor_firmware(tmp_path):
    log_path = tmp_path / 'profile.log'
    log_path.write_bytes(''.join([
        'imu_init=ok who=0x47\r\n',
        telemetry_line(100, 640), prof_line(100, 10),
        telemetry_line(200, 700), prof_line(200, 20),
    ]).encode())
    counts, profiles, loop_us_max_values, examples = summary.read_log(log_path)
    assert counts == {'good': 2, 'boot': 1, 'prof': 2, 'other bad': 0}
    assert loop_us_max_values == [640, 700]
    assert profiles[1]['t_ms'] == 200
    assert profiles[1]['mot'] == 20 + FIRMWARE_PROF_KEYS.index('mot')
    assert examples == []


def test_main_prints_every_kind(tmp_path, capsys, monkeypatch):
    log_path = tmp_path / 'profile.log'
    log_path.write_bytes((telemetry_line(100, 640) + prof_line(100, 10)).encode())
    monkeypatch.setattr('sys.argv', ['summarize_loop_profile.py', str(log_path)])
    assert summary.main() == 0
    printed = capsys.readouterr().out
    assert 'Motor section' in printed
    assert 'report iteration 3 (motor part): 1 windows' in printed
    assert 'report iteration 4 (reset cause + steering + battery part): 1 windows' in printed
    assert 'report iteration 5 (speed control part + UART start): 1 windows' in printed
    assert 'speed control + stall check (part of the motor section)' in printed


def test_read_log_with_v3_lines(tmp_path):
    def v3_line(t_ms, loop_us_max):
        values = {name: 1 for name in TELEMETRY_FIELDS_V3}
        values.update(t_ms=t_ms, loop_us_max=loop_us_max, rst=36, s_pct=-40)
        return ' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V3) + '\r\n'

    log_path = tmp_path / 'profile_v3.log'
    log_path.write_bytes(''.join([
        'imu_init=ok who=0x47\r\n',
        v3_line(100, 641), prof_line(100, 10),
        telemetry_line(200, 650),                      # a v2 line in the same log
        v3_line(300, 705), prof_line(300, 30),
    ]).encode())
    counts, profiles, loop_us_max_values, examples = summary.read_log(log_path)
    assert counts == {'good': 3, 'boot': 1, 'prof': 2, 'other bad': 0}
    assert loop_us_max_values == [641, 650, 705]
    assert examples == []


def test_read_log_with_v4_lines(tmp_path):
    def v4_line(t_ms, loop_us_max):
        values = {name: 1 for name in TELEMETRY_FIELDS_V4}
        values.update(t_ms=t_ms, loop_us_max=loop_us_max, vbat=7921, bstate=1, vdda=3299)
        return ' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V4) + '\r\n'

    log_path = tmp_path / 'profile_v4.log'
    log_path.write_bytes(''.join([
        'imu_init=ok who=0x47\r\n',
        v4_line(100, 690), prof_line(100, 10),
        telemetry_line(200, 650),                      # a v2 line in the same log
        v4_line(300, 702), prof_line(300, 30),
    ]).encode())
    counts, profiles, loop_us_max_values, examples = summary.read_log(log_path)
    assert counts == {'good': 3, 'boot': 1, 'prof': 2, 'other bad': 0}
    assert loop_us_max_values == [690, 650, 702]
    assert examples == []


def test_read_log_with_v5_lines(tmp_path):
    def v5_line(t_ms, loop_us_max):
        values = {name: 1 for name in TELEMETRY_FIELDS_V5}
        values.update(t_ms=t_ms, loop_us_max=loop_us_max, vset=50, vest=49, vi=307)
        return ' '.join(f'{name}={values[name]}' for name in TELEMETRY_FIELDS_V5) + '\r\n'

    log_path = tmp_path / 'profile_v5.log'
    log_path.write_bytes(''.join([
        'imu_init=ok who=0x47\r\n',
        v5_line(100, 712), prof_line(100, 10),
        telemetry_line(200, 650),                      # a v2 line in the same log
        v5_line(300, 735), prof_line(300, 30),
    ]).encode())
    counts, profiles, loop_us_max_values, examples = summary.read_log(log_path)
    assert counts == {'good': 3, 'boot': 1, 'prof': 2, 'other bad': 0}
    assert loop_us_max_values == [712, 650, 735]
    assert profiles[1]['ctl_us'] == 30 + FIRMWARE_PROF_KEYS.index('ctl_us')
    assert examples == []
