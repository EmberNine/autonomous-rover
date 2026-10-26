"""Source-invariant tests for the firmware. Run from the repo root: python3 -m pytest tools/

Some safety rules live in the firmware's structure, not in a function a host
test can call: where the hardware watchdog is refreshed, how it is started,
which build switches are off, what the emergency stop writes. These tests
read the C sources and fail if one of those rules is broken. They also check
that the telemetry and profile field orders printed by main.c are exactly
the ones the tools parse.
"""
import re
from pathlib import Path

import serial_logger
import summarize_loop_profile

FIRMWARE = Path(__file__).resolve().parents[1] / 'firmware'
MAIN_C = FIRMWARE / 'Core' / 'Src' / 'main.c'
IT_C = FIRMWARE / 'Core' / 'Src' / 'stm32l4xx_it.c'
DISPATCH_H = FIRMWARE / 'Core' / 'Inc' / 'command_dispatch.h'
STEERING_H = FIRMWARE / 'Core' / 'Inc' / 'steering_control.h'
BATTERY_H = FIRMWARE / 'Core' / 'Inc' / 'battery_monitor.h'
CAPTURE_H = FIRMWARE / 'Core' / 'Inc' / 'speed_capture.h'
ANGLE_CORRECTION_H = FIRMWARE / 'Core' / 'Inc' / 'angle_correction.h'
IOC = FIRMWARE / 'rover_firmware.ioc'


def source(path):
    return path.read_text(encoding='utf-8')


def without_comments(text):
    """C source with /* */ and // comments blanked out (string literals here never contain them)."""
    text = re.sub(r'/\*.*?\*/', lambda match: re.sub(r'[^\n]', ' ', match.group(0)), text, flags=re.DOTALL)
    return re.sub(r'//[^\n]*', '', text)


def block_between(text, begin_marker, end_marker):
    """The code between two CubeMX USER CODE markers (found in the raw text,
    where the markers are comments), with comments then removed."""
    start = text.index('*/', text.index(begin_marker)) + 2   # after the begin marker's comment
    end = text.rindex('/*', start, text.index(end_marker, start))  # before the end marker's comment
    return without_comments(text[start:end])


def function_body(text, signature):
    """The text between the braces of the function with this signature (brace matching)."""
    start = text.index('{', text.index(signature))
    depth = 0
    for position in range(start, len(text)):
        if text[position] == '{':
            depth += 1
        elif text[position] == '}':
            depth -= 1
            if depth == 0:
                return text[start + 1:position]
    raise AssertionError(f'unbalanced braces after {signature}')


# --- Hardware watchdog (IWDG) ---------------------------------------------------

def test_iwdg_is_refreshed_exactly_once_in_the_whole_firmware():
    calls = []
    for path in sorted((FIRMWARE / 'Core' / 'Src').glob('*.c')):
        code = without_comments(source(path))
        calls += [path.name] * code.count('HAL_IWDG_Refresh(')
    assert calls == ['main.c'], f'HAL_IWDG_Refresh calls: {calls}'


def test_iwdg_refresh_is_the_last_statement_of_the_main_loop():
    loop_body = block_between(source(MAIN_C), 'USER CODE BEGIN 3', 'USER CODE END 3')
    assert 'HAL_IWDG_Refresh(&hiwdg);' in loop_body, 'the refresh must be in the main loop body'
    after_refresh = loop_body.split('HAL_IWDG_Refresh(&hiwdg);', 1)[1]
    # Only the closing brace of while (1) may follow it.
    assert after_refresh.split() == ['}'], f'code after the refresh: {after_refresh.split()}'


def test_iwdg_refresh_is_not_in_any_interrupt_handler_or_callback():
    for path in sorted((FIRMWARE / 'Core' / 'Src').glob('*.c')):
        code = without_comments(source(path))
        for match in re.finditer(r'\bvoid\s+(\w+(?:Handler|Callback))\s*\([^)]*\)\s*\{', code):
            body = function_body(code, match.group(0))
            assert 'HAL_IWDG_Refresh' not in body, f'{path.name}: refresh inside {match.group(1)}'


def test_iwdg_is_started_once_from_user_code_after_its_debug_freeze():
    text = source(MAIN_C)
    assert len(re.findall(r'\bMX_IWDG_Init\(\);', without_comments(text))) == 1, \
        'MX_IWDG_Init must be called exactly once'
    user_code_2 = block_between(text, 'USER CODE BEGIN 2', 'USER CODE END 2')
    assert 'MX_IWDG_Init();' in user_code_2, 'the IWDG must be started from USER CODE 2 (CubeMX call disabled)'
    assert user_code_2.index('__HAL_DBGMCU_FREEZE_IWDG();') < user_code_2.index('MX_IWDG_Init();')


def test_iwdg_timeout_is_250_ms():
    body = function_body(source(MAIN_C), 'void MX_IWDG_Init(void)')
    assert 'hiwdg.Init.Prescaler = IWDG_PRESCALER_32;' in body
    assert 'hiwdg.Init.Reload = 249;' in body
    assert 'hiwdg.Init.Window = 4095;' in body  # window off


# --- Build switches -------------------------------------------------------------

def test_hang_test_switch_is_off():
    assert re.search(r'^#define IWDG_HANG_TEST\s+0\s*$', source(DISPATCH_H), re.MULTILINE), \
        'IWDG_HANG_TEST must be committed as 0'


def test_dispatcher_gets_the_hang_switch_not_a_constant():
    code = without_comments(source(MAIN_C))
    calls = re.findall(r'CommandDispatch_Line(?:WithSpeed)?\(([^;]*)\);', code)
    assert len(calls) == 1, 'main.c must dispatch command lines in exactly one place'
    arguments = [argument.strip() for argument in calls[0].split(',')]
    # (line_result, &command, motor, steering, speed, hang_test_enabled, now_ms)
    assert arguments[5] == '(IWDG_HANG_TEST != 0)', f'hang_test_enabled argument: {arguments[5]}'


def test_hang_stall_only_exists_in_a_hang_test_build():
    loop_body = block_between(source(MAIN_C), 'USER CODE BEGIN 3', 'USER CODE END 3')
    stall = loop_body.index('stall_start_ms')
    guard = loop_body.rindex('#if IWDG_HANG_TEST', 0, stall)
    assert '#endif' not in loop_body[guard:stall] and '#else' not in loop_body[guard:stall]


# --- Steering limits and output guard -------------------------------------------

def test_steering_limits_are_the_calibrated_ones():
    # Pinned on purpose. The host tests derive their expectations from these
    # constants, so they would pass with any limits; but MIN and MAX keep the
    # servo off its mechanical stops and CENTER is "straight" on link loss.
    # These are the values from the 2026-10-10 bench calibration (README,
    # "Steering calibration"). Change them only after recalibrating, and update
    # this test, the README and the Pi's steering mapping in the same commit.
    code = without_comments(source(STEERING_H))
    limits = dict(re.findall(r'#define\s+(SERVO_PULSE_(?:MIN|CENTER|MAX)_US)\s+(\d+)\s*$', code, flags=re.MULTILINE))
    assert limits == {'SERVO_PULSE_MIN_US': '1200', 'SERVO_PULSE_CENTER_US': '1500', 'SERVO_PULSE_MAX_US': '1800'}


def test_steering_locks_out_when_the_servo_guard_fires():
    # main.c must act on the guard every ms: a refused timer or an out-of-range
    # pulse puts the steering in FAULT (no pulses from the next ms on).
    loop_body = block_between(source(MAIN_C), 'USER CODE BEGIN 3', 'USER CODE END 3')
    match = re.search(r'ServoPlan\s+(\w+)\s*=\s*Servo_Apply\(', loop_body)
    assert match, 'the main loop must keep the plan returned by Servo_Apply'
    plan = match.group(1)
    guarded = re.search(
        r'if\s*\(\s*' + plan + r'\.timer_refused\s*\|\|\s*' + plan + r'\.out_of_range\s*\)\s*'
        r'\{\s*SteeringController_LockOut\(&steering_controller\);\s*\}',
        loop_body[match.end():])
    assert guarded, f'no "if ({plan}.timer_refused || {plan}.out_of_range) SteeringController_LockOut(...)" after Servo_Apply'


# --- Emergency stop -------------------------------------------------------------

def test_emergency_stop_ends_servo_pulses_without_cutting_one_short():
    body = without_comments(function_body(source(MAIN_C), 'void Board_EmergencyStop(void)'))
    assert 'TIM15->CCR2 = 0u;' in body
    # Clearing MOE or re-muxing PA3 would cut the pulse in progress short.
    assert 'TIM15->BDTR' not in body
    assert 'SERVO_PWM' not in body
    # Motor first.
    assert body.index('MOTOR_STBY_GPIO_Port->BRR') < body.index('TIM15->CCR2')


def test_every_fault_handler_stops_the_actuators_first():
    code = without_comments(source(IT_C))
    for handler in ('NMI_Handler', 'HardFault_Handler', 'MemManage_Handler', 'BusFault_Handler',
                    'UsageFault_Handler'):
        body = function_body(code, f'void {handler}(void)')
        assert body.split()[0] == 'Board_EmergencyStop();', f'{handler} must call Board_EmergencyStop first'
    error_handler = without_comments(function_body(source(MAIN_C), 'void Error_Handler(void)'))
    assert error_handler.split()[0] == 'Board_EmergencyStop();'


# --- Loop scheduling ----------------------------------------------------------------

def main_loop_code():
    """The body of the main loop (USER CODE 3), without comments."""
    return block_between(source(MAIN_C), '/* USER CODE BEGIN 3 */', '/* USER CODE END 3 */')


def test_no_modulo_schedules_in_the_main_loop():
    # "t % period == phase" is never true again once long iterations (a stuck
    # I2C bus: ~52 ms each) step past it: no telemetry, no heartbeat, while the
    # loop runs and the motor can be armed (firmware/Core/Inc/loop_schedule.h).
    modulo_comparisons = re.findall(r'%[^;{}\n]*[=!]=|[=!]=[^;{}\n]*%', main_loop_code())
    assert modulo_comparisons == []


def test_periodic_tasks_are_due_time_schedules():
    loop = main_loop_code()
    for schedule in ('health_schedule', 'report_schedule', 'heartbeat_schedule', 'battery_schedule',
                     'link_state_schedule', 'link_status_schedule'):
        # Exactly once per iteration: IsDue consumes the due time.
        assert loop.count(f'PeriodicSchedule_IsDue(&{schedule}, scheduled_time_ms)') == 1, schedule


# A function's definition, not its prototype (which comes first in main.c).
DEFINITION = '{}\n{{'
COMMANDS_PROCESS_RECEIVED = DEFINITION.format(
    'static uint32_t Commands_ProcessReceived(CommandLine *command_line, MotorController *motor,\n'
    '                                         SpeedController *speed, SteeringController *steering,\n'
    '                                         uint32_t *rx_trouble_count_seen, bool *hello_received, uint32_t now_ms)')


def define_value(text, name):
    match = re.search(r'^#define\s+' + name + r'\s+(\d+)u?\b', text, re.MULTILINE)
    assert match, f'no #define {name}'
    return int(match.group(1))


def test_battery_samples_never_share_a_ms_with_the_report_or_health_iterations():
    # The report head (t % 100 == 0), its follow-on parts (1 to the number
    # of later steps) and health (50) are the longest iterations; the battery
    # sample must never land on one of them (in 1 ms steps, where the marks
    # are exact).
    code = source(MAIN_C)
    period, phase = define_value(code, 'BATTERY_PERIOD_MS'), define_value(code, 'BATTERY_PHASE_MS')
    report, health_phase = define_value(code, 'REPORT_PERIOD_MS'), define_value(code, 'HEALTH_PHASE_MS')
    later_steps = len(re.findall(r'^\s*LINE_\w+_PART_NEXT\b', code, re.MULTILINE))
    assert later_steps == 4   # IMU, motor, steering + battery, speed + send (v5)
    busy = set(range(later_steps + 1)) | {health_phase}
    marks = {t % report for t in range(phase, 10 * report, period)}
    assert not marks & busy, sorted(marks & busy)
    assert 'PeriodicSchedule_Init(&battery_schedule, BATTERY_PERIOD_MS, BATTERY_PHASE_MS, next_deadline_ms);' in code


# --- Battery ------------------------------------------------------------------------

def test_battery_cutoff_test_switch_is_off():
    text = source(BATTERY_H)
    assert re.search(r'^#define BATTERY_CUTOFF_TEST\s+0\b', text, re.MULTILINE), \
        'BATTERY_CUTOFF_TEST must be committed as 0'
    # A test build says so when it compiles.
    guarded = text[text.index('#if BATTERY_CUTOFF_TEST'):]
    assert guarded.index('#warning') < guarded.index('#else')


def test_battery_lockout_reaches_the_motor_every_ms():
    loop = main_loop_code()
    match = re.search(r'MotorFaultReason\s+(\w+)\s*=\s*BatteryAdc_MotorFaultReason\(&battery_monitor\);', loop)
    assert match, 'the main loop must ask the battery monitor for a lockout every ms'
    reason = match.group(1)
    assert re.search(r'if\s*\(\s*' + reason + r'\s*!=\s*MOTOR_FAULT_NONE\s*\)\s*'
                     r'\{\s*MotorController_LockOut\(&motor_controller,\s*' + reason + r'\);\s*\}',
                     loop[match.end():]), 'no MotorController_LockOut(&motor_controller, ...) for the battery'
    # Not inside the 10 ms block: also a fault from boot locks out at once.
    battery_block = loop[loop.index('PeriodicSchedule_IsDue(&battery_schedule'):match.start()]
    assert battery_block.count('{') == battery_block.count('}')
    # Every latched battery state maps to its own reason.
    body = without_comments(function_body(source(MAIN_C), DEFINITION.format(
        'static MotorFaultReason BatteryAdc_MotorFaultReason(const BatteryMonitor *battery)')))
    for state, fault in (('BATTERY_STATE_ADC_FAULT', 'MOTOR_FAULT_BATTERY_ADC'),
                         ('BATTERY_STATE_CUTOFF', 'MOTOR_FAULT_LOW_BATTERY'),
                         ('BATTERY_STATE_REMOVED_WHILE_DRIVING', 'MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING')):
        assert re.search(r'case\s+' + state + r':\s*return\s+' + fault + ';', body), state


def test_battery_sampling_never_waits_and_gets_the_real_duty():
    loop = main_loop_code()
    # No HAL ADC calls (they lock, change state and can wait) in the loop.
    assert 'HAL_ADC' not in loop
    start = loop.index('PeriodicSchedule_IsDue(&battery_schedule')
    block = loop[start:loop.index('BatteryAdc_MotorFaultReason', start)]
    assert 'while' not in block and 'for (' not in block
    assert 'LL_ADC_IsActiveFlag_JEOS(ADC1)' in block and 'BatteryMonitor_MissedSample(&battery_monitor);' in block
    assert re.search(r'BatteryMonitor_Update\(&battery_monitor,\s*true,\s*vrefint_raw,\s*battery_raw,\s*'
                     r'motor_output\.duty_ticks != 0u,\s*now_ms\);', block)
    # Ranks: 1 is VREFINT, 2 the battery (as configured in MX_ADC1_Init).
    assert 'vrefint_raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)' in block
    assert 'battery_raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2)' in block
    adc_init = without_comments(function_body(source(MAIN_C), DEFINITION.format('static void MX_ADC1_Init(void)')))
    assert re.search(r'InjectedChannel = ADC_CHANNEL_VREFINT;\s*sConfigInjected\.InjectedRank = ADC_INJECTED_RANK_1;',
                     adc_init)
    assert re.search(r'InjectedChannel = ADC_CHANNEL_11;\s*sConfigInjected\.InjectedRank = ADC_INJECTED_RANK_2;',
                     adc_init)


def test_battery_adc_failure_at_boot_is_a_fault():
    user_code_2 = block_between(source(MAIN_C), 'USER CODE BEGIN 2', 'USER CODE END 2')
    assert 'BatteryMonitor_Init(&battery_monitor, *VREFINT_CAL_ADDR);' in user_code_2
    assert re.search(r'if\s*\(\s*!BatteryAdc_Start\(\)\s*\)\s*\{\s*BatteryMonitor_Update\(&battery_monitor,\s*false,',
                     user_code_2)


def test_battery_adc_boot_waits_all_time_out_well_under_the_iwdg():
    # The IWDG (250 ms) is already running at boot, so a wait that spins
    # forever on a dead ADC would end in a reset loop instead of a latched ADC
    # fault (which locks the motor out and keeps steering and telemetry).
    # Every wait loop in BatteryAdc_Start must give up within a few ms and
    # return false (-> the fault path checked above).
    body = without_comments(function_body(source(MAIN_C), DEFINITION.format('static bool BatteryAdc_Start(void)')))
    waits = list(re.finditer(r'while\s*\(([^{]*)\)\s*\{((?:[^{}]|\{[^{}]*\})*)\}', body))  # one level of nested braces
    assert any('LL_ADC_IsActiveFlag_JEOS(ADC1)' in wait.group(1) for wait in waits), 'no wait for the end of the sequence'
    for wait in waits:
        timeout = re.search(r'if\s*\(\s*HAL_GetTick\(\)\s*-\s*(\w+)\s*>\s*(\d+)u?\s*\)\s*\{\s*return false;\s*\}',
                            wait.group(2))
        assert timeout, f'wait without a timeout that returns false: while ({wait.group(1).strip()})'
        start_variable, timeout_ms = timeout.group(1), int(timeout.group(2))
        assert timeout_ms <= 10, f'boot wait timeout {timeout_ms} ms: keep it well under the 250 ms IWDG'
        assert re.search(r'\b' + start_variable + r'\s*=\s*HAL_GetTick\(\);', body[:wait.start()]), \
            f'{start_variable} must be set from HAL_GetTick() before the wait'


def test_adc_sampling_time_is_640_cycles_in_ioc_and_code():
    # 640.5 cycles: a long sample for the divider's source impedance and for
    # VREFINT (which needs a minimum sampling time); ~41 us per sequence.
    ioc = source(IOC)
    for rank in (1, 2):
        assert f'ADC1.InjectedSamplingTime-{rank}\\#ChannelInjectedConversion=ADC_SAMPLETIME_640CYCLES_5' in ioc
    adc_init = without_comments(function_body(source(MAIN_C), DEFINITION.format('static void MX_ADC1_Init(void)')))
    assert adc_init.count('ADC_SAMPLETIME_640CYCLES_5') == 1  # set once, kept for rank 2


def test_pb6_is_never_driven_or_pulled():
    # On the Nucleo-L432KC, SB16 ties PB6 to A5 (PA6, the battery divider).
    # PB6 must stay analog with no pull, or it would load the divider.
    ioc = source(IOC)
    assert 'PB6.Signal=GPIO_Analog' in ioc
    assert 'PB6.GPIO_PuPd' not in ioc
    gpio_init = without_comments(function_body(source(MAIN_C), DEFINITION.format('static void MX_GPIO_Init(void)')))
    assert re.search(r'GPIO_InitStruct\.Pin = SB16_A5_KEEP_ANALOG_Pin;\s*GPIO_InitStruct\.Mode = GPIO_MODE_ANALOG;'
                     r'\s*GPIO_InitStruct\.Pull = GPIO_NOPULL;\s*HAL_GPIO_Init\(SB16_A5_KEEP_ANALOG_GPIO_Port', gpio_init)
    # Nothing else touches it.
    for path in sorted((FIRMWARE / 'Core' / 'Src').glob('*.c')):
        code = without_comments(source(path))
        uses = code.count('SB16_A5_KEEP_ANALOG')
        assert uses == (2 if path == MAIN_C else 0), (path.name, uses)


# --- Speed capture (TEMPORARY bench instrumentation) --------------------------------

def without_blocks(text, condition):
    """text with every "#if <condition>" ... matching "#endif" block removed
    (nested #if/#endif inside it included); the #else branch, if any, kept."""
    kept, depth, skipping = [], 0, False
    for line in text.split('\n'):
        stripped = line.strip()
        if not skipping:
            if stripped == f'#if {condition}':
                skipping, depth = True, 1
                continue
            kept.append(line)
            continue
        if stripped.startswith('#if'):
            depth += 1
        elif stripped.startswith('#endif'):
            depth -= 1
            if depth == 0:
                skipping = False
        elif stripped.startswith('#else') and depth == 1:
            skipping = False   # the #else branch is the normal build's code
            depth = 0
    return '\n'.join(kept)


def test_loop_timing_profile_switch_is_off_and_pinned():
    # Like the other build switches: committed as 0, and a #warning when 1
    # (inside the first "#if LOOP_TIMING_PROFILE" block, right after the define).
    text = without_comments(source(MAIN_C))   # the comment above the define quotes "#if LOOP_TIMING_PROFILE"
    assert re.search(r'^#define LOOP_TIMING_PROFILE\s+0\b', text, re.MULTILINE), \
        'LOOP_TIMING_PROFILE must be committed as 0'
    guarded = text[text.index('#if LOOP_TIMING_PROFILE'):]
    assert guarded.index('#warning') < guarded.index('#endif')
    assert text.index('#define LOOP_TIMING_PROFILE') < text.index('#if LOOP_TIMING_PROFILE')


def test_loop_timing_profile_is_compiled_out_of_a_normal_build():
    # Outside the "#if LOOP_TIMING_PROFILE" blocks only the profile's type
    # declarations (no code, no RAM), the switch itself, its place in the
    # test-build list (text commands) and its HELLO_ACK build flag may mention it.
    code = without_comments(source(MAIN_C))
    normal = without_blocks(code, 'LOOP_TIMING_PROFILE')
    normal = re.sub(r'typedef\s+(?:enum|struct)\s*\{.*?\}\s*\w+;', '', normal, flags=re.DOTALL)
    leftovers = [line.strip() for line in normal.split('\n') if re.search(r'[Pp]rofile|PROFILE', line)]
    assert leftovers == ['#define LOOP_TIMING_PROFILE   0',
                         '(LOOP_TIMING_PROFILE != 0))',
                         '((LOOP_TIMING_PROFILE != 0) ? LINK_BUILD_LOOP_TIMING_PROFILE : 0u) | \\'], leftovers


def test_angle_correction_constants_are_the_fitted_ones():
    # Pinned on purpose, like the steering limits: a sensor calibration for
    # the encoder board's current seating (on its pegs, not glued), fitted
    # with tools/fit_angle_correction.py from cap_m30_c2_1.log (2026-10-15).
    # Change them only with a re-fit, together with the header comment.
    code = without_comments(source(ANGLE_CORRECTION_H))
    constants = dict(re.findall(r'^#define\s+(ANGLE_H4_(?:AMPLITUDE_Q4|PHASE_COUNTS))\s+\(?(-?\d+)\)?\s*$', code,
                                flags=re.MULTILINE))
    assert constants == {'ANGLE_H4_AMPLITUDE_Q4': '703', 'ANGLE_H4_PHASE_COUNTS': '-9'}
    header = source(ANGLE_CORRECTION_H)
    for note in ('cap_m30_c2_1', '2026-10-15', 'NOT glued', 're-fit'):
        assert note in header, note


def test_speed_capture_switch_is_off():
    text = source(CAPTURE_H)
    assert re.search(r'^#define SPEED_CAPTURE\s+0\b', text, re.MULTILINE), 'SPEED_CAPTURE must be committed as 0'
    guarded = text[text.index('#if SPEED_CAPTURE'):]
    assert guarded.index('#warning') < guarded.index('#endif')


def test_speed_capture_is_compiled_out_of_a_normal_build():
    # Everything the capture adds to main.c sits inside "#if SPEED_CAPTURE"
    # blocks: without them, nothing of it is left but the include. So a
    # normal build is the firmware it was before (same flash image).
    code = without_comments(source(MAIN_C))
    assert '#if SPEED_CAPTURE' in code
    normal = without_blocks(code, 'SPEED_CAPTURE')
    for name in ('SpeedCapture', 'speed_capture', 'capture_', 'CAPTURE_'):
        leftovers = [line.strip() for line in normal.split('\n')
                     if name in line and '#include "speed_capture.h"' not in line]
        assert leftovers == [], (name, leftovers)
    # Not with the profiler, which also sends an extra line at ms 51.
    capture_pd = code[code.index('#if SPEED_CAPTURE'):]
    assert re.search(r'#if LOOP_TIMING_PROFILE\s*#error', capture_pd)


def test_capture_command_and_recording_are_where_they_should_be():
    main_c = source(MAIN_C)
    # "c" is handled before dispatch, only in a capture build, feeds no
    # watchdog, and a refused value counts as a rejected line.
    receive = function_body(main_c, COMMANDS_PROCESS_RECEIVED)
    block = receive[receive.index('#if SPEED_CAPTURE'):receive.index('#endif', receive.index('#if SPEED_CAPTURE'))]
    assert "command.letter == 'c'" in block and 'SpeedCapture_Command(&speed_capture, command.value)' in block
    assert 'MotorController_CountRejected(motor);' in block and 'AcceptCommand' not in block
    # Recording: the target and applied duty from the motor controller, the
    # fresh count, right after the angle read and before the magnitude read.
    loop = without_comments(block_between(main_c, '/* USER CODE BEGIN 3 */', '/* USER CODE END 3 */'))
    angle, tick, magnitude = (loop.index('AS5600_ReadRawAngle('), loop.index('SpeedCapture_Tick(&speed_capture'),
                              loop.index('AS5600_ReadMagnitude('))
    assert angle < tick < magnitude
    for field in ('.target_ticks = motor_controller.target_ticks', '.applied_ticks = motor_controller.applied_ticks',
                  '.count = wheel_encoder.continuous_count',
                  '.read_ok = (i2c_error_count == capture_i2c_errors_before)'):
        assert field in loop[angle:magnitude], field
    # Lines go out only when the UART is free and no telemetry line is being built.
    send = loop[loop.index('HAL_UART_Transmit_IT(&huart2, (uint8_t *)capture_line') - 400:]
    assert 'huart2.gState == HAL_UART_STATE_READY' in send and 'line_step == LINE_IDLE' in send
    assert loop.count('PeriodicSchedule_IsDue(&capture_schedule, scheduled_time_ms)') == 1


ENCODER_FAULT_H = FIRMWARE / 'Core' / 'Inc' / 'encoder_fault.h'


def test_encoder_fault_test_switch_is_off_and_pinned():
    text = source(ENCODER_FAULT_H)
    assert re.search(r'^#define ENCODER_FAULT_TEST\s+0\b', text, re.MULTILINE), 'ENCODER_FAULT_TEST must be committed as 0'
    guarded = text[text.index('#if ENCODER_FAULT_TEST'):]
    assert guarded.index('#warning') < guarded.index('#endif')
    code = without_comments(text)
    assert define_value(code, 'ENCODER_FAULT_SHORT_OUTAGE_MS') == 20
    assert define_value(code, 'ENCODER_FAULT_LONG_OUTAGE_MS') == 100
    assert define_value(code, 'ENCODER_FAULT_FROZEN_ANGLE_MS') == 700
    # Never together with a capture build: a capture records the real encoder.
    main_c = without_comments(source(MAIN_C))
    assert re.search(r'#if ENCODER_FAULT_TEST && SPEED_CAPTURE\s*#error', main_c)


def test_encoder_fault_is_compiled_out_of_a_normal_build():
    code = without_comments(source(MAIN_C))
    normal = without_blocks(code, 'ENCODER_FAULT_TEST')
    for name in ('EncoderFault', 'encoder_fault', "'e'"):
        leftovers = [line.strip() for line in normal.split('\n')
                     if name in line and '#include "encoder_fault.h"' not in line]
        assert leftovers == [], (name, leftovers)


def test_encoder_fault_command_and_injection_are_where_they_should_be():
    main_c = source(MAIN_C)
    # "e" is handled before dispatch, only in a fault test build, feeds no
    # watchdog, and a refused value counts as a rejected line.
    receive = function_body(main_c, COMMANDS_PROCESS_RECEIVED)
    start = receive.index('#if ENCODER_FAULT_TEST')
    block = without_comments(receive[start:receive.index('#endif', start)])
    assert re.search(r"if \(line_result == COMMAND_LINE_VALID && command\.letter == 'e'\)\s*\{\s*"
                     r"if \(!EncoderFault_Command\(&encoder_fault, command\.value, now_ms\)\)\s*\{\s*"
                     r"MotorController_CountRejected\(motor\);\s*\}\s*continue;\s*\}", block), block
    assert 'Accept' not in block
    assert receive.index('#if ENCODER_FAULT_TEST') < receive.index('CommandDispatch_LineWithSpeed(')
    # Injection: after the real read and its i2c_err count, before both
    # unwrappers and the estimate; once per iteration.
    loop = main_loop_code()
    order = [loop.index(call) for call in (
        'AS5600_ReadRawAngle(&hi2c1, &raw_angle)',
        'i2c_error_count++;',
        'EncoderFault_Apply(&encoder_fault, &angle_read_ok, &raw_angle, now_ms);',
        'AS5600_UnwrapperUpdate(&wheel_encoder, raw_angle);',
        'SpeedEstimator_Update(&speed_estimator, &estimator_input);')]
    assert order == sorted(order), order
    assert loop.count('EncoderFault_Apply(') == 1


# --- Closed loop: through motor_control, never around it ----------------------------

def test_speed_control_and_stall_detect_act_on_the_motor_only_through_its_api():
    # The speed controller's output and the stall response must go through
    # motor_control (cap, ramp, reverse dwell, watchdog, latch, FAULT, guards):
    # no direct write to a MotorController field, and only these calls.
    allowed = {
        'speed_control.c': {'MotorController_SetClosedLoopTarget', 'MotorController_Latch'},
        'stall_detect.c': {'MotorController_Latch', 'MotorController_LockOut'},
    }
    for name, calls in allowed.items():
        code = without_comments(source(FIRMWARE / 'Core' / 'Src' / name))
        writes = re.findall(r'motor->\w+(?:\.\w+)*\s*(?:[-+*/|&]?=(?!=)|\+\+|--)', code)
        assert writes == [], (name, writes)
        assert set(re.findall(r'MotorController_\w+', code)) == calls, name


def test_saturation_stall_constants_and_read_only_speed_controller():
    # Pinned like the other stall numbers (also by tests/test_stall_detect.c):
    # at the cap, below 1/2 of the setpoint, for 1000 ms. The detector only
    # reads the speed controller (const), and calls none of its functions.
    header = without_comments(source(FIRMWARE / 'Core' / 'Inc' / 'stall_detect.h'))
    assert define_value(header, 'STALL_SAT_HOLD_MS') == 1000
    assert define_value(header, 'STALL_SAT_SPEED_NUMERATOR') == 1
    assert define_value(header, 'STALL_SAT_SPEED_DENOMINATOR') == 2
    assert 'const SpeedController *speed' in header
    code = without_comments(source(FIRMWARE / 'Core' / 'Src' / 'stall_detect.c'))
    assert re.findall(r'SpeedControl_\w+\(', code) == []
    assert re.findall(r'speed->\w+\s*(?:[-+*/|&]?=(?!=)|\+\+|--)', code) == []
    # The floor note (an unreachable setpoint latches by design) stays with the rule.
    raw = source(FIRMWARE / 'Core' / 'Inc' / 'stall_detect.h')
    assert 'BY DESIGN, on the floor' in raw and 'The Pi must command reachable speeds' in raw


def test_only_the_dispatcher_feeds_the_motor_watchdog():
    # "m" and "v" are the only things that feed the motor watchdog, and the
    # only callers of the two functions that do are in command_dispatch.c
    # (the speed controller's own output, SetClosedLoopTarget, doesn't).
    for path in sorted((FIRMWARE / 'Core' / 'Src').glob('*.c')):
        if path.name in ('command_dispatch.c', 'motor_control.c'):
            continue
        code = without_comments(source(path))
        for feeder in ('MotorController_AcceptCommand(', 'MotorController_AcceptSpeedCommand('):
            assert feeder not in code, (path.name, feeder)
    dispatch = without_comments(source(FIRMWARE / 'Core' / 'Src' / 'command_dispatch.c'))
    assert dispatch.count('MotorController_AcceptCommand(') == 1
    assert dispatch.count('MotorController_AcceptSpeedCommand(') == 1


# --- Field orders: firmware vs tools ------------------------------------------------

TELEMETRY_FIELD = re.compile(r'TelemetryWriter_Append(Signed|Unsigned)Field\(&line_writer,\s*"(\w+)"')


def telemetry_fields_in_line_order(code):
    """(kind, key) of every telemetry field, in the order they appear in a line.
    In the source the later parts (IMU, motor, steering + battery, speed, in
    that order) are formatted above the head part that starts the line at
    window close."""
    head_start = code.index('TelemetryWriter_Init(&line_writer')
    return TELEMETRY_FIELD.findall(code, head_start) + TELEMETRY_FIELD.findall(code, 0, head_start)


def test_telemetry_fields_in_main_c_are_v5_in_order():
    code = without_comments(source(MAIN_C))
    assert tuple(key for _, key in telemetry_fields_in_line_order(code)) == serial_logger.TELEMETRY_FIELDS_V5
    # And the parts follow each other in that order:
    # IMU -> motor -> steering + battery -> speed + send.
    assert code.index('line_step = LINE_MOTOR_PART_NEXT;') < code.index('"cmd_pct"')
    assert code.index('line_step = LINE_STEERING_PART_NEXT;') < code.index('"rst"')
    assert code.index('"s_wd"') < code.index('"vbat"') < code.index('"vdda"')
    assert code.index('"vdda"') < code.index('else if (line_step == LINE_SPEED_PART_NEXT)') < code.index('"vset"')
    # Each later step hands on to the next, and only the last goes back to idle
    # (a step that went idle early would never send the line).
    for step, following in (('LINE_IMU_PART_NEXT', 'LINE_MOTOR_PART_NEXT'),
                            ('LINE_MOTOR_PART_NEXT', 'LINE_STEERING_PART_NEXT'),
                            ('LINE_STEERING_PART_NEXT', 'LINE_SPEED_PART_NEXT'),
                            ('LINE_SPEED_PART_NEXT', 'LINE_IDLE')):
        assert re.search(rf'if \(line_step == {step}\)\s*\{{\s*line_step = {following};', code), step
    # The speed fields are the 5th step: the line ends and is sent there, once.
    end = code.index('TelemetryWriter_AppendText(&line_writer, "\\r\\n")')
    assert code.count('TelemetryWriter_AppendText(&line_writer, "\\r\\n")') == 1
    assert code.index('"mfault"') < end < code.index('(uint8_t *)telemetry_line, (uint16_t)line_writer.length')
    assert code.count('(uint8_t *)telemetry_line, (uint16_t)line_writer.length') == 1


def test_telemetry_line_buffer_holds_the_worst_case_line():
    # Every field at its full 32-bit width, whatever it can really reach: an
    # unsigned value up to 10 digits, a signed one up to 11 (with the '-'),
    # each " key=" before it (no space before the first), and "\r\n". The
    # buffer also keeps the NUL. A bound that needs no knowledge of the values.
    code = without_comments(source(MAIN_C))
    fields = telemetry_fields_in_line_order(code)
    width = lambda part: sum(1 + len(key) + 1 + (11 if kind == 'Signed' else 10) for kind, key in part)
    worst = width(fields) - 1 + 2
    assert width(fields[len(serial_logger.TELEMETRY_FIELDS_V4):]) == 135   # the v5 speed fields
    assert worst == 956
    size = int(re.search(r'static char telemetry_line\[(\d+)\];', code).group(1))
    assert size == 1024 and worst + 1 <= size
    raw = source(MAIN_C)
    assert 'full 32-bit width is 956 bytes' in raw and '(worst case 786 of 1024 bytes' in raw


# --- Group D wiring: encoder -> correction -> estimator -> speed control -------------

def test_closed_loop_wiring_order_in_the_main_loop():
    loop = main_loop_code()
    # Commands first, then speed control and the stall check on the previous
    # ms's estimate, then the motor state machine (so a new target or a latch
    # acts in this same ms).
    order = [loop.index(call) for call in (
        'Commands_ProcessReceived(&command_line, &motor_controller, &speed_controller,',
        'SpeedControl_Update(&speed_controller, &motor_controller, &speed_estimator, now_ms);',
        'StallDetect_Update(&stall_detector, &motor_controller, &speed_controller, &speed_estimator, now_ms);',
        'MotorController_Update(&motor_controller, now_ms);')]
    assert order == sorted(order), order
    # The encoder read, then both unwrappers (raw first, then corrected), then
    # the estimate from the corrected count, before the magnitude read.
    order = [loop.index(call) for call in (
        'AS5600_ReadRawAngle(&hi2c1, &raw_angle)',
        'AS5600_UnwrapperUpdate(&wheel_encoder, raw_angle);',
        'AS5600_UnwrapperUpdate(&corrected_encoder, AngleCorrection_Apply(raw_angle));',
        'SpeedEstimator_Update(&speed_estimator, &estimator_input);',
        'AS5600_ReadMagnitude(')]
    assert order == sorted(order), order
    estimate = loop[order[3] - 400:order[3]]
    for field in ('.now_ms = now_ms', '.count = corrected_encoder.continuous_count', '.read_ok = angle_read_ok',
                  '.magnet_detected = magnet_status.magnet_detected'):
        assert field in estimate, field
    # Each runs exactly once per iteration.
    for call in ('SpeedControl_Update(', 'StallDetect_Update(', 'SpeedEstimator_Update(', 'AngleCorrection_Apply('):
        assert loop.count(call) == 1, call


def test_count_and_d100_are_corrected_while_raw_and_the_capture_stay_raw():
    code = without_comments(source(MAIN_C))
    assert '"count",       corrected_encoder.continuous_count' in code
    assert '(uint32_t)corrected_encoder.continuous_count - (uint32_t)count_at_last_report' in code
    assert 'count_at_last_report = corrected_encoder.continuous_count;' in code
    assert '"raw",         latest_raw_angle' in code
    assert 'latest_raw_angle = raw_angle;' in code
    assert '.count = wheel_encoder.continuous_count' in code   # SpeedCapture: the raw angle's count
    assert 'wheel_encoder.continuous_count' not in code.replace('.count = wheel_encoder.continuous_count', '')


def test_main_dispatches_with_the_speed_controller():
    receive = without_comments(function_body(source(MAIN_C), COMMANDS_PROCESS_RECEIVED))
    assert 'CommandDispatch_LineWithSpeed(line_result, &command, motor, steering, speed,' in receive
    assert 'CommandDispatch_Line(' not in without_comments(source(MAIN_C))


def test_speed_report_snapshot_is_taken_at_window_close():
    code = without_comments(source(MAIN_C))
    head = code[code.index('TelemetryWriter_Init(&line_writer'):code.index('line_step = LINE_IMU_PART_NEXT;')]
    for assignment in ('speed_report.setpoint_units = speed_controller.setpoint_units;',
                       'speed_report.estimate_units = speed_estimator.speed_units;',
                       'speed_report.state = (uint32_t)SpeedControl_State(&speed_controller);',
                       'speed_report.estimate_valid = speed_estimator.valid ? 1u : 0u;',
                       'speed_report.integrator_ticks = SpeedControl_IntegratorTicks(&speed_controller);',
                       'speed_report.stall_count = stall_detector.stall_count;',
                       'speed_report.enc_bad_count = speed_controller.enc_bad_count;',
                       'speed_report.fault_reason = (uint32_t)motor_controller.fault_reason;'):
        assert assignment in head, assignment


MOTOR_CONTROL_H = FIRMWARE / 'Core' / 'Inc' / 'motor_control.h'
SPEED_CONTROL_H = FIRMWARE / 'Core' / 'Inc' / 'speed_control.h'

# mfault and ctl codes as the telemetry reports them (tools decode them): append only.
MOTOR_FAULT_CODES = ('MOTOR_FAULT_NONE', 'MOTOR_FAULT_PERIOD_MISMATCH', 'MOTOR_FAULT_CAP_GUARD',
                     'MOTOR_FAULT_DIRECTION_GUARD', 'MOTOR_FAULT_LOW_BATTERY', 'MOTOR_FAULT_BATTERY_ADC',
                     'MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING', 'MOTOR_FAULT_REPEATED_STALL')
SPEED_CTL_CODES = ('SPEED_CTL_OPEN_LOOP', 'SPEED_CTL_RUNNING', 'SPEED_CTL_STOPPED', 'SPEED_CTL_GATE')


def enum_members(text, last_member):
    """The members of the enum that ends with last_member, in order (no explicit values after the first)."""
    code = without_comments(text)
    end = code.index(last_member)
    start = code.rindex('{', 0, end)
    body = code[start + 1:code.index('}', end)]
    return tuple(re.sub(r'\s*=\s*\d+', '', member).strip() for member in body.split(',') if member.strip())


def test_mfault_and_ctl_codes_are_pinned():
    assert enum_members(source(MOTOR_CONTROL_H), 'MOTOR_FAULT_REPEATED_STALL') == MOTOR_FAULT_CODES
    assert 'MOTOR_FAULT_NONE = 0' in without_comments(source(MOTOR_CONTROL_H))
    assert enum_members(source(SPEED_CONTROL_H), 'SPEED_CTL_GATE') == SPEED_CTL_CODES
    code = without_comments(source(SPEED_CONTROL_H))
    for value, name in enumerate(SPEED_CTL_CODES):
        assert re.search(rf'\b{name}\s*=\s*{value}\b', code), name


def test_console_decodes_every_mfault_and_ctl_code():
    # The console's names, code by code (MOTOR_FAULT_CODES / SPEED_CTL_CODES
    # are pinned against the firmware enums above).
    import motor_console
    console_names = {
        'MOTOR_FAULT_NONE': 'none', 'MOTOR_FAULT_PERIOD_MISMATCH': 'PWM PERIOD', 'MOTOR_FAULT_CAP_GUARD': 'CAP GUARD',
        'MOTOR_FAULT_DIRECTION_GUARD': 'DIRECTION GUARD', 'MOTOR_FAULT_LOW_BATTERY': 'LOW BATTERY',
        'MOTOR_FAULT_BATTERY_ADC': 'BATTERY ADC', 'MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING': 'BATTERY REMOVED WHILE DRIVING',
        'MOTOR_FAULT_REPEATED_STALL': 'REPEATED STALL',
    }
    assert motor_console.MOTOR_FAULT_NAMES == tuple(console_names[code] for code in MOTOR_FAULT_CODES)
    ctl_names = {'SPEED_CTL_OPEN_LOOP': 'open', 'SPEED_CTL_RUNNING': 'run', 'SPEED_CTL_STOPPED': 'stop',
                 'SPEED_CTL_GATE': 'GATE'}
    assert motor_console.SPEED_CTL_NAMES == tuple(ctl_names[code] for code in SPEED_CTL_CODES)


def test_profile_ctl_and_est_time_exactly_the_closed_loop_code():
    loop = main_loop_code()
    ctl = loop[loop.index('profile_ctl_start_cycles = DWT->CYCCNT;'):loop.index('profile_ctl_cycles = DWT->CYCCNT')]
    assert re.findall(r'\b(\w+_Update)\(', ctl) == ['SpeedControl_Update', 'StallDetect_Update']
    # est: from the last restart of its start mark (after the raw unwrap) to its end.
    est_end = loop.index('profile_est_cycles = DWT->CYCCNT')
    est = loop[loop.rindex('profile_est_start_cycles = DWT->CYCCNT;', 0, est_end):est_end]
    assert 'AngleCorrection_Apply(raw_angle)' in est and 'SpeedEstimator_Update(' in est
    assert 'AS5600_Read' not in est and 'wheel_encoder' not in est
    # The 5th report step has its own kind, and the line is sent well before the next window.
    code = without_comments(source(MAIN_C))
    assert 'profile_was_report_speed = true;' in code and 'PROFILE_REPORT_SPEED' in code
    assert define_value(code, 'PROFILE_SEND_DEADLINE_MS') < define_value(code, 'REPORT_PERIOD_MS') - 45


def test_profile_fields_in_main_c_match_the_summary_tool():
    code = without_comments(source(MAIN_C))
    fields = re.findall(r'TelemetryWriter_AppendUnsignedField\(writer,\s*"(\w+)"', code)
    assert tuple(fields) == summarize_loop_profile.PROFILE_KEYS


# --- Pi link (link_rx / link_protocol / link_codec / tx_queue in main.c) ---------

LINK_RX_C = FIRMWARE / 'Core' / 'Src' / 'link_rx.c'
LINK_PROTOCOL_H = FIRMWARE / 'Core' / 'Inc' / 'link_protocol.h'
CMAKELISTS = FIRMWARE / 'CMakeLists.txt'


def test_link_sources_are_in_the_firmware_build():
    text = source(CMAKELISTS)
    for name in ('link_codec.c', 'link_protocol.c', 'link_rx.c', 'tx_queue.c'):
        assert f'    Core/Src/{name}\n' in text, name


def test_only_a_validated_drive_feeds_the_motor_watchdog_over_the_link():
    # MotorController_AcceptDriveCommand is called from link_rx.c only (never
    # main.c), once, in the DRIVE case, after the mode check and after the
    # payload checks returned OK.
    for path in sorted((FIRMWARE / 'Core' / 'Src').glob('*.c')):
        if path.name in ('link_rx.c', 'motor_control.c'):
            continue
        assert 'MotorController_AcceptDriveCommand(' not in without_comments(source(path)), path.name
    code = without_comments(source(LINK_RX_C))
    assert code.count('MotorController_AcceptDriveCommand(') == 1
    drive_case = code[code.index('case LINK_MSG_DRIVE:'):code.index('case LINK_MSG_ESTOP:')]
    call = drive_case.index('MotorController_AcceptDriveCommand(targets->motor, drive.arm, now_ms);')
    assert drive_case.index('if (link->mode != LINK_MODE_BINARY)') < call
    assert drive_case.index('result = LinkProtocol_ParseDrive(frame, &drive);') < call
    assert drive_case.index('if (result != LINK_PARSE_OK)') < call
    # HELLO and ESTOP latch and feed nothing.
    for case, end in (('case LINK_MSG_HELLO:', 'case LINK_MSG_DRIVE:'), ('case LINK_MSG_ESTOP:', 'default:')):
        body = code[code.index(case):code.index(end)]
        assert 'MotorController_Latch(targets->motor,' in body and 'Accept' not in body, case


def test_link_frames_never_share_a_ms_with_health_battery_or_each_other():
    # STATE (every 10 ms) and STATUS (every 100 ms) are packed in their own
    # iterations: never on health (50), a battery sample (t % 10 == 5) or
    # each other; STATUS also never on the text line's head or its 4 steps.
    code = source(MAIN_C)
    state_period, state_phase = define_value(code, 'LINK_STATE_PERIOD_MS'), define_value(code, 'LINK_STATE_PHASE_MS')
    status_period, status_phase = (define_value(code, 'LINK_STATUS_PERIOD_MS'),
                                   define_value(code, 'LINK_STATUS_PHASE_MS'))
    assert (state_period, state_phase, status_period, status_phase) == (10, 3, 100, 8)
    report, health = define_value(code, 'REPORT_PERIOD_MS'), define_value(code, 'HEALTH_PHASE_MS')
    battery_period, battery_phase = define_value(code, 'BATTERY_PERIOD_MS'), define_value(code, 'BATTERY_PHASE_MS')
    horizon = range(10 * report)
    state = {t for t in horizon if t % state_period == state_phase}
    status = {t for t in horizon if t % status_period == status_phase}
    battery = {t for t in horizon if t % battery_period == battery_phase}
    health_ms = {t for t in horizon if t % report == health}
    text_steps = {t for t in horizon if t % report <= 4}
    assert not state & (battery | health_ms | status)
    assert not status & (battery | health_ms | text_steps)
    assert ('PeriodicSchedule_Init(&link_state_schedule, LINK_STATE_PERIOD_MS, LINK_STATE_PHASE_MS, '
            'next_deadline_ms);') in code
    assert ('PeriodicSchedule_Init(&link_status_schedule, LINK_STATUS_PERIOD_MS, LINK_STATUS_PHASE_MS, '
            'next_deadline_ms);') in code


def test_text_line_only_in_text_mode_and_frames_only_in_binary_mode():
    loop = main_loop_code()
    # The text line: started only in TEXT mode; a half-built one is dropped in binary mode.
    assert ('if (PeriodicSchedule_IsDue(&report_schedule, scheduled_time_ms) && link_rx.mode == LINK_MODE_TEXT)'
            in loop)
    drop = re.search(r'if \(link_rx\.mode == LINK_MODE_BINARY\)\s*\{\s*line_step = LINE_IDLE;\s*\}', loop)
    assert drop and drop.start() < loop.index('if (line_step == LINE_IMU_PART_NEXT)')
    # Frames: STATE and STATUS only in binary mode; HELLO_ACK only after a HELLO.
    state = loop[loop.index('PeriodicSchedule_IsDue(&link_state_schedule'):loop.index('link_period = (LinkPeriod){0};')]
    assert 'if (link_rx.mode == LINK_MODE_BINARY)' in state and 'LinkTx_SendState(' in state
    assert ('if (PeriodicSchedule_IsDue(&link_status_schedule, scheduled_time_ms) && link_rx.mode == LINK_MODE_BINARY)'
            in loop)
    assert loop.count('LinkTx_SendState(') == 1 and loop.count('LinkTx_SendStatus(') == 1
    hello = loop[loop.index('if (hello_received)'):]
    assert hello.index('LinkTx_SendHelloAck(') < hello.index('}')
    assert loop.count('LinkTx_SendHelloAck(') == 1


def test_link_receive_wiring_in_commands_process_received():
    receive = without_comments(function_body(source(MAIN_C), COMMANDS_PROCESS_RECEIVED))
    # Every byte goes to the link first; text only after it says so.
    feed = receive.index('LinkRx_FeedByte(&link_rx, received_byte, &link_targets, now_ms);')
    assert feed < receive.index('CommandLine_FeedByte(command_line, received_byte, &command);')
    # A UART error: the frame if inside one, else the text line (as before).
    assert re.search(r'if \(!LinkRx_Poison\(&link_rx\)\)\s*\{\s*CommandLine_DiscardCurrent\(command_line\);', receive)
    # A frame start abandons the partial text line.
    assert re.search(r'if \(link_event == LINK_RX_FRAME_START\)\s*\{\s*CommandLine_Init\(command_line\);\s*continue;',
                     receive)
    assert re.search(r'if \(link_event != LINK_RX_TEXT_BYTE\)\s*\{\s*continue;', receive)
    # Every finished text line passes LinkRx_TextCommandAllowed (link_rx.c, host-tested)
    # before anything can act on it: the c and e blocks and the dispatcher all come after
    # the gate, and the gate is asked with the real switches.
    gate = receive.index('if (!LinkRx_TextCommandAllowed(link_rx.mode, letter, (BENCH_TEXT_COMMANDS != 0), '
                         '(LINK_TEST_BUILD != 0)))')
    assert receive.index('CommandLine_FeedByte(command_line, received_byte, &command);') < gate
    assert "char letter = (line_result == COMMAND_LINE_VALID) ? command.letter : '\\0';" in receive
    for acting in ('CommandDispatch_LineWithSpeed(', 'SpeedCapture_Command(', 'EncoderFault_Command('):
        assert gate < receive.index(acting), acting
    assert receive.count('LinkRx_TextCommandAllowed(') == 1
    # A line the gate refuses: counted in cmd_bad in TEXT mode only (binary mode counts it
    # in rx_text_ignored already), then nothing else.
    refused = receive[gate:receive.index('continue;', receive.index('continue;', gate) + 1) + len('continue;')]
    assert re.search(r'if \(link_rx\.mode == LINK_MODE_TEXT\)\s*\{\s*MotorController_CountRejected\(motor\);\s*\}'
                     r'\s*continue;', refused)
    # No other mode filter is left (the gate replaced the old binary-mode e/c/h filter).
    assert 'command.letter == \'h\'' not in receive and 'LINK_TEXT_TEST_COMMANDS' not in receive
    code = without_comments(source(MAIN_C))
    assert ('#define LINK_TEST_BUILD  ((IWDG_HANG_TEST != 0) || (SPEED_CAPTURE != 0) || (ENCODER_FAULT_TEST != 0) || '
            '\\\n                          (LOOP_TIMING_PROFILE != 0))') in code


def test_bench_text_commands_switch_is_off_and_pinned():
    # The truck build's text mode only waits for a HELLO: BENCH_TEXT_COMMANDS is
    # committed as 0, with a #warning when 1, like the other build switches.
    text = without_comments(source(MAIN_C))
    assert re.search(r'^#define BENCH_TEXT_COMMANDS\s+0\b', text, re.MULTILINE), \
        'BENCH_TEXT_COMMANDS must be committed as 0'
    guarded = text[text.index('#if BENCH_TEXT_COMMANDS'):]
    assert guarded.index('#warning') < guarded.index('#endif')
    # Defined before the gate's test-build list and the HELLO_ACK flags that use it.
    assert text.index('#define BENCH_TEXT_COMMANDS') < text.index('#define LINK_BUILD_FLAGS')


def test_tx_queue_is_popped_only_by_the_transmit_complete_callback():
    code = without_comments(source(MAIN_C))
    callback = function_body(code, 'void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)\n{')
    assert code.count('TxQueue_Pop(') == 1 and 'TxQueue_Pop(&link_tx_queue);' in callback
    assert callback.index('if (link_tx_queue_sending)') < callback.index('TxQueue_Pop(&link_tx_queue);')
    assert 'HAL_IWDG_Refresh' not in callback
    # Defined in USER CODE 4 (so CubeMX keeps it).
    assert 'void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)' in block_between(
        source(MAIN_C), '/* USER CODE BEGIN 4 */', '/* USER CODE END 4 */')
    # The main loop pushes only through LinkTx_Send, and sets "sending" only with interrupts masked.
    assert code.count('TxQueue_Push(') == 1
    assert 'TxQueue_Push(&link_tx_queue,' in function_body(code, 'static void LinkTx_Send(uint8_t type, uint8_t *seq, const uint8_t *payload, size_t length)' + chr(10) + '{')
    kick = function_body(code, 'static void LinkTx_Kick(void)\n{')
    masked = kick[kick.index('__disable_irq();'):kick.index('__set_PRIMASK(interrupt_mask);')]
    assert 'link_tx_queue_sending = true;' in masked
    outside = code.replace(kick, '').replace(callback, '')
    assert re.findall(r'link_tx_queue_sending\s*=\s*true', outside) == []
    assert re.findall(r'link_tx_queue_sending = false;', outside) == ['link_tx_queue_sending = false;']  # boot init


def test_hello_ack_build_flags_cover_every_test_switch():
    code = without_comments(source(MAIN_C))
    flags = code[code.index('#define LINK_BUILD_FLAGS'):]
    flags = flags[:flags.index('\n\n')]
    for switch, bit in (('IWDG_HANG_TEST', 'LINK_BUILD_IWDG_HANG_TEST'), ('SPEED_CAPTURE', 'LINK_BUILD_SPEED_CAPTURE'),
                        ('ENCODER_FAULT_TEST', 'LINK_BUILD_ENCODER_FAULT_TEST'),
                        ('LOOP_TIMING_PROFILE', 'LINK_BUILD_LOOP_TIMING_PROFILE'),
                        ('BENCH_TEXT_COMMANDS', 'LINK_BUILD_BENCH_TEXT_COMMANDS')):
        assert f'(({switch} != 0) ? {bit} : 0u)' in flags, switch
    assert '.build_flags = LINK_BUILD_FLAGS' in code
    protocol = without_comments(source(LINK_PROTOCOL_H))
    assert re.search(r'#define LINK_BUILD_BENCH_TEXT_COMMANDS\s+0x10u', protocol)


def test_every_status_counter_is_filled_exactly_once():
    header = without_comments(source(LINK_PROTOCOL_H))
    enum = header[header.index('LINK_COUNT_WATCHDOG_TRIPS = 0'):header.index('} LinkStatusCounter;')]
    members = re.findall(r'\b(LINK_COUNT_\w+)\b', enum)
    assert len(members) == 20
    loop = main_loop_code()
    for member in members:
        assert loop.count(f'status->counters[{member}] =') == 1, member


def test_state_imu_and_encoder_accumulate_every_ms_and_restart_every_period():
    loop = main_loop_code()
    for line in ('link_period.gyro_sum[0] += imu_sample.gyro_x;', 'link_period.accel_sum[2] += imu_sample.accel_z;',
                 'link_period.imu_read_failed = true;', 'link_period.estimate_invalid = true;',
                 'link_period.angle_read_failed = true;', 'link_odometer_epoch++;'):
        assert loop.count(line) == 1, line
    # The epoch counts valid -> invalid transitions of the estimate, after its update.
    assert loop.index('SpeedEstimator_Update(&speed_estimator, &estimator_input);') < loop.index('link_odometer_epoch++;')
    assert 'if (link_estimate_was_valid)' in loop and 'link_estimate_was_valid = speed_estimator.valid;' in loop
    # Restarted at every STATE mark in either mode (after the binary-only part).
    state = loop[loop.index('PeriodicSchedule_IsDue(&link_state_schedule'):]
    assert state.index('link_period = (LinkPeriod){0};') > state.index('LinkTx_SendState(')
