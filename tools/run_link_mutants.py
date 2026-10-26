"""Mutation testing for the Pi link: proves the tests would notice the bugs
that matter most, by planting each one and requiring a test to fail.

Each mutant is one textual change (old -> new, old must occur exactly once)
in a copy of the sources, in a temporary directory: the working tree is never
touched. A C mutant is built and run with the same flags as the host tests
(GCC, AddressSanitizer + UBSan, -fno-sanitize-recover=all) in every suite
that compiles the mutated file; a Python mutant runs its tools/test_<name>.py.
main.c has no host suite: its mutants run the source pins
(tools/test_firmware_sources.py), which proves a pin notices the change; that
is a weaker kill than a behaviour test, and the report says which is which.
A mutant is KILLED when a suite fails, crashes or times out, SURVIVED when
every suite passes, and INVALID when it doesn't compile (a broken mutant,
not a result). The unmutated sources must pass first.

Run from the repo root, in WSL (needs gcc and python3 with pytest):
    python3 tools/run_link_mutants.py            # all mutants
    python3 tools/run_link_mutants.py link_rx    # only ids containing "link_rx"

Exit code 0 only if every mutant was killed. tools/test_run_link_mutants.py
checks that each mutant's text still occurs exactly once in its file.
"""
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = REPOSITORY_ROOT / 'firmware'
FLAGS = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
         '-fno-sanitize-recover=all', '-ICore/Inc']
SUITE_TIMEOUT_SECONDS = 300

LINK_SOURCES = ['Core/Src/link_rx.c', 'Core/Src/link_protocol.c', 'Core/Src/link_codec.c',
                'Core/Src/motor_control.c', 'Core/Src/speed_control.c', 'Core/Src/steering_control.c']
# The host suites that cover the files mutated here (the README's list for the others).
SUITES = {
    'test_link_codec': ['tests/test_link_codec.c', 'Core/Src/link_codec.c'],
    'test_link_protocol': ['tests/test_link_protocol.c', 'Core/Src/link_protocol.c', 'Core/Src/link_codec.c', '-lm'],
    'test_link_rx': ['tests/test_link_rx.c'] + LINK_SOURCES,
    'test_link_fuzz': ['tests/test_link_fuzz.c'] + LINK_SOURCES,
    'test_tx_queue': ['tests/test_tx_queue.c', 'Core/Src/tx_queue.c'],
    'test_motor_control': ['tests/test_motor_control.c', 'Core/Src/motor_control.c', 'Core/Src/byte_ring.c'],
    'test_stall_detect': ['tests/test_stall_detect.c', 'Core/Src/stall_detect.c', 'Core/Src/motor_control.c',
                          'Core/Src/speed_control.c'],
    'test_saturation_stall': ['tests/test_saturation_stall.c', 'Core/Src/stall_detect.c', 'Core/Src/speed_control.c',
                              'Core/Src/speed_estimator.c', 'Core/Src/motor_control.c', '-lm'],
    'test_speed_control': ['tests/test_speed_control.c', 'Core/Src/speed_control.c', 'Core/Src/motor_control.c',
                           'Core/Src/speed_estimator.c', '-lm'],
}
# Headers: the suites whose sources include them.
HEADER_SUITES = {
    'Core/Inc/link_codec.h': ['test_link_codec', 'test_link_protocol', 'test_link_rx', 'test_link_fuzz'],
}


@dataclass(frozen=True)
class Mutant:
    id: str
    path: str        # relative to the repo root
    old: str
    new: str
    bug: str


MUTANTS = [
    # --- link_codec: CRC, COBS, frames
    Mutant('codec_crc_init_zero', 'firmware/Core/Inc/link_codec.h',
           '#define LINK_CRC_INITIAL        0xFFFFu', '#define LINK_CRC_INITIAL        0x0000u',
           'CRC-16/XMODEM instead of CCITT-FALSE (initial value 0)'),
    Mutant('codec_skip_crc_check', 'firmware/Core/Src/link_codec.c',
           'if (LinkCrc16(body, covered) != received)', 'if (0 && LinkCrc16(body, covered) != received)',
           'the CRC is never compared'),
    Mutant('codec_crc_byte_order', 'firmware/Core/Src/link_codec.c',
           'uint16_t received = (uint16_t)(body[covered] | ((uint16_t)body[covered + 1u] << 8));',
           'uint16_t received = (uint16_t)(body[covered + 1u] | ((uint16_t)body[covered] << 8));',
           'the CRC read big-endian'),
    Mutant('codec_encode_extra_block_after_254', 'firmware/Core/Src/link_codec.c',
           "if (in[i] == 0u || (code == 0xFFu && i + 1u < length))", "if (in[i] == 0u || code == 0xFFu)",
           'COBS: an empty block after a full one at the end'),
    Mutant('codec_decode_zero_after_full_block', 'firmware/Core/Src/link_codec.c',
           'if (code != 0xFFu && read < length)', 'if (read < length)',
           'COBS: a 0x00 inserted after a 254-byte block'),
    Mutant('codec_decode_accepts_zero_data', 'firmware/Core/Src/link_codec.c',
           'if (read >= length || in[read] == 0u || write >= out_capacity)',
           'if (read >= length || write >= out_capacity)',
           'COBS: a 0x00 inside a block accepted'),
    Mutant('codec_too_short_off_by_two', 'firmware/Core/Src/link_codec.c',
           'if (body_length < LINK_BODY_OVERHEAD)', 'if (body_length < 2u)',
           'a body without room for the CRC accepted'),
    Mutant('codec_too_long_unchecked', 'firmware/Core/Src/link_codec.c',
           'if (length > LINK_MAX_COBS)\n  {\n    return LINK_FRAME_TOO_LONG;',
           'if (length > LINK_MAX_COBS + 64u)\n  {\n    return LINK_FRAME_TOO_LONG;',
           'pieces longer than the maximum not refused'),
    # --- link_protocol: checks, conversion, packing
    Mutant('protocol_drive_length_unchecked', 'firmware/Core/Src/link_protocol.c',
           'if (frame->payload_length != LINK_DRIVE_LENGTH)', 'if (frame->payload_length < LINK_DRIVE_LENGTH)',
           'a DRIVE with extra bytes accepted'),
    Mutant('protocol_reserved_flags_accepted', 'firmware/Core/Src/link_protocol.c',
           'if ((flags & LINK_DRIVE_RESERVED) != 0u)', 'if ((flags & 0x80u) != 0u)',
           'reserved DRIVE flag bits 2-6 accepted'),
    Mutant('protocol_speed_range_off_by_one', 'firmware/Core/Src/link_protocol.c',
           'if (speed_cps > LINK_MAX_SPEED_CPS || speed_cps < -LINK_MAX_SPEED_CPS',
           'if (speed_cps > LINK_MAX_SPEED_CPS + 1 || speed_cps < -LINK_MAX_SPEED_CPS',
           '5001 cps accepted'),
    Mutant('protocol_speed_range_too_strict', 'firmware/Core/Src/link_protocol.c',
           'if (speed_cps > LINK_MAX_SPEED_CPS || speed_cps < -LINK_MAX_SPEED_CPS',
           'if (speed_cps >= LINK_MAX_SPEED_CPS || speed_cps < -LINK_MAX_SPEED_CPS',
           '5000 cps rejected'),
    Mutant('protocol_arm_honoured_with_speed', 'firmware/Core/Src/link_protocol.c',
           'if (arm && speed_cps != 0)', 'if (false && arm && speed_cps != 0)',
           'arm accepted together with a non-zero speed'),
    Mutant('protocol_round_ties_away_from_zero', 'firmware/Core/Src/link_protocol.c',
           '(magnitude + LINK_CPS_PER_UNIT / 2 - 1) / LINK_CPS_PER_UNIT',
           '(magnitude + LINK_CPS_PER_UNIT / 2) / LINK_CPS_PER_UNIT',
           'cps rounding: ties away from zero'),
    Mutant('protocol_no_minimum_setpoint', 'firmware/Core/Src/link_protocol.c',
           'if (magnitude < LINK_MIN_SPEED_CPS)', 'if (magnitude < 0)',
           'setpoints below 1000 cps passed on'),
    Mutant('protocol_min_check_after_rounding', 'firmware/Core/Src/link_protocol.c',
           '  if (magnitude < LINK_MIN_SPEED_CPS)\n  {\n    return 0;\n  }\n'
           '  int32_t units = (magnitude + LINK_CPS_PER_UNIT / 2 - 1) / LINK_CPS_PER_UNIT;\n',
           '  int32_t units = (magnitude + LINK_CPS_PER_UNIT / 2 - 1) / LINK_CPS_PER_UNIT;\n'
           '  if (units < SPEED_MIN_SETPOINT_UNITS)\n  {\n    return 0;\n  }\n',
           'the minimum checked after rounding (976..999 cps -> 20 units)'),
    Mutant('protocol_hello_version_unchecked', 'firmware/Core/Src/link_protocol.c',
           'if (frame->payload[0] != LINK_PROTOCOL_VERSION)', 'if (false)',
           'HELLO of any version accepted'),
    Mutant('protocol_u16_wraps', 'firmware/Core/Src/link_protocol.c',
           'uint32_t v = SaturateUnsigned(value, 0xFFFFu);', 'uint32_t v = value & 0xFFFFu;',
           'u16 fields wrap instead of saturating'),
    # --- link_rx: modes, actions, watchdogs, counting
    # (DRIVE's own mode check is defence in depth: in TEXT mode a DRIVE, 9 COBS
    # bytes, is given up by the stray-0x00 limit before it can reach it, so
    # removing that check alone is an equivalent mutant. ESTOP, 5 bytes, does
    # reach its check.)
    Mutant('rx_estop_in_text_mode', 'firmware/Core/Src/link_rx.c',
           '    case LINK_MSG_ESTOP:\n      if (link->mode != LINK_MODE_BINARY)\n      {\n'
           '        return LinkRx_Reject(link, LINK_PARSE_NOT_BINARY_MODE);\n      }\n',
           '    case LINK_MSG_ESTOP:\n',
           'ESTOP acted on before HELLO'),
    Mutant('rx_text_mode_stray_zero_swallows_text', 'firmware/Core/Src/link_rx.c',
           'if (link->mode == LINK_MODE_TEXT && link->length >= LINK_TEXT_MODE_MAX_COBS)',
           'if (link->mode == LINK_MODE_TEXT && link->length >= LINK_MAX_COBS + 1u)',
           'a stray 0x00 in TEXT mode swallows the text after it'),
    Mutant('rx_feeds_watchdog_before_checks', 'firmware/Core/Src/link_rx.c',
           '      LinkDrive drive;\n      result = LinkProtocol_ParseDrive(frame, &drive);',
           '      LinkDrive drive;\n      MotorController_AcceptDriveCommand(targets->motor, false, now_ms);\n'
           '      result = LinkProtocol_ParseDrive(frame, &drive);',
           'a DRIVE feeds the watchdog before it is checked'),
    Mutant('rx_drive0_rearms_without_arm', 'firmware/Core/Src/link_rx.c',
           'MotorController_AcceptDriveCommand(targets->motor, drive.arm, now_ms);',
           'MotorController_AcceptDriveCommand(targets->motor, drive.arm || drive.speed_cps == 0, now_ms);',
           'DRIVE 0 re-arms without the arm bit'),
    Mutant('rx_estop_feeds_watchdog', 'firmware/Core/Src/link_rx.c',
           '      MotorController_Latch(targets->motor, MOTOR_LATCH_ESTOP);',
           '      MotorController_AcceptDriveCommand(targets->motor, false, now_ms);\n'
           '      MotorController_Latch(targets->motor, MOTOR_LATCH_ESTOP);',
           'ESTOP feeds the motor watchdog'),
    Mutant('rx_estop_does_not_latch', 'firmware/Core/Src/link_rx.c',
           '      MotorController_Latch(targets->motor, MOTOR_LATCH_ESTOP);', '',
           'ESTOP does nothing'),
    Mutant('rx_hello_in_binary_does_not_latch', 'firmware/Core/Src/link_rx.c',
           '      link->mode = LINK_MODE_BINARY;\n      MotorController_Latch(targets->motor, MOTOR_LATCH_LINK_HELLO);',
           '      if (link->mode != LINK_MODE_BINARY)\n      {\n'
           '        MotorController_Latch(targets->motor, MOTOR_LATCH_LINK_HELLO);\n      }\n'
           '      link->mode = LINK_MODE_BINARY;',
           'a HELLO in binary mode (bridge restart) does not latch'),
    Mutant('rx_steer_valid_ignored', 'firmware/Core/Src/link_rx.c',
           'if (drive.steer_valid)', 'if (true)',
           'every DRIVE feeds the steering'),
    Mutant('rx_poison_ignored', 'firmware/Core/Src/link_rx.c',
           'if (overlong || poisoned)', 'if (overlong || (poisoned && false))',
           'a frame hit by a UART error is acted on'),
    Mutant('rx_seq_wrap_is_a_gap', 'firmware/Core/Src/link_rx.c',
           'else if (seq != (uint8_t)(link->last_seq + 1u))', 'else if (seq != link->last_seq + 1u)',
           'seq 255 -> 0 counted as a gap'),
    Mutant('rx_duplicates_not_counted', 'firmware/Core/Src/link_rx.c',
           '      link->duplicate_count++;', '',
           'duplicate seq not counted'),
    Mutant('rx_text_in_binary_not_counted', 'firmware/Core/Src/link_rx.c',
           '      link->text_ignored_count++;', '',
           'text in binary mode not counted'),
    Mutant('rx_unknown_type_acted', 'firmware/Core/Src/link_rx.c',
           '    default:\n      return LinkRx_Reject(link, LINK_PARSE_UNKNOWN_TYPE);',
           '    default:\n      MotorController_AcceptDriveCommand(targets->motor, false, now_ms);\n'
           '      return LinkRx_Reject(link, LINK_PARSE_UNKNOWN_TYPE);',
           'an unknown type feeds the watchdog'),
    Mutant('rx_gate_text_always_open', 'firmware/Core/Src/link_rx.c',
           '    return bench_text_commands || test_build;', '    return bench_text_commands || test_build || true;',
           'the truck build acts on text commands'),
    Mutant('rx_gate_ignores_test_build', 'firmware/Core/Src/link_rx.c',
           '    return bench_text_commands || test_build;', '    return bench_text_commands;',
           'a test build loses its text commands (capture m steps)'),
    Mutant('rx_gate_binary_allows_m', 'firmware/Core/Src/link_rx.c',
           "return test_build && (letter == 'e' || letter == 'c' || letter == 'h');",
           "return test_build && (letter == 'e' || letter == 'c' || letter == 'h' || letter == 'm');",
           'm acts in binary mode in a test build'),
    Mutant('rx_gate_binary_without_test_build', 'firmware/Core/Src/link_rx.c',
           "return test_build && (letter == 'e' || letter == 'c' || letter == 'h');",
           "return (letter == 'e' || letter == 'c' || letter == 'h');",
           'e, c, h accepted in binary mode in a normal build'),
    # --- tx_queue
    Mutant('txq_full_off_by_one', 'firmware/Core/Src/tx_queue.c',
           'write_count - queue->read_count >= TX_QUEUE_SLOTS', 'write_count - queue->read_count > TX_QUEUE_SLOTS',
           'a fifth frame overwrites the one being sent'),
    Mutant('txq_pop_on_empty', 'firmware/Core/Src/tx_queue.c',
           '  if (read_count == queue->write_count)\n  {\n    return;\n  }\n  queue->read_count',
           '  queue->read_count',
           'pop on an empty queue runs ahead of the producer'),
    Mutant('txq_long_frames_accepted', 'firmware/Core/Src/tx_queue.c',
           'length > TX_QUEUE_SLOT_BYTES ||', 'length > TX_QUEUE_SLOT_BYTES + 1u ||',
           'a 129-byte frame accepted into a 128-byte slot'),
    # --- group 1: motor latch reason, explicit arm, stall cause
    Mutant('motor_rearm_keeps_reason', 'firmware/Core/Src/motor_control.c',
           '      controller->state = MOTOR_STATE_OFF;\n      controller->latch_reason = MOTOR_LATCH_NONE;\n'
           '      controller->target_ticks = 0;\n    }\n    return;',
           '      controller->state = MOTOR_STATE_OFF;\n      controller->target_ticks = 0;\n    }\n    return;',
           '"m 0" re-arms but the latch reason stays'),
    Mutant('motor_drive_does_not_feed', 'firmware/Core/Src/motor_control.c',
           '  controller->last_command_ms = now_ms;   /* DRIVE feeds the watchdog, like "v" */', '  (void)now_ms;',
           'DRIVE does not feed the watchdog'),
    Mutant('motor_drive_rearms_without_arm', 'firmware/Core/Src/motor_control.c',
           'if (controller->state == MOTOR_STATE_LATCHED && arm)',
           'if (controller->state == MOTOR_STATE_LATCHED && (arm || !arm))',
           'any DRIVE re-arms'),
    Mutant('motor_watchdog_reason', 'firmware/Core/Src/motor_control.c',
           '    controller->latch_reason = MOTOR_LATCH_WATCHDOG;', '    controller->latch_reason = MOTOR_LATCH_BOOT;',
           'a watchdog trip reported as BOOT'),
    Mutant('stall_cause_swapped', 'firmware/Core/Src/stall_detect.c',
           'StallDetect_Stall(detector, motor, MOTOR_LATCH_STALL_SATURATION, now_ms);',
           'StallDetect_Stall(detector, motor, MOTOR_LATCH_STALL_LOCKED, now_ms);',
           'a saturation stall reported as a locked wheel'),
    Mutant('speed_invalid_reason', 'firmware/Core/Src/speed_control.c',
           'MotorController_Latch(motor, MOTOR_LATCH_ESTIMATE_INVALID);',
           'MotorController_Latch(motor, MOTOR_LATCH_STALL_LOCKED);',
           'an invalid-estimate latch reported as a stall'),
    # --- main.c wiring (killed by the source pins: tools/test_firmware_sources.py)
    Mutant('main_feeds_drive_watchdog', 'firmware/Core/Src/main.c',
           '      LinkTx_SendHelloAck(reset_cause_flags, imu_ok, imu_who_am_i, &link_ack_seq);\n',
           '      LinkTx_SendHelloAck(reset_cause_flags, imu_ok, imu_who_am_i, &link_ack_seq);\n'
           '      MotorController_AcceptDriveCommand(&motor_controller, false, now_ms);\n',
           'main.c feeds the motor watchdog outside a validated DRIVE'),
    Mutant('main_text_line_in_binary_mode', 'firmware/Core/Src/main.c',
           'if (PeriodicSchedule_IsDue(&report_schedule, scheduled_time_ms) && link_rx.mode == LINK_MODE_TEXT)',
           'if (PeriodicSchedule_IsDue(&report_schedule, scheduled_time_ms))',
           'the text line is built in binary mode too'),
    Mutant('main_status_on_a_battery_ms', 'firmware/Core/Src/main.c',
           '#define LINK_STATUS_PHASE_MS    8', '#define LINK_STATUS_PHASE_MS    5',
           'STATUS packed in a battery-sample iteration'),
    Mutant('main_pop_without_guard', 'firmware/Core/Src/main.c',
           '  if (link_tx_queue_sending)\n  {\n    TxQueue_Pop(&link_tx_queue);',
           '  if (true)\n  {\n    TxQueue_Pop(&link_tx_queue);',
           'the end of a text line pops a frame that was never sent'),
    Mutant('main_gate_bench_switch_bypassed', 'firmware/Core/Src/main.c',
           '(BENCH_TEXT_COMMANDS != 0), (LINK_TEST_BUILD != 0)))', 'true, (LINK_TEST_BUILD != 0)))',
           'the truck build acts on text m, v, s (gate asked with "bench" always on)'),
    Mutant('main_gated_lines_not_counted', 'firmware/Core/Src/main.c',
           '        MotorController_CountRejected(motor);  /* truck build: counted in cmd_bad, never acted on */\n', '',
           'a gated text line is not counted in cmd_bad'),
    Mutant('main_bench_text_committed_on', 'firmware/Core/Src/main.c',
           '#define BENCH_TEXT_COMMANDS   0', '#define BENCH_TEXT_COMMANDS   1',
           'BENCH_TEXT_COMMANDS committed as 1'),
    Mutant('main_bench_flag_missing', 'firmware/Core/Src/main.c',
           '((BENCH_TEXT_COMMANDS != 0) ? LINK_BUILD_BENCH_TEXT_COMMANDS : 0u))', '0u)',
           'HELLO_ACK hides a bench-text build'),
    Mutant('main_profiler_not_a_test_build', 'firmware/Core/Src/main.c',
           '(ENCODER_FAULT_TEST != 0) || \\\n                          (LOOP_TIMING_PROFILE != 0))',
           '(ENCODER_FAULT_TEST != 0))',
           'a profiler build loses its text commands'),
    Mutant('main_poison_always_on_text', 'firmware/Core/Src/main.c',
           '      if (!LinkRx_Poison(&link_rx))\n', '      if (true)\n',
           'a UART error inside a frame poisons the text line instead'),
    Mutant('main_frame_start_keeps_text_line', 'firmware/Core/Src/main.c',
           '      CommandLine_Init(command_line);\n      continue;\n', '      continue;\n',
           'a partial text line survives a frame start'),
    Mutant('main_status_counter_missing', 'firmware/Core/Src/main.c',
           '      status->counters[LINK_COUNT_RX_TEXT_IGNORED] = link_rx.text_ignored_count;\n', '',
           'a STATUS counter never filled'),
    Mutant('main_build_flag_missing', 'firmware/Core/Src/main.c',
           '((LOOP_TIMING_PROFILE != 0) ? LINK_BUILD_LOOP_TIMING_PROFILE : 0u)', '0u',
           'HELLO_ACK hides a profiler build'),
    Mutant('main_period_not_restarted', 'firmware/Core/Src/main.c',
           '      link_period = (LinkPeriod){0};  /* the next period starts now */\n', '',
           'STATE sums never restarted'),
    Mutant('main_profiler_switch_committed_on', 'firmware/Core/Src/main.c',
           '#define LOOP_TIMING_PROFILE   0', '#define LOOP_TIMING_PROFILE   1',
           'LOOP_TIMING_PROFILE committed as 1'),
    Mutant('main_profiler_without_warning', 'firmware/Core/Src/main.c',
           '#warning "LOOP_TIMING_PROFILE build: an extra prof line per window (never sent in binary mode). Bench only."\n',
           '', 'a profiler build compiles without its #warning'),
    # --- tools/rover_link.py
    Mutant('py_crc_init_zero', 'tools/rover_link.py',
           "return binascii.crc_hqx(bytes(data), 0xFFFF)", "return binascii.crc_hqx(bytes(data), 0)",
           'Python CRC with initial value 0'),
    Mutant('py_skip_crc_check', 'tools/rover_link.py',
           'if crc16(body[:-2]) != received:', 'if False:',
           'Python never compares the CRC'),
    Mutant('py_crc_big_endian', 'tools/rover_link.py',
           'body += bytes([crc & 0xFF, crc >> 8])', 'body += bytes([crc >> 8, crc & 0xFF])',
           'Python sends the CRC big-endian'),
    Mutant('py_encode_extra_block_after_254', 'tools/rover_link.py',
           'if not byte or (code == 0xFF and i + 1 < length):', 'if not byte or code == 0xFF:',
           'Python COBS: an empty block after a full one at the end'),
    Mutant('py_decode_zero_after_full_block', 'tools/rover_link.py',
           'if code != 0xFF and read < length:', 'if read < length:',
           'Python COBS: a 0x00 inserted after a 254-byte block'),
    Mutant('py_decode_accepts_zero_data', 'tools/rover_link.py',
           '        if 0 in block:\n            raise FrameError(COBS_ERROR)\n', '',
           'Python COBS: a 0x00 inside a block accepted'),
    Mutant('py_too_long_unchecked', 'tools/rover_link.py',
           'if len(cobs_bytes) > MAX_COBS:', 'if len(cobs_bytes) > MAX_COBS + 64:',
           'Python: pieces longer than the maximum not refused'),
    Mutant('py_stream_unbounded', 'tools/rover_link.py',
           'if len(self.buffer) < MAX_COBS:', 'if True:',
           'Python stream decoder: buffer grows without bound'),
    Mutant('py_stream_no_leading', 'tools/rover_link.py',
           'if not self.synced:', 'if False:',
           'Python stream decoder: the piece before the first 0x00 decoded as a frame'),
    # --- tools: rover_link messages, the binary console, logger and dump
    Mutant('py_drive_arm_bit_lost', 'tools/rover_link.py',
           '(DRIVE_ARM if arm else 0)', '(0 if arm else 0)',
           'Python DRIVE: the arm bit never set'),
    Mutant('py_cps_ties_away_from_zero', 'tools/rover_link.py',
           'units = (magnitude + CPS_PER_UNIT // 2 - 1) // CPS_PER_UNIT',
           'units = (magnitude + CPS_PER_UNIT // 2) // CPS_PER_UNIT',
           'Python cps -> units: ties away from zero'),
    Mutant('py_state_nibbles_from_the_wrong_byte', 'tools/rover_link.py',
           'states = values.pop(14)', 'states = values.pop(15)',
           'Python STATE: mstate/ctl read from the seq echo byte'),
    Mutant('console_esc_without_estop', 'tools/motor_console.py',
           '        frames = [self._estop()]\n', '        frames = []\n',
           'Esc / estop send no ESTOP'),
    Mutant('console_keepalive_arms', 'tools/motor_console.py',
           '            due.append(self._drive())', '            due.append(self._drive(arm=True))',
           'every DRIVE keepalive carries the arm bit'),
    Mutant('console_drives_before_hello_ack', 'tools/motor_console.py',
           'if self.acknowledged and self.sending and', 'if self.sending and',
           'DRIVEs before the board has answered HELLO'),
    Mutant('console_m_not_refused', 'tools/motor_console.py',
           "if COMMAND_PATTERN.match(text) and text.startswith('m'):", 'if False:',
           '"m" not refused in binary mode'),
    Mutant('console_injected_crc_is_valid', 'tools/motor_console.py',
           'crc = rover_link.crc16(body) ^ 0x0001', 'crc = rover_link.crc16(body)',
           '"inject crc" sends valid frames'),
    Mutant('console_exit_without_estop', 'tools/motor_console.py',
           '        return [self._estop()]\n', '        return []\n',
           'no ESTOP on the way out'),
    Mutant('logger_hello_never_stops', 'tools/serial_logger.py',
           '    def acknowledged(self):\n        self.pending = False',
           '    def acknowledged(self):\n        self.pending = True',
           'HELLO resent forever (it latches the motor each time)'),
    Mutant('logger_no_hello_after_boot', 'tools/serial_logger.py',
           '            hello.board_in_text_mode(now, HELLO_AFTER_BOOT_SECONDS)\n', '',
           'no HELLO after a board reset (binary mode lost)'),
    Mutant('logger_text_piece_counted_bad', 'tools/serial_logger.py',
           '                if self.piece_had_text:', '                if False:',
           'a boot line between frames counted as a bad frame'),
    Mutant('dump_ignores_seq_gaps', 'tools/link_dump.py',
           "logger.counts['bad'] == 0 and not logger.seq_gaps and", "logger.counts['bad'] == 0 and",
           'link_dump passes a log with lost frames'),
    Mutant('dump_piece_is_the_last_one', 'tools/link_dump.py',
           'rows[name].append([piece, seq]', 'rows[name].append([logger.frame_number, seq]',
           'link_dump writes the final piece count on every row (the 2026-10-15 bug)'),
    Mutant('logger_event_piece_lost', 'tools/serial_logger.py',
           "events.append(('frame', name, frame.seq, fields, self.frame_number))",
           "events.append(('frame', name, frame.seq, fields, 0))",
           'frame events carry no piece number'),
    Mutant('console_gate_never_warns', 'tools/motor_console.py',
           'elif bad - base_bad >= TEXT_GATE_REJECTED:', 'elif False:',
           'text console never explains a truck build'),
    Mutant('console_gate_ignores_cmd_ok', 'tools/motor_console.py',
           '        if ok != base_ok:\n            self.gate_settled = True',
           '        if False:\n            self.gate_settled = True',
           'text console warns on a bench build that also rejected some lines'),
    Mutant('console_gate_counts_raw_sends', 'tools/motor_console.py',
           "                self.messages.append(f'sent once, as-is: {text[1:]!r}')\n",
           "                self.messages.append(f'sent once, as-is: {text[1:]!r}')\n"
           '                self.sent_valid_command = True\n',
           'text console warns while you test rejection with !text'),
    Mutant('py_bench_flag_unnamed', 'tools/rover_link.py',
           "(0x08, 'LOOP_TIMING_PROFILE'), (0x10, 'BENCH_TEXT_COMMANDS'))", "(0x08, 'LOOP_TIMING_PROFILE'))",
           'the tools show a bench-text build as an unknown flag'),
]


def suites_for(path):
    relative = path.removeprefix('firmware/')
    if relative in HEADER_SUITES:
        return HEADER_SUITES[relative]
    return [name for name, sources in SUITES.items() if relative in sources]


def run_c_suites(firmware_dir, names, build_dir):
    """'pass', 'fail:<suite>' or 'invalid:<suite>'."""
    for name in names:
        binary = build_dir / name
        build = subprocess.run(['gcc', *FLAGS, *SUITES[name], '-o', str(binary)], cwd=firmware_dir,
                               capture_output=True, text=True)
        if build.returncode != 0:
            return f'invalid:{name}: {build.stderr.strip().splitlines()[0] if build.stderr.strip() else ""}'
        try:
            run = subprocess.run([str(binary)], cwd=firmware_dir, capture_output=True, text=True,
                                 timeout=SUITE_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            return f'fail:{name} (timeout)'
        if run.returncode != 0:
            return f'fail:{name}'
    return 'pass'


def python_tests_for(path):
    """The pytest file that covers a mutated file: tools/x.py -> tools/test_x.py; main.c,
    which has no host suite, -> the source pins (tools/test_firmware_sources.py). A main.c
    mutant killed there proves a pin notices the change, which is weaker than a behaviour test."""
    if path == 'firmware/Core/Src/main.c':
        return 'tools/test_firmware_sources.py'
    return f'tools/test_{Path(path).stem}.py'


def run_python_tests(root, test_file):
    # The copy isn't a git repository; tests that read an old version of a file
    # (test_serial_logger's pre-refactor comparison) read it from the real one.
    environment = dict(os.environ, GIT_DIR=str(REPOSITORY_ROOT / '.git'))
    run = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-x', '-p', 'no:cacheprovider', test_file],
                         cwd=root, capture_output=True, text=True, timeout=SUITE_TIMEOUT_SECONDS, env=environment)
    return 'pass' if run.returncode == 0 else f'fail:{Path(test_file).stem}'


def copy_tree(destination):
    """The parts of the repo the tests need: firmware Core, tests, CMakeLists.txt and the
    .ioc (the source pins read them), every tools/*.py, and tools/test_data (test_link_dump's
    captured frames)."""
    shutil.copytree(FIRMWARE / 'Core', destination / 'firmware' / 'Core')
    shutil.copytree(FIRMWARE / 'tests', destination / 'firmware' / 'tests')
    for name in ('CMakeLists.txt', 'rover_firmware.ioc'):
        shutil.copy2(FIRMWARE / name, destination / 'firmware' / name)
    (destination / 'tools').mkdir()
    for path in (REPOSITORY_ROOT / 'tools').glob('*.py'):
        shutil.copy2(path, destination / 'tools' / path.name)
    shutil.copytree(REPOSITORY_ROOT / 'tools' / 'test_data', destination / 'tools' / 'test_data')


PYTHON_TEST_FILES = sorted({python_tests_for(m.path) for m in MUTANTS
                            if m.path.startswith('tools/') or m.path == 'firmware/Core/Src/main.c'})


def evaluate(mutant, workspace):
    root = workspace / mutant.id
    copy_tree(root)
    target = root / mutant.path
    text = target.read_text(encoding='utf-8')
    if text.count(mutant.old) != 1:
        return f'invalid: text occurs {text.count(mutant.old)} times'
    if mutant.new == mutant.old:
        return 'invalid: no change'
    target.write_text(text.replace(mutant.old, mutant.new), encoding='utf-8', newline='')
    if mutant.path.startswith('tools/') or mutant.path == 'firmware/Core/Src/main.c':
        result = run_python_tests(root, python_tests_for(mutant.path))
    else:
        build_dir = root / 'build'
        build_dir.mkdir()
        result = run_c_suites(root / 'firmware', suites_for(mutant.path), build_dir)
    shutil.rmtree(root, ignore_errors=True)
    return result


def main():
    selected = [m for m in MUTANTS if len(sys.argv) < 2 or sys.argv[1] in m.id]
    with tempfile.TemporaryDirectory(prefix='link_mutants_') as temporary:
        workspace = Path(temporary)
        # Baseline: the unmutated sources pass every suite used here.
        baseline_root = workspace / 'baseline'
        copy_tree(baseline_root)
        (baseline_root / 'build').mkdir()
        baseline = run_c_suites(baseline_root / 'firmware', list(SUITES), baseline_root / 'build')
        python_baseline = next((result for result in (run_python_tests(baseline_root, test_file)
                                                      for test_file in PYTHON_TEST_FILES) if result != 'pass'), 'pass')
        if baseline != 'pass' or python_baseline != 'pass':
            print(f'BASELINE FAILS: C {baseline}, Python {python_baseline}')
            return 2
        shutil.rmtree(baseline_root, ignore_errors=True)
        print(f'baseline passes; {len(selected)} mutants')
        killed = survived = invalid = 0
        for mutant in selected:
            result = evaluate(mutant, workspace)
            if result.startswith('fail'):
                killed += 1
                verdict = 'KILLED  '
            elif result == 'pass':
                survived += 1
                verdict = 'SURVIVED'
            else:
                invalid += 1
                verdict = 'INVALID '
            print(f'{verdict} {mutant.id:40s} {result:32s} {mutant.bug}', flush=True)
        print(f'{killed} killed, {survived} survived, {invalid} invalid of {len(selected)}')
        return 0 if survived == 0 and invalid == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
