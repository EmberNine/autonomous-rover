#!/usr/bin/env python3
"""Send motor and steering commands to the rover firmware while logging its telemetry (Windows or Linux).

Only one program can hold the COM port, so this does both jobs: it logs
exactly like tools/serial_logger.py (same TelemetryLogger code, so the same
<output> raw file, <output>.times.csv and status counts), and it sends what
you type.

Keys:
  m 20 + Enter   20 % forward          m -15 + Enter   15 % reverse   (open loop, duty)
  m 0  + Enter   stop (also re-arms the firmware after a watchdog trip, a latch or a reset)
  v 50 + Enter   closed loop: 50 speed units forward (1 unit = 50 counts/s);
                 v -30 reverse; v 0 stop (coast; also re-arms like m 0)
  s 50 + Enter   steer 50 % left       s -50 + Enter   50 % right       s 0   centre
  Esc            stop now: sends "m 0" at once and keeps sending it; also
                 "s 0" (centre) if steering was commanded
  Ctrl+C         sends "m 0" (and "s 0" if steering was commanded), then quits
  -m / -v / -s + Enter  test aid: stop sending the motor (m or v) / s keepalive
                 (the firmware's motor watchdog / steering timeout then acts
                 within 500 ms)
  !text + Enter  sends "text" once, as-is (for testing that bad lines are
                 rejected, or for a test build's own letter: "!e 2")

"m" is a bench/diagnostic command; the Pi never sends text (it uses the
binary link, --binary below).

Text commands work only on a bench or test build: a truck build
(BENCH_TEXT_COMMANDS 0) counts every text line as rejected and acts on none.
If the board's cmd_bad grows by 3 after your commands while cmd_ok doesn't
move, the console says so once (and suggests --binary).

Keepalives: the last valid motor command ("m" or "v": they share ONE slot,
because both feed the same firmware motor watchdog; the last one typed
wins, so typing "v 50" stops resending "m 20") and the last valid "s" are
each resent every 100 ms, each on its own schedule (the firmware stops the
motor, or centres the steering, if it hears no valid command for it for
500 ms; the two timeouts are independent). Nothing is sent until you type
the first command, so the firmware's boot latch can be tested, and no "s" is
ever sent unless you typed one, so the servo stays without pulses. When the
board resets (a boot line arrives), sending stops until you type again.

The loop is single-threaded on purpose: if the console freezes, keepalives
stop and the firmware's watchdog stops the motor (a dead man's switch).

Status line (one row, redrawn in place): the logger's counts, what is being
sent, "IMU FAIL" if the board booted without its IMU (imu_ok=0), the motor
state with d100 (encoder counts in the last 100 ms), the steering, the
battery (telemetry v4: "vbat=7.92V bst=1", with NO BAT / LOW BAT / CUTOFF /
ADC FAULT / REMOVED WHILE DRIVING in front when it isn't OK), the speed
control (telemetry v5: "run vset=50 vest=49 vi=307", with EST BAD when the
estimate is invalid and stall= / enc_bad= once they are non-zero), the
motor fault reason ("MFAULT 7 REPEATED STALL") when there is one, the reset
cause, and what you are typing. It never gets wider than
the window, because a wrapped row can't be redrawn in place (the rows would
pile up). In a narrow window the reset-cause decode goes first, then the
logger's counts are shortened (see status_line_text); the battery, the speed
control and the fault reason are always kept.

QuickEdit: in the classic Windows console window, a mouse click or drag starts
a text selection that pauses the program's output, which freezes this loop
(no keepalives: the watchdog trips), and while a selection is active Esc and
Ctrl+C go to the selection, not to this program. So at startup QuickEdit is
turned off for this window (SetConsoleMode on the input handle with
ENABLE_EXTENDED_FLAGS and without ENABLE_QUICK_EDIT_MODE, as documented at
https://learn.microsoft.com/en-us/windows/console/setconsolemode), and the
original mode is put back on every way out this program controls: the Ctrl+C
key, KeyboardInterrupt, or an exception. A kill from Task Manager can't
restore it: that window then stays without QuickEdit until it is closed.
If the mode can't be read or set (stdin isn't a console window), a note says
so and the console runs without that protection. Edit > Mark from the window
menu and the Pause key can still pause the window; the watchdog covers that.
In Windows Terminal, selection is the terminal's, not the console mode's, so
this setting probably has no effect there; its docs say Esc clears an active
selection, so don't select text while driving in any terminal.

Binary mode (--binary): the Pi link, as the Pi will use it (tools/rover_link.py,
firmware/Core/Inc/link_protocol.h). The console sends HELLO (again every
0.5 s until the board answers, and again after a reset) and shows the
HELLO_ACK, loudly if the board runs a test build. HELLO latches the motor.
  v 2500 + Enter   speed setpoint in counts/s (-5000..5000; below 1000 the
                   firmware coasts); s 20 steer 20 % left
  arm + Enter      one DRIVE 0 with the arm bit: re-arms a latched motor
                   (the only thing that does); the keepalive goes on at v 0
  Esc / estop      ESTOP: latched at once, until "arm" (Esc also centres
                   the steering if it was commanded)
  hello            HELLO again (latches the motor)
  -v / -s          test aids: stop all DRIVEs (motor watchdog and steering
                   timeout act) / DRIVEs without steer_valid (steering centres)
  inject <kind> N  test aid (bench step 6): N frames the firmware must drop:
                   crc, cobs, length, type, reserved, range, arm; the console
                   says which STATUS counter must grow by N
  !text            sends text once (binary mode must ignore it)
  "m" is refused: open loop is a bench text-mode command.
One DRIVE (speed + steering) goes out every 20 ms (50 Hz) once something is
typed and the board has answered HELLO; nothing re-arms by itself. The status
line decodes STATE and STATUS: motor state (with the latch or fault reason),
duty, the estimate in counts/s, the odometer count and epoch (a change = a
stretch of "distance uncertain"), the controller, steering, battery, stalls
by rule, the link's receive counters and loop_us_max. On the way out: ESTOP x3.
The raw file is logged as in text mode; <output>.times.csv has one row per
frame (serial_logger.BinaryLogger), and tools/link_dump.py decodes the raw file.
The first HELLO goes out 0.1 s after the start; frames before the first
HELLO_ACK are stale (left over in the ST-LINK bridge from before the port was
opened): kept in the raw file, counted on their own in the summary, never
shown on the status line.

On the way out (both modes), after the stops, the console keeps logging for
EXIT_DRAIN_SECONDS (0.2 s), so the log shows the stop taking effect.

Linux (the Pi, over SSH): keys come from the terminal (termios: no echo,
each key at once; the settings come back on every way out the console
controls). Commands, keys, the status line and the exit stops are as on
Windows, with one difference: on Linux, arrows, Home/End, Delete, F-keys
and Alt+key are Esc (stop), because they arrive as Esc sequences
(decode_keys); on Windows they are ignored. SIGHUP (the SSH session
dropped) and SIGTERM stop the console like Ctrl+C: the same stops go out
(ESTOP x3, or "m 0" x3) before the port is closed. SIGKILL can't be
caught: nothing is sent, and the firmware's 500 ms motor watchdog stops the
motor (the steering timeout centres the servo). The port is opened with
pyserial's exclusive=True, an advisory lock: it stops a second
motor_console, not a program that opens the port without it. There is no
QuickEdit on Linux.

Usage:  py tools\\motor_console.py COM5 C:\\dev\\rover-logs\\motor_bench.log [--baud 115200] [--binary]
        python3 tools/motor_console.py /dev/ttyACM0 ~/rover-logs/motor_bench.log [--baud 115200] [--binary]
"""
import argparse
import codecs
import contextlib
import ctypes
import os
import re
import select
import shutil
import sys
import time

import rover_link
from serial_logger import (EXIT_SIGNAL_NAMES, HELLO_START_DELAY_SECONDS, STATUS_EVERY_SECONDS,  # noqa: F401
                           BinaryLogger, ExitSignals, HelloSender, Sequence, TelemetryLogger, handle_hello_events,
                           print_writer_problems, queued_writer)

# Same grammar as the firmware (firmware/Core/Inc/command_line.h): the letter,
# one space, optional "-", 1 to 3 digits, value -100..100. The console only
# sends the letters the firmware acts on.
COMMAND_PATTERN = re.compile(r'^([msv]) -?(\d{1,3})$')
# Keepalive slot of each letter: "m" and "v" share the motor's (one watchdog).
KEEPALIVE_SLOT = {'m': 'motor', 'v': 'motor', 's': 'steering'}
MAX_COMMAND_PERCENT = 100
STOP_COMMAND = 'm 0'
CENTRE_COMMAND = 's 0'
KEEPALIVE_SECONDS = 0.1        # 5 keepalives per firmware watchdog timeout (500 ms)
READ_TIMEOUT_SECONDS = 0.02    # the loop runs at least 50 times a second, so keys and keepalives are prompt
EXIT_STOP_REPEATS = 3          # "m 0" (and "s 0") sent this many times on exit, in case one is lost
EXIT_STOP_SPACING_SECONDS = 0.02
RAW_SEND_PREFIX = '!'
STOP_KEEPALIVE_PREFIX = '-'    # "-m" / "-s": stop that keepalive (test aid)

ESC = '\x1b'
CTRL_C = '\x03'
ENTER = '\r'
BACKSPACE = '\x08'
SPECIAL_KEY_PREFIXES = ('\x00', '\xe0')  # msvcrt: arrow/function keys arrive as prefix + code
LINE_FEED = '\n'               # Linux: Enter may arrive as LF
DELETE = '\x7f'                # Linux: what most terminals send for Backspace

MOTOR_STATUS_PATTERN = re.compile(rb' duty=(-?\d+) mstate=(\d+) ')
# Encoder counts in the last 100 ms window; in every telemetry version (v1 on).
D100_PATTERN = re.compile(rb' d100=(-?\d+) ')
# Valid and rejected command lines since boot (v2 on): the truck build's text
# mode (BENCH_TEXT_COMMANDS 0) counts every line as rejected.
COMMAND_COUNTS_PATTERN = re.compile(rb' cmd_ok=(\d+) cmd_bad=(\d+) ')
TEXT_GATE_REJECTED = 3     # rejected lines, with no valid one, before the console says why
TEXT_GATE_WARNING = ('the board counts every command as rejected (cmd_bad +{bad}, cmd_ok unchanged): this looks '
                     'like a truck build (BENCH_TEXT_COMMANDS 0), whose text mode only waits for the Pi link. '
                     'Use motor_console.py --binary, or flash a BENCH_TEXT_COMMANDS 1 build for text commands.')
# IMU init result, fixed after boot (v1 on). 0: the firmware runs without the
# IMU and its LED double-blinks; the Pi must not drive autonomously.
IMU_OK_PATTERN = re.compile(rb' imu_ok=(\d+) ')
IMU_MISSING_MARKER = 'IMU FAIL'
# Telemetry v3 only (firmware/README.md): reset cause, then the steering fields.
STEERING_STATUS_PATTERN = re.compile(rb' rst=(\d+) s_pct=(-?\d+) s_us=(\d+) sstate=(\d+) ')
# Telemetry v4 on: the battery fields (they end a v4 line; the speed fields follow in v5).
BATTERY_STATUS_PATTERN = re.compile(rb' vbat=(\d+) bstate=(\d+) vdda=(\d+)(?: |\r\n$)')
# Telemetry v5 only: the speed control fields end the line.
SPEED_STATUS_PATTERN = re.compile(
    rb' vset=(-?\d+) vest=(-?\d+) ctl=(\d+) vok=(\d+) vi=(-?\d+) stall_n=(\d+) enc_bad=(\d+) mfault=(\d+)\r\n$')
# ctl (firmware speed_control.h, SpeedControlState), in code order.
SPEED_CTL_NAMES = ('open', 'run', 'stop', 'GATE')
EST_BAD_MARKER = 'EST BAD'
# mfault (firmware motor_control.h, MotorFaultReason), in code order; 0 = none.
# tools/test_firmware_sources.py checks the count against the firmware enum.
MOTOR_FAULT_NAMES = ('none', 'PWM PERIOD', 'CAP GUARD', 'DIRECTION GUARD', 'LOW BATTERY', 'BATTERY ADC',
                     'BATTERY REMOVED WHILE DRIVING', 'REPEATED STALL')
# bstate (firmware battery_monitor.h, BatteryState) -> marker shown on the
# status line; OK (1) shows none.
BATTERY_STATE_MARKERS = {
    0: 'NO BAT', 2: 'LOW BAT', 3: 'CUTOFF', 4: 'ADC FAULT', 5: 'REMOVED WHILE DRIVING',
}
# Used when the console can't tell the window width (shutil.get_terminal_size fallback).
FALLBACK_COLUMNS = 120
# rst bits (RCC_CSR reset flags shifted down by 24, firmware main.c), most
# telling first: an internal reset also sets PIN.
RESET_CAUSE_BITS = (
    (5, 'IWDG'), (6, 'WWDG'), (4, 'SFT'), (7, 'LPWR'), (3, 'BOR'), (1, 'OBL'), (0, 'FW'), (2, 'PIN'),
)

# Console input modes, from https://learn.microsoft.com/en-us/windows/console/setconsolemode
# ("To disable this mode, use ENABLE_EXTENDED_FLAGS without this flag.")
ENABLE_QUICK_EDIT_MODE = 0x0040
ENABLE_EXTENDED_FLAGS = 0x0080
STD_INPUT_HANDLE = -10                               # GetStdHandle: (DWORD)-10
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value    # what GetStdHandle returns on failure
QUICK_EDIT_OFF_NOTE = 'QuickEdit is off in this console window until the console exits.'
QUICK_EDIT_FAILED_NOTE = ('Note: could not turn QuickEdit off ({reason}). In a classic console window, '
                          'selecting text pauses sending; the firmware watchdog then stops the motor.')


def command_letter(text):
    """'m', 'v' or 's' for a valid command (the firmware's grammar), else None."""
    match = COMMAND_PATTERN.match(text)
    if not match or int(match.group(2)) > MAX_COMMAND_PERCENT:
        return None
    return match.group(1)


def is_motor_command(text):
    """"m" or "v": both drive the motor and feed its watchdog."""
    return command_letter(text) in ('m', 'v')


def is_steering_command(text):
    return command_letter(text) == 's'


def encode_line(text):
    return (text + '\n').encode('utf-8')


class CommandConsole:
    """What to send and when, from key presses and received lines. No I/O.

    The motor and steering keepalives are independent: each slot has its own
    last command and its own 100 ms schedule, so typing one never delays the
    other. "m" and "v" share the motor slot (one firmware watchdog): the last
    one typed is the one resent."""

    SLOTS = ('motor', 'steering')  # keepalives go out in this order when both are due

    def __init__(self):
        self.commands = {'motor': None, 'steering': None}   # last command per slot; None = not sending
        self.next_keepalive_times = {'motor': None, 'steering': None}
        self.typed_text = ''
        self.quit_requested = False
        self.skip_next_key = False     # second half of a special key
        self.messages = []             # lines for the screen
        self._reset_text_gate_watch()

    def _reset_text_gate_watch(self):
        # Is the board ignoring text commands (a truck build)? Watched from the
        # first valid command sent until the board shows it acts on one, or
        # the warning has been given (once per boot).
        self.sent_valid_command = False
        self.gate_baseline = None      # (cmd_ok, cmd_bad) at the first line after the first command
        self.gate_settled = False

    def check_text_gate(self, good_line):
        """A good telemetry line arrived: warn once if, since the first valid
        command, cmd_bad has grown by TEXT_GATE_REJECTED or more while cmd_ok
        hasn't moved (steering-only commands count in s_ok, so they never
        look like acting; on a truck build they count in cmd_bad)."""
        if self.gate_settled or not self.sent_valid_command or not good_line:
            return
        match = COMMAND_COUNTS_PATTERN.search(good_line)
        if not match:
            return
        ok, bad = int(match.group(1)), int(match.group(2))
        if self.gate_baseline is None:
            self.gate_baseline = (ok, bad)
            return
        base_ok, base_bad = self.gate_baseline
        if ok != base_ok:
            self.gate_settled = True   # the board acts on text commands
        elif bad - base_bad >= TEXT_GATE_REJECTED:
            self.messages.append(TEXT_GATE_WARNING.format(bad=bad - base_bad))
            self.gate_settled = True

    @property
    def motor_command(self):
        return self.commands['motor']

    @property
    def steering_command(self):
        return self.commands['steering']

    def _set_command(self, command, now):
        slot = KEEPALIVE_SLOT[command[0]]
        self.commands[slot] = command
        self.next_keepalive_times[slot] = now + KEEPALIVE_SECONDS
        self.sent_valid_command = True
        return [encode_line(command)]

    def _stop_keepalive(self, letter):
        slot = KEEPALIVE_SLOT[letter]
        command = self.commands[slot]
        if command is None:
            self.messages.append(f'-{letter}: no {letter} keepalive was being sent')
            return
        self.commands[slot] = None
        self.next_keepalive_times[slot] = None
        what = 'motor watchdog' if slot == 'motor' else 'steering timeout (-> centre)'
        sent = command[0]
        shared = '' if sent == letter else ' (m and v share one slot)'
        self.messages.append(f'-{letter}: stopped sending {sent} keepalives{shared}; '
                             f'the firmware {what} acts within 500 ms')

    def exit_commands(self):
        """What to send (repeatedly) on the way out: always "m 0"; "s 0" only
        if steering was commanded, so a servo that never got a command isn't
        started on exit."""
        return [STOP_COMMAND] + ([CENTRE_COMMAND] if self.steering_command is not None else [])

    def handle_key(self, key, now):
        """One key press. Returns the bytes to send right away (a list)."""
        if self.skip_next_key:
            self.skip_next_key = False
            return []
        if key in SPECIAL_KEY_PREFIXES:
            self.skip_next_key = True
            return []
        if key == CTRL_C:
            self.quit_requested = True
            return []
        if key == ESC:
            self.typed_text = ''
            if self.steering_command is None:
                self.messages.append('Esc: m 0')
                return self._set_command(STOP_COMMAND, now)
            # Centre too: the neutral point at once, not 500 ms later via the
            # firmware timeout. Not sent if steering was never commanded.
            self.messages.append('Esc: m 0, s 0')
            return self._set_command(STOP_COMMAND, now) + self._set_command(CENTRE_COMMAND, now)
        if key == BACKSPACE:
            self.typed_text = self.typed_text[:-1]
            return []
        if key == ENTER:
            text, self.typed_text = self.typed_text.strip(), ''
            if not text:
                return []
            if text.startswith(RAW_SEND_PREFIX):
                self.messages.append(f'sent once, as-is: {text[1:]!r}')
                return [encode_line(text[1:])]
            if text in tuple(STOP_KEEPALIVE_PREFIX + letter for letter in KEEPALIVE_SLOT):
                self._stop_keepalive(text[1])
                return []
            if command_letter(text) is not None:
                return self._set_command(text, now)
            self.messages.append(f'not sent: {text!r} is not a command '
                                 f'(m <-100..100>, v <-100..100>, s <-100..100>, -m, -v, -s, '
                                 f'or !text to send as-is)')
            return []
        if key.isprintable():
            self.typed_text += key
        return []

    def handle_line_results(self, results):
        """Classifications of the lines just received (from TelemetryLogger.feed)."""
        if 'boot' in results:
            for slot in self.SLOTS:
                self.commands[slot] = None
                self.next_keepalive_times[slot] = None
            self._reset_text_gate_watch()
            self.messages.append('board reset: sending stopped; the firmware is latched until "m 0" '
                                 'and the servo has no pulses until "s"')

    def keepalive_due(self, now):
        """The keepalive lines to send now (a list of bytes, maybe empty), the motor's first."""
        due = []
        for slot in self.SLOTS:
            command, next_time = self.commands[slot], self.next_keepalive_times[slot]
            if command is None or now < next_time:
                continue
            next_time += KEEPALIVE_SECONDS
            if next_time <= now:                     # fell behind (e.g. the PC stalled): no burst
                next_time = now + KEEPALIVE_SECONDS
            self.next_keepalive_times[slot] = next_time
            due.append(encode_line(command))
        return due

    def take_messages(self):
        messages, self.messages = self.messages, []
        return messages


def motor_status_text(good_line):
    """'duty=320 mstate=1 d100=245' from a line with motor fields, 'd100=245'
    from a v1 line (no motor fields), '' before the first good line."""
    if not good_line:
        return ''
    motor = MOTOR_STATUS_PATTERN.search(good_line)
    d100 = D100_PATTERN.search(good_line)
    parts = []
    if motor:
        parts.append(f'duty={int(motor.group(1))} mstate={int(motor.group(2))}')
    if d100:
        parts.append(f'd100={int(d100.group(1))}')
    return ' '.join(parts)


def imu_missing(good_line):
    """True if the line says imu_ok=0 (IMU init failed at boot)."""
    if not good_line:
        return False
    match = IMU_OK_PATTERN.search(good_line)
    return bool(match) and int(match.group(1)) == 0


def reset_cause_text(flags):
    """'IWDG+PIN' for rst=36: the set bits, most telling first; 'none' for 0."""
    names = [name for bit, name in RESET_CAUSE_BITS if flags & (1 << bit)]
    return '+'.join(names) if names else 'none'


def steering_status_text(good_line):
    """'s=-40 us=1380 sst=1' from a v3 line, else '' (v1/v2 lines have no steering)."""
    if not good_line:
        return ''
    match = STEERING_STATUS_PATTERN.search(good_line)
    if not match:
        return ''
    return f's={int(match.group(2))} us={int(match.group(3))} sst={int(match.group(4))}'


def battery_status_text(good_line):
    """'vbat=7.92V bst=1' from a v4 line, with the state's marker first when it
    isn't OK ('CUTOFF vbat=6.58V bst=3'); '' for v1-v3 lines (no battery)."""
    if not good_line:
        return ''
    match = BATTERY_STATUS_PATTERN.search(good_line)
    if not match:
        return ''
    centivolts = (int(match.group(1)) + 5) // 10
    state = int(match.group(2))
    text = f'vbat={centivolts // 100}.{centivolts % 100:02d}V bst={state}'
    marker = BATTERY_STATE_MARKERS.get(state, f'BSTATE {state}?' if state != 1 else None)
    return f'{marker} {text}' if marker else text


def speed_status_text(good_line):
    """The speed control from a v5 line, '' for older lines:
    'run vset=50 vest=49 vi=307' (v running), 'GATE vset=-50 vest=12 vi=0'
    (waiting for the wheel to stop), 'stop vset=0 vest=0 vi=0', 'open vest=92'
    (open loop: "m"), then ' EST BAD' while the estimate is invalid, and
    ' stall=N' / ' enc_bad=N' once non-zero."""
    if not good_line:
        return ''
    match = SPEED_STATUS_PATTERN.search(good_line)
    if not match:
        return ''
    vset, vest, ctl, vok, vi, stall_n, enc_bad, _ = (int(group) for group in match.groups())
    name = SPEED_CTL_NAMES[ctl] if ctl < len(SPEED_CTL_NAMES) else f'CTL {ctl}?'
    text = f'{name} vest={vest}' if ctl == 0 else f'{name} vset={vset} vest={vest} vi={vi}'
    if not vok:
        text += ' ' + EST_BAD_MARKER
    if stall_n:
        text += f' stall={stall_n}'
    if enc_bad:
        text += f' enc_bad={enc_bad}'
    return text


def motor_fault_text(good_line):
    """'MFAULT 7 REPEATED STALL' from a v5 line whose mfault isn't 0, else ''."""
    if not good_line:
        return ''
    match = SPEED_STATUS_PATTERN.search(good_line)
    if not match or int(match.group(8)) == 0:
        return ''
    code = int(match.group(8))
    name = MOTOR_FAULT_NAMES[code] if code < len(MOTOR_FAULT_NAMES) else '?'
    return f'MFAULT {code} {name}'


def reset_status_text(good_line, decoded=True):
    """'rst=36 (IWDG+PIN)' (or 'rst=36' with decoded=False) from a v3 line, else ''."""
    if not good_line:
        return ''
    match = STEERING_STATUS_PATTERN.search(good_line)
    if not match:
        return ''
    flags = int(match.group(1))
    return f'rst={flags} ({reset_cause_text(flags)})' if decoded else f'rst={flags}'


def status_line_text(status_text, console, logger, max_length):
    """The status line, at most max_length characters.

    Full form: the logger's status text unchanged, then the console's part:
      good=.. bad=.. (..%) boot=.. t_ms_gaps=..   | sent: v 50 s -40 | duty=307 mstate=1 d100=250
      | run vset=50 vest=49 vi=307 | s=-40 us=1380 sst=1 | vbat=7.92V bst=1 | rst=36 (IWDG+PIN) | > typed
    with "IMU FAIL | " and then "MFAULT <code> <reason> | " before the motor
    state when the line says imu_ok=0 / mfault != 0, and the battery marker
    (NO BAT, LOW BAT, CUTOFF, ADC FAULT, REMOVED WHILE DRIVING) before vbat
    when the battery isn't OK. The speed part is there for v5 lines only.
    If that doesn't fit, parts are given up in this order until it does: the
    reset-cause decode, the logger's counts beyond good/bad, the reset cause,
    the logger's counts, the commands being sent. The IMU and fault markers,
    the motor state, d100, the speed control, the steering, the battery and
    the typed text are never given up; if even those don't fit,
    the line is cut from the left so the end of the typed text stays visible.
    """
    good_line = logger.last_good_line
    sending = ' '.join(command for command in (console.motor_command, console.steering_command) if command)
    short_counts = f"good={logger.counts['good']} bad={logger.counts['bad']}"
    motor = motor_status_text(good_line) or 'no motor fields yet'
    fault = motor_fault_text(good_line)
    speed = speed_status_text(good_line)
    steering = steering_status_text(good_line)
    battery = battery_status_text(good_line)
    typed = f'> {console.typed_text}'

    def build(counts, show_sent, reset):
        parts = ([f'sent: {sending or "-"}'] if show_sent else [])
        parts += ([IMU_MISSING_MARKER] if imu_missing(good_line) else []) + ([fault] if fault else []) + [motor]
        parts += [part for part in (speed, steering, battery, reset) if part]
        text = ' | '.join(parts + [typed])
        return f'{counts}   | {text}' if counts else text

    variants = (
        build(status_text, True, reset_status_text(good_line)),
        build(status_text, True, reset_status_text(good_line, decoded=False)),
        build(short_counts, True, reset_status_text(good_line, decoded=False)),
        build(short_counts, True, ''),
        build('', True, ''),
        build('', False, ''),
    )
    for text in variants:
        if len(text) <= max_length:
            return text
    return variants[-1][-max_length:]


class StatusLine:
    """One screen line rewritten in place with '\\r', like serial_logger's
    status line (text from status_line_text).

    Never wider than the window minus one column. A line that wraps can't be
    redrawn with '\\r', which only goes back to the start of its last row:
    every redraw would leave the earlier rows behind, so the line seemed to
    repeat in a narrow window. The last column stays empty because writing
    into it makes some consoles move to the next row at once. The width is
    read on every redraw, so resizing the window takes effect at the next
    redraw (rows the console already wrapped while resizing are not cleaned up).

    columns: returns the window width; by default shutil.get_terminal_size
    (FALLBACK_COLUMNS when there is no console)."""

    def __init__(self, write, columns=None):
        self.write = write
        self.columns = columns or (lambda: shutil.get_terminal_size((FALLBACK_COLUMNS, 24)).columns)
        self.previous_length = 0

    def _max_length(self):
        return max(self.columns() - 1, 1)

    def max_length(self):
        return self._max_length()

    def show(self, status_text, console, logger):
        self.show_text(status_line_text(status_text, console, logger, self._max_length()))

    def show_text(self, text):
        """Redraws the line with text (already at most max_length() long)."""
        max_length = self._max_length()
        padded_length = min(self.previous_length, max_length)   # blanks out the rest of the previous line
        self.write('\r' + text.ljust(padded_length))
        if len(text) < padded_length:
            self.write('\r' + text)                  # cursor back to the end of the typed text
        self.previous_length = len(text)

    def message(self, text):
        """A line of its own above the status line (BAD lines, console notes)."""
        blank = ' ' * min(self.previous_length, self._max_length())
        self.write('\r' + blank + '\r' + text.lstrip('\n') + '\n')
        self.previous_length = 0


def load_kernel32():
    """kernel32 with the three console calls typed, or None where there is none (not Windows)."""
    try:
        kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    except (AttributeError, OSError):     # ctypes.WinDLL only exists on Windows
        return None
    kernel32.GetStdHandle.argtypes = [ctypes.c_int32]           # DWORD, passed as its bit pattern
    kernel32.GetStdHandle.restype = ctypes.c_void_p              # HANDLE
    kernel32.GetConsoleMode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
    kernel32.GetConsoleMode.restype = ctypes.c_int               # BOOL
    kernel32.SetConsoleMode.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    kernel32.SetConsoleMode.restype = ctypes.c_int
    return kernel32


class QuickEditOff:
    """Context manager: QuickEdit off for this console window while inside,
    the original input mode back on the way out (normal exit or exception).

    kernel32: load_kernel32() or a test fake with GetStdHandle, GetConsoleMode
    and SetConsoleMode (same arguments as the real calls); None = no console
    API. notify: prints one line. Never raises because of the console mode:
    if it can't be read or set, it says so once and changes nothing.
    """

    def __init__(self, kernel32, notify):
        self.kernel32 = kernel32
        self.notify = notify
        self.handle = None
        self.original_mode = None     # set only once the mode was actually changed

    def __enter__(self):
        if self.kernel32 is None:
            self.notify(QUICK_EDIT_FAILED_NOTE.format(reason='no Windows console API here'))
            return self
        handle = self.kernel32.GetStdHandle(STD_INPUT_HANDLE)
        if not handle or handle == INVALID_HANDLE_VALUE:
            self.notify(QUICK_EDIT_FAILED_NOTE.format(reason='no console input handle'))
            return self
        mode = ctypes.c_uint32()
        if not self.kernel32.GetConsoleMode(handle, ctypes.pointer(mode)):
            self.notify(QUICK_EDIT_FAILED_NOTE.format(reason='input is not a console window'))
            return self
        original_mode = mode.value
        if not self.kernel32.SetConsoleMode(handle, (original_mode | ENABLE_EXTENDED_FLAGS) & ~ENABLE_QUICK_EDIT_MODE):
            self.notify(QUICK_EDIT_FAILED_NOTE.format(reason='SetConsoleMode failed'))
            return self
        self.handle, self.original_mode = handle, original_mode
        self.notify(QUICK_EDIT_OFF_NOTE)
        return self

    def __exit__(self, *exception):
        if self.original_mode is not None:
            # EXTENDED_FLAGS so the saved QuickEdit/insert bits are applied as they were read.
            if not self.kernel32.SetConsoleMode(self.handle, self.original_mode | ENABLE_EXTENDED_FLAGS):
                self.notify('Note: could not restore the console mode; QuickEdit stays off in this window.')
            self.original_mode = None
        return False                  # never swallow the exception


# --- Keyboard: msvcrt on Windows, the terminal (termios) on Linux ---------------------

ESCAPE_TAIL_SECONDS = 0.05     # the rest of an Esc sequence split across reads must arrive within this
MAX_PENDING_ESCAPE = 16        # longer unfinished sequences are given up (decoded no further)
KEYBOARD_READ_BYTES = 256
NOT_A_TERMINAL_NOTE = ('motor_console.py needs a terminal for its keys (stdin is not one). '
                       'Over SSH, log in first, or use "ssh -t".')


def _escape_end(text, start):
    """text[start] is ESC: (index after its sequence, complete). Dropped with
    the ESC: a CSI sequence (ESC [ parameters final, e.g. arrows, Home/End,
    Delete, most F-keys), SS3 (ESC O x: F1-F4, some arrows) and Alt+key
    (ESC + a printable character). An ESC followed by a control character
    (another ESC, Ctrl+C, Enter) ends there: that character is a key of its
    own. complete False: the text ended inside the sequence."""
    end = len(text)
    i = start + 1
    if i == end:
        return end, False
    if text[i] == '[':
        i += 1
        while i < end and '\x20' <= text[i] <= '\x3f':     # parameter and intermediate bytes
            i += 1
        if i == end:
            return end, False
        return (i + 1 if '\x40' <= text[i] <= '\x7e' else i), True
    if text[i] == 'O':
        if i + 1 == end:
            return end, False
        return (i + 2 if '\x40' <= text[i + 1] <= '\x7e' else i + 1), True
    return (i + 1 if text[i].isprintable() else i), True


def decode_keys(text, pending=''):
    """Characters from a Linux terminal -> the keys handle_key takes, as
    msvcrt.getwch gives them on Windows: ESC, CTRL_C, ENTER, BACKSPACE and
    the characters typed.

    Every ESC is one ESC key (stop at once), and the rest of its sequence is
    dropped: a lone Esc and the start of an arrow key differ only in timing,
    so a rule that ignored sequences could lose a real Esc that arrives
    packed with what follows it. So on Linux arrows, Home/End, Delete,
    F-keys and Alt+key are ESC (on Windows they are ignored).

    pending: the unfinished sequence the previous call returned (its ESC
    already given as a key), so a sequence split across reads is still
    dropped whole. Returns (keys, pending). Enter as CR or LF, Backspace as
    DEL or BS. The two msvcrt prefix characters are dropped, so they can't
    make handle_key skip the next key (e.g. an Esc)."""
    text = pending + text
    keys = []
    i = 0
    while i < len(text):
        char = text[i]
        if char == ESC:
            if not (i == 0 and pending):
                keys.append(ESC)
            end, complete = _escape_end(text, i)
            if not complete:
                return keys, text[i:]
            i = end
            continue
        i += 1
        if char in (ENTER, LINE_FEED):
            keys.append(ENTER)
        elif char in (BACKSPACE, DELETE):
            keys.append(BACKSPACE)
        elif char not in SPECIAL_KEY_PREFIXES:
            keys.append(char)
    return keys, ''


class PosixKeyboard:
    """msvcrt-like keys (kbhit, getwch) from a Linux/POSIX terminal.

    Context manager: inside, the terminal doesn't echo and gives each key at
    once (no line editing). ISIG is off, so Ctrl+C arrives as a key and the
    loop quits through its own way out (Ctrl+Z can't suspend the console);
    IXON is off, so Ctrl+S can't freeze the output (and with it the loop).
    Output processing is unchanged ("\\n" still starts a new row). The saved
    settings come back on every way out of the with block, normal or
    exception; if the terminal is gone by then (SSH dropped), that is
    ignored.

    End of input (the terminal hung up) is reported as one Ctrl+C key, as is
    request_quit() (ExitSignals calls it): the loop then quits and sends its
    stops. fd: the terminal's file descriptor; clock: time-like (monotonic)."""

    def __init__(self, fd, clock=time):
        self.fd = fd
        self.clock = clock
        self.saved_attributes = None
        self.keys = []
        self.pending = ''
        self.pending_since = None
        self.ended = False
        self.decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')

    def __enter__(self):
        import termios
        self.saved_attributes = termios.tcgetattr(self.fd)
        attributes = termios.tcgetattr(self.fd)
        attributes[0] &= ~termios.IXON                                                     # iflag
        attributes[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)  # lflag
        attributes[6][termios.VMIN] = 1
        attributes[6][termios.VTIME] = 0
        # TCSANOW keeps keys typed before the start, as the Windows console
        # does; on the way out TCSAFLUSH drops unread keys (they'd go to the shell).
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        return self

    def __exit__(self, *exception):
        import termios
        if self.saved_attributes is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.saved_attributes)
            except (termios.error, OSError):
                pass                      # the terminal is gone: nothing to restore
            self.saved_attributes = None
        return False                      # never swallow the exception

    def request_quit(self):
        self.keys.insert(0, CTRL_C)

    def kbhit(self):
        if not self.keys:
            self._read()
        return bool(self.keys)

    def getwch(self):
        return self.keys.pop(0)

    def _read(self):
        if self.ended or not select.select([self.fd], [], [], 0)[0]:
            return
        try:
            data = os.read(self.fd, KEYBOARD_READ_BYTES)
        except OSError:                   # EIO: the terminal hung up
            data = b''
        if not data:
            self.ended = True
            self.keys.append(CTRL_C)
            return
        now = self.clock.monotonic()
        if self.pending and (now - self.pending_since > ESCAPE_TAIL_SECONDS
                             or len(self.pending) > MAX_PENDING_ESCAPE):
            self.pending = ''             # not the rest of that sequence: decoded as keys
        keys, pending = decode_keys(self.decoder.decode(data), self.pending)
        if pending != self.pending:
            self.pending_since = now
        self.pending = pending
        self.keys.extend(keys)


# ExitSignals (serial_logger.py): SIGHUP and SIGTERM call the keyboard's
# request_quit, so the console quits through its own way out and sends its
# stops (ESTOP x3 / "m 0" x3), as for Ctrl+C.


EXIT_DRAIN_SECONDS = 0.2       # logging goes on this long after the exit stops


def drain_after_exit(port, clock, logger):
    """After the exit stops were sent: keep logging what the board sends for
    EXIT_DRAIN_SECONDS, so the log shows the stop taking effect (the latch).
    Bounded (each read returns within READ_TIMEOUT_SECONDS) and never raises:
    a failing port, or a second Ctrl+C, ends it. Prints nothing (the logger's
    print_function is off meanwhile), so a dead or stalled terminal can't
    hold it up."""
    print_function, logger.print_function = logger.print_function, lambda text: None
    try:
        end = clock.monotonic() + EXIT_DRAIN_SECONDS
        while clock.monotonic() < end:
            logger.feed(port.read(port.in_waiting or 1), clock.time(), clock.monotonic())
    except (Exception, KeyboardInterrupt):
        pass
    finally:
        logger.print_function = print_function


def run_console(port, keyboard, clock, logger, status_line, sleep=time.sleep):
    """The console loop. Returns when Ctrl+C is pressed (as a key, or as
    KeyboardInterrupt). Always sends "m 0" before closing the port, and
    "s 0" too if steering was commanded (CommandConsole.exit_commands),
    then logs for EXIT_DRAIN_SECONDS more (drain_after_exit).

    port: pyserial-like (read, in_waiting, write, flush, close); keyboard:
    msvcrt-like (kbhit, getwch); clock: time-like (time, monotonic).
    """
    console = CommandConsole()
    status_text = logger.status_text()
    next_status_time = clock.monotonic() + STATUS_EVERY_SECONDS
    status_line.show(status_text, console, logger)
    try:
        while not console.quit_requested:
            chunk = port.read(port.in_waiting or 1)
            received_at = clock.time()  # all lines finished in this chunk get the same host time
            results = logger.feed(chunk, received_at, clock.monotonic())
            console.handle_line_results(results)
            if 'good' in results:
                console.check_text_gate(logger.last_good_line)
            now = clock.monotonic()

            redraw = bool(results)
            while keyboard.kbhit() and not console.quit_requested:
                for data in console.handle_key(keyboard.getwch(), now):
                    port.write(data)
                redraw = True
            for keepalive in console.keepalive_due(now):
                port.write(keepalive)

            if now >= next_status_time:
                logger.flush()
                status_text = logger.status_text()
                next_status_time += STATUS_EVERY_SECONDS
                redraw = True
            for message in console.take_messages():
                status_line.message(message)
                redraw = True
            if redraw:
                status_line.show(status_text, console, logger)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(EXIT_STOP_REPEATS):
                for command in console.exit_commands():
                    port.write(encode_line(command))
                sleep(EXIT_STOP_SPACING_SECONDS)
            port.flush()
            drain_after_exit(port, clock, logger)
        finally:
            port.close()
    return console


# --- Binary mode (--binary, the Pi link) ----------------------------------------------

DRIVE_PERIOD_SECONDS = 0.02      # 50 Hz: 25 DRIVEs per firmware watchdog timeout (500 ms)
REDRAW_SECONDS = 0.1             # the status line at most 10 times a second (STATE comes at 100 Hz)
EXIT_ESTOP_REPEATS = 3
BINARY_COMMAND_PATTERN = re.compile(r'^([vs]) (-?\d{1,5})$')
INJECT_PATTERN = re.compile(r'^inject (\w+) (\d{1,3})$')
# What the firmware's STATUS counters must show after "inject <kind> N" (bench step 6).
INJECT_EXPECTED = {
    'crc': 'rx_crc_errors +N, and rx_seq_gaps +1 at the next good frame',
    'cobs': 'rx_frame_errors +N, and rx_seq_gaps +1 at the next good frame',
    'length': 'rx_rejected +N (BAD_LENGTH)',
    'type': 'rx_rejected +N (UNKNOWN_TYPE)',
    'reserved': 'rx_rejected +N (BAD_FLAGS)',
    'range': 'rx_rejected +N (OUT_OF_RANGE)',
    'arm': 'rx_rejected +N (ARM_WITH_SPEED)',
}
MOTOR_STATE_NAMES = ('OFF', 'RUNNING', 'LATCHED', 'FAULT')
BINARY_HELP = ('v <cps -5000..5000>, s <-100..100>, arm, estop, hello, -v, -s, '
               'inject <crc|cobs|length|type|reserved|range|arm> <N>, or !text to send as text')


def bad_frame(kind, seq):
    """One frame the firmware must drop, of the given kind, built around a
    harmless DRIVE (speed 0, no arm) where the kind allows it. "range" asks for
    5001 cps and "arm" for 1000 cps with the arm bit: inject those only while
    the motor is latched, so even a firmware that wrongly accepted one couldn't
    move the wheel."""
    if kind == 'crc':
        body = bytes([rover_link.DRIVE, seq]) + rover_link.pack_drive(0)
        crc = rover_link.crc16(body) ^ 0x0001               # valid COBS, wrong CRC
        return b'\x00' + rover_link.cobs_encode(body + bytes([crc & 0xFF, crc >> 8])) + b'\x00'
    if kind == 'cobs':
        wire = bytearray(rover_link.encode_frame(rover_link.DRIVE, seq, rover_link.pack_drive(0)))
        wire[1] = 0xFF                                       # a 254-byte block that isn't there
        return bytes(wire)
    if kind == 'length':
        return rover_link.encode_frame(rover_link.DRIVE, seq, rover_link.pack_drive(0) + b'\x00')
    if kind == 'type':
        return rover_link.encode_frame(0x55, seq, rover_link.pack_drive(0))
    if kind == 'reserved':
        return rover_link.encode_frame(rover_link.DRIVE, seq, rover_link.pack_drive(0, extra_flags=0x04))
    if kind == 'range':
        return rover_link.encode_frame(rover_link.DRIVE, seq, rover_link.pack_drive(rover_link.MAX_SPEED_CPS + 1))
    if kind == 'arm':
        return rover_link.encode_frame(rover_link.DRIVE, seq, rover_link.pack_drive(rover_link.MIN_SPEED_CPS, arm=True))
    raise ValueError(f'unknown bad frame kind {kind!r}')


class BinaryConsole:
    """What to send and when in --binary mode, from key presses and the
    logger's events. No I/O.

    HELLO first (HelloSender: again until a HELLO_ACK, and after a reset).
    Then one DRIVE every DRIVE_PERIOD_SECONDS carries both slots: the speed
    (v, counts/s) and the steering (s, %, with steer_valid). DRIVEs start at
    the first v or s typed and only once the board has answered HELLO. They
    never re-arm the motor: only "arm" does (one DRIVE with speed 0 and the
    arm bit; the keepalive goes on at v 0). Esc / "estop" send ESTOP (the
    motor latches at once and stays latched until "arm"). There is no "m" in
    binary mode."""

    def __init__(self, hello_delay=0.0):
        self.sequence = Sequence()
        self.hello = HelloSender(self.sequence, first_delay=hello_delay)
        self.speed_cps = None          # None: no DRIVE keepalive
        self.steer_percent = None      # None: DRIVEs without steer_valid
        self.next_drive_time = None    # None: due at once
        self.typed_text = ''
        self.quit_requested = False
        self.skip_next_key = False
        self.messages = []

    @property
    def acknowledged(self):
        return not self.hello.pending

    @property
    def sending(self):
        return self.speed_cps is not None or self.steer_percent is not None

    def _drive(self, arm=False):
        speed = 0 if arm or self.speed_cps is None else self.speed_cps
        return rover_link.encode_frame(
            rover_link.DRIVE, self.sequence.next(),
            rover_link.pack_drive(speed, self.steer_percent or 0, self.steer_percent is not None, arm))

    def _estop(self):
        return rover_link.encode_frame(rover_link.ESTOP, self.sequence.next())

    def _drive_now(self, now, arm=False):
        """A DRIVE at once (if the board has answered HELLO), and the keepalive from now."""
        if not self.acknowledged:
            self.messages.append('no HELLO_ACK yet: DRIVEs start once the board answers')
            return []
        self.next_drive_time = now + DRIVE_PERIOD_SECONDS
        return [self._drive(arm)]

    def exit_frames(self):
        """Sent (repeatedly) on the way out: ESTOP, so the motor latches at once and stays latched."""
        return [self._estop()]

    def handle_key(self, key, now):
        """One key press. Returns the bytes to send right away (a list)."""
        if self.skip_next_key:
            self.skip_next_key = False
            return []
        if key in SPECIAL_KEY_PREFIXES:
            self.skip_next_key = True
            return []
        if key == CTRL_C:
            self.quit_requested = True
            return []
        if key == ESC:
            self.typed_text = ''
            return self._stop_now(now, centre=True)
        if key == BACKSPACE:
            self.typed_text = self.typed_text[:-1]
            return []
        if key == ENTER:
            text, self.typed_text = self.typed_text.strip(), ''
            return self._command(text, now) if text else []
        if key.isprintable():
            self.typed_text += key
        return []

    def _stop_now(self, now, centre):
        """ESTOP, then the keepalive goes on at v 0 (and s 0 if steering was commanded and centre)."""
        frames = [self._estop()]
        self.speed_cps = 0 if self.sending else None
        if centre and self.steer_percent is not None:
            self.steer_percent = 0
        self.messages.append('ESTOP: latched until "arm"' + (', s 0' if centre and self.steer_percent == 0 else ''))
        if self.sending and self.acknowledged:
            frames += self._drive_now(now)
        return frames

    def _command(self, text, now):
        if text.startswith(RAW_SEND_PREFIX):
            self.messages.append(f'sent once, as text: {text[1:]!r} (binary mode ignores text, '
                                 'except e, c and h in a test build)')
            return [encode_line(text[1:])]
        if text == 'arm':
            self.speed_cps = 0
            self.messages.append('arm: DRIVE 0 with the arm bit, then v 0 keepalives')
            return self._drive_now(now, arm=True)
        if text == 'estop':
            return self._stop_now(now, centre=False)
        if text == 'hello':
            self.hello.pending, self.hello.next_time = True, None
            self.messages.append('HELLO again (the firmware latches the motor)')
            return []
        if text == '-v':
            self.speed_cps = self.steer_percent = None
            self.messages.append('-v: stopped sending DRIVEs; the motor watchdog latches within 500 ms '
                                 '(and the steering centres: it rides in the same DRIVE)')
            return []
        if text == '-s':
            self.steer_percent = None
            self.messages.append('-s: DRIVEs go on without steer_valid; the steering centres within 500 ms')
            return []
        inject = INJECT_PATTERN.match(text)
        if inject and inject.group(1) in INJECT_EXPECTED:
            kind, count = inject.group(1), int(inject.group(2))
            self.messages.append(f'inject {kind} x{count}: expect {INJECT_EXPECTED[kind].replace("N", str(count))}')
            return [bad_frame(kind, self.sequence.next()) for _ in range(count)]
        if COMMAND_PATTERN.match(text) and text.startswith('m'):
            self.messages.append('not sent: no "m" in binary mode (open loop is a bench text-mode command); '
                                 'use v <counts/s>')
            return []
        command = BINARY_COMMAND_PATTERN.match(text)
        if command:
            letter, value = command.group(1), int(command.group(2))
            limit = rover_link.MAX_SPEED_CPS if letter == 'v' else rover_link.MAX_STEER_PERCENT
            if abs(value) > limit:
                self.messages.append(f'not sent: {letter} {value} is beyond +-{limit} (the firmware would reject it)')
                return []
            if letter == 'v':
                self.speed_cps = value
            else:
                self.steer_percent = value
            return self._drive_now(now)
        self.messages.append(f'not sent: {text!r} ({BINARY_HELP})')
        return []

    def handle_events(self, events, now):
        """The logger's events: HELLO_ACK, board reset (sending stops), text mode."""
        handle_hello_events(events, self.hello, now, self.messages.append)
        if any(event[0] == 'boot' for event in events):
            self.speed_cps = self.steer_percent = None
            self.messages.append('sending stopped until you type again (the motor is latched until "arm")')

    def keepalive_due(self, now):
        """HELLO while unanswered, then the DRIVE keepalive (a list of bytes)."""
        due = self.hello.due(now)
        if self.acknowledged and self.sending and (self.next_drive_time is None or now >= self.next_drive_time):
            next_time = (self.next_drive_time or now) + DRIVE_PERIOD_SECONDS
            self.next_drive_time = next_time if next_time > now else now + DRIVE_PERIOD_SECONDS
            due.append(self._drive())
        return due

    def take_messages(self):
        messages, self.messages = self.messages, []
        return messages


def _name(names, code):
    return names[code] if 0 <= code < len(names) else f'{code}?'


def binary_status_line_text(console, logger, max_length):
    """The status line in binary mode, at most max_length characters:
      HELLO.. | sent: v 2500 s 20 | RUNNING duty=320 vest=2450 cnt=12345 ep=0 | run vset=2500 vi=307
      | s=20 us=1620 sst=1 | vbat=7.92V bst=1 | rx ok=812 crc=0 fr=0 rej=0 gap=0 txt=0 | loop=702us
      | frames=.. bad=.. | > typed
    with TEST BUILD, IMU FAIL, EST BAD, LATCHED (reason) and FAULT (reason)
    where they apply. Too wide: the logger counts go first, then loop, rx and
    what is sent; then it is cut from the left."""
    state, status, ack = logger.last_state, logger.last_status, logger.hello_ack
    link = 'bin' if console.acknowledged else 'HELLO..'
    sending = ' '.join(part for part in (
        f'v {console.speed_cps}' if console.speed_cps is not None else '',
        f's {console.steer_percent}' if console.steer_percent is not None else '') if part) or '-'
    head = [link]
    if ack and ack['build_flags']:
        head.append(f'TEST BUILD {rover_link.build_flag_text(ack["build_flags"])}')
    body = []
    if state:
        motor = _name(MOTOR_STATE_NAMES, state['motor_state'])
        if status and state['motor_state'] == 2:
            motor += f' ({_name(rover_link.LATCH_REASON_NAMES, status["latch_reason"])})'
        if status and state['motor_state'] == 3:
            motor += f' ({_name(MOTOR_FAULT_NAMES, status["fault_reason"])})'
        flags = ('' if state['enc_flags'] & rover_link.ENC_VALID_NOW else ' ' + EST_BAD_MARKER) + \
                ('' if state['imu_flags'] & rover_link.IMU_OK else ' ' + IMU_MISSING_MARKER)
        body.append(f"{motor} duty={state['duty']} vest={state['speed_cps']} cnt={state['count']} "
                    f"ep={state['odometer_epoch']}{flags}")
    else:
        body.append('no STATE yet')
    rx = loop = ''
    if status:
        ctl = _name(SPEED_CTL_NAMES, status['control_state'])
        body.append(f"{ctl} vset={status['setpoint_cps']} vi={status['integrator_ticks']}")
        body.append(f"s={status['steer_percent']} us={status['steer_pulse_us']} sst={status['steer_state']}")
        centivolts = (status['vbat_mv'] + 5) // 10
        battery = f"vbat={centivolts // 100}.{centivolts % 100:02d}V bst={status['battery_state']}"
        marker = BATTERY_STATE_MARKERS.get(status['battery_state'])
        body.append(f'{marker} {battery}' if marker else battery)
        if status['locked_stalls'] or status['saturation_stalls']:
            body.append(f"stalls lock={status['locked_stalls']} sat={status['saturation_stalls']}")
        rx = (f"rx ok={status['rx_ok']} crc={status['rx_crc_errors']} fr={status['rx_frame_errors']} "
              f"rej={status['rx_rejected']} gap={status['rx_seq_gaps']} txt={status['rx_text_ignored']}")
        loop = f"loop={status['loop_us_max']}us"
    counts = logger.status_text()
    typed = f'> {console.typed_text}'

    def build(show_counts, show_loop, show_rx, show_sent):
        parts = head + ([f'sent: {sending}'] if show_sent else []) + body
        parts += [part for part, show in ((rx, show_rx), (loop, show_loop), (counts, show_counts)) if part and show]
        return ' | '.join(parts + [typed])

    variants = (build(True, True, True, True), build(False, True, True, True), build(False, False, True, True),
                build(False, False, False, True), build(False, False, False, False))
    for text in variants:
        if len(text) <= max_length:
            return text
    return variants[-1][-max_length:]


def run_binary_console(port, keyboard, clock, logger, status_line, sleep=time.sleep,
                       hello_delay=HELLO_START_DELAY_SECONDS):
    """The --binary loop. Returns when Ctrl+C is pressed (as a key, or as
    KeyboardInterrupt). Always sends ESTOP (EXIT_ESTOP_REPEATS times) before
    closing the port, then logs for EXIT_DRAIN_SECONDS more. Same port /
    keyboard / clock as run_console. The first HELLO goes out hello_delay
    after the start (what arrives before it is the bridge's stale leftover)."""
    console = BinaryConsole(hello_delay)
    logger.expect_hello()
    next_redraw_time = clock.monotonic()
    status_line.show_text(binary_status_line_text(console, logger, status_line.max_length()))
    try:
        while not console.quit_requested:
            now = clock.monotonic()
            frames = console.keepalive_due(now)
            if console.hello.sent_count:
                logger.mark_hello_sent()
            for frame in frames:
                port.write(frame)
            chunk = port.read(port.in_waiting or 1)
            events = logger.feed(chunk, clock.time(), clock.monotonic())
            now = clock.monotonic()
            console.handle_events(events, now)
            redraw = now >= next_redraw_time
            while keyboard.kbhit() and not console.quit_requested:
                for data in console.handle_key(keyboard.getwch(), now):
                    port.write(data)
                redraw = True
            for message in console.take_messages():
                status_line.message(message)
                redraw = True
            if redraw:
                status_line.show_text(binary_status_line_text(console, logger, status_line.max_length()))
                next_redraw_time = now + REDRAW_SECONDS
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(EXIT_ESTOP_REPEATS):
                for frame in console.exit_frames():
                    port.write(frame)
                sleep(EXIT_STOP_SPACING_SECONDS)
            port.flush()
            drain_after_exit(port, clock, logger)
        finally:
            port.close()
    return console


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='serial port, e.g. COM5 or /dev/ttyACM0')
    parser.add_argument('output', help='file for the raw bytes; timestamps go to <output>.times.csv')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--binary', action='store_true', help='Pi link binary mode (see "Binary mode" above)')
    args = parser.parse_args()

    windows = sys.platform == 'win32'
    if not windows and not sys.stdin.isatty():
        print(NOT_A_TERMINAL_NOTE)
        return 2
    import serial

    def write(text):
        sys.stdout.write(text)
        sys.stdout.flush()

    status_line = StatusLine(write)
    if windows:
        import msvcrt
        keyboard_context, signals = contextlib.nullcontext(msvcrt), contextlib.nullcontext()
        port_options = {}
    else:
        keyboard = PosixKeyboard(sys.stdin.fileno())
        # Signals first, so a hangup while the terminal is being set up already quits cleanly.
        keyboard_context, signals = keyboard, ExitSignals(keyboard.request_quit)
        port_options = {'exclusive': True}   # POSIX opens are shared by default; Windows' never are
    # Outermost, so the terminal (Linux) or the console mode (Windows) is
    # restored whatever fails inside (even opening the port).
    with signals, keyboard_context as keyboard, \
         (QuickEditOff(load_kernel32(), status_line.message) if windows else contextlib.nullcontext()):
        port = serial.Serial(args.port, args.baud, timeout=READ_TIMEOUT_SECONDS, **port_options)
        # The loops never write the files themselves: a writer thread does (serial_logger.QueuedWriter),
        # so a slow disk can't hold up the keepalives.
        with open(args.output, 'wb') as raw_file, \
             open(args.output + '.times.csv', 'w', encoding='ascii', newline='') as times_file, \
             queued_writer(raw_file) as raw, queued_writer(times_file) as times:
            if args.binary:
                logger = BinaryLogger(raw, times, print_function=status_line.message)
                run_binary_console(port, keyboard, time, logger, status_line)
            else:
                logger = TelemetryLogger(raw, times, print_function=status_line.message)
                console = run_console(port, keyboard, time, logger, status_line)
    writers = (('the raw log', raw), ('the times file', times))

    received = getattr(signals, 'received', None)
    if args.binary:
        sent = 'ESTOP'
    else:
        sent = ' and '.join(f'"{command}"' for command in console.exit_commands())
    try:
        if received:
            print(f'\n{received}: stopped as for Ctrl+C.', end='')
        print(f'\nSent {sent} x{EXIT_ESTOP_REPEATS if args.binary else EXIT_STOP_REPEATS} and closed {args.port}.')
        print(logger.summary_text())
        print_writer_problems(writers)
    except OSError:
        pass                              # the terminal is gone (SSH dropped): the log files are complete
    return 1 if any(writer.problem_text() for _, writer in writers) else logger.exit_code()


if __name__ == '__main__':
    sys.exit(main())
