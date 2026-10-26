"""The bridge's safety state machine (group 3). Pure: no ROS, no I/O, an injected clock (now
arguments, monotonic seconds); the caller holds no lock, this class has its own.

What it decides is what the TX thread sends, 50 times a second (tick()):

  DISARMED   nothing at all (no DRIVE: the firmware's motor stays latched, and its 500 ms
             watchdog would latch it anyway)
  ARMING     one DRIVE 0 with the arm bit (the only thing that re-arms the firmware; it
             takes a LATCHED motor to OFF whatever the latch reason, motor_control.c), then
             DRIVE 0 until a STATUS shows the motor no longer LATCHED -> ARMED; not within
             ARM_TIMEOUT (0.5 s, before the heartbeat's 1 s) -> ESTOP x3 and DISARMED
  ARMED      a DRIVE every tick: the speed and the steering from the latest /cmd_vel
             (steer_valid on every DRIVE, so the servo holds what it is sent)

Operator heartbeat (Pi receive time of /cmd_vel; it covers the Wi-Fi drop the firmware's
watchdog can't see, because the Pi keeps sending DRIVEs):
  older than HEARTBEAT_STOP (0.3 s)   DRIVE 0 with the steering centred, still armed: a new
                                      /cmd_vel drives again
  older than HEARTBEAT_ESTOP (1 s)    ESTOP x3 and DISARMED: driving needs /rover/arm again
The /cmd_vel that ends a stop is an event too ("/cmd_vel back after N ms"). Since the last arm,
while ARMING / ARMED: the number of stops, the longest /cmd_vel gap (an ESTOP's included), and the
last stop's whole gap (from the /cmd_vel before it to the one that ended it), and the gaps of
SLOW_GAP_SECONDS (150 ms) or more, stops included. One of those that didn't stop the wheel is an
event too ("/cmd_vel gap 251 ms (no stop)"), so a slow link shows up with a Pi time before it
costs a stop (2026-10-19, Wi-Fi: 251 ms, then 433 and 341 ms stops).

arm(force) refuses (with the reason) unless: the board answered HELLO, it is a normal build
(build_flags 0), a STATUS is fresh, the motor isn't FAULT, the battery is OK (LOW, CUTOFF, no
pack, ADC fault, removed: refused, so driving needs the LiPo), the heartbeat is fresh, and the
latch reason is one arm may clear: BOOT, HELLO, ESTOP, WATCHDOG (or the motor isn't latched).
STALL_LOCKED, STALL_SATURATION and ESTIMATE_INVALID need force (the /rover/arm_force service).

ESTOP x3 (on three consecutive ticks) and DISARMED on: estop(), the heartbeat's 1 s, and
shutdown() (every way out of the node, before anything is torn down). After shutdown() nothing
but those ESTOPs is ever sent. The firmware latching by itself while armed (a STATUS showing
LATCHED: watchdog, stall, battery) or a board reset (boot line, a new HELLO_ACK) -> DISARMED.

Speed: /cmd_vel linear.x (m/s) / meters_per_encoder_tick -> counts/s, rounded; |cps| below
MIN_SPEED_CPS (1000, the firmware coasts below it) -> 0, counted, never rounded up; beyond
MAX_SPEED_CPS (5000) -> clamped, counted; not a finite number -> 0, counted.

Steering (steer_percent): the bicycle model, delta = atan(wheelbase * angular.z / linear.x),
clamped to +/- max_steering_angle (config/vehicle.cfg; counted), then percent = delta /
max_steering_angle * 100, rounded. ROS angular.z > 0 is CCW (left), and the firmware's s > 0 is
left (REP 103): the sign passes straight through. Reversing (linear.x < 0) flips it by itself,
which is the bicycle model's answer: CCW while backing up is steering right. Percent -> angle is
treated as linear; only full lock was measured (2026-10-22).
  - a /cmd_vel whose speed is sent as 0 (0, or the coast region below MIN_SPEED_CPS): the last
    steering is kept (a car can't turn in place; the wheels don't flick straight at each pause),
    counted as 'steering held' when angular.z asks for a turn
  - linear.x or angular.z not a finite number: centre, counted
  - centred (the held value reset to 0) when arming starts, on every disarm or ESTOP, and on a
    heartbeat stop; ARMING and the heartbeat stop send 0 % explicitly
  - after an ESTOP or a disarm nothing is sent (as before): the firmware's own steering timeout
    (500 ms without a DRIVE with steer_valid) centres it. An ESTOP latches only the motor.

Events (arming, armed, refused, ESTOP, disarmed, heartbeat stop): each one is queued, and
take_events() hands them to the node to log, every one, not only when the text changes (a second
heartbeat stop with the same text went unlogged on the 2026-10-19 bench). Only the heartbeat stop
and resume (and a slow /cmd_vel gap), which can repeat several times a second, are limited to
one line each per EVENT_LOG_SECONDS. What was held back is said within about a second: by the next
line of that kind if one comes in time, else by a line of its own once the second is over. The
counters count every one.
"""
import collections
import math
import threading

from rover_bridge import _tools  # noqa: F401
import rover_link  # noqa: E402

from rover_bridge import battery  # noqa: E402

HEARTBEAT_STOP = 0.3
HEARTBEAT_ESTOP = 1.0
SLOW_GAP_SECONDS = 0.15                # 20 Hz is 50 ms between messages
ARM_TIMEOUT = 0.5
STATUS_FRESH = 0.5
ESTOP_REPEATS = 3
CENTRE = 0                             # steering %, sent explicitly
MIN_SPEED_CPS = rover_link.MIN_SPEED_CPS
MAX_SPEED_CPS = rover_link.MAX_SPEED_CPS
LATCHED, FAULT = 2, 3
# MotorLatchReason (motor_control.h): 0 none, 1 BOOT, 2 WATCHDOG, 3 STALL_LOCKED, 4 STALL_SATURATION,
# 5 ESTIMATE_INVALID, 6 ESTOP, 7 HELLO.
ARM_CLEARS = {0, 1, 2, 6, 7}
ARM_NEEDS_FORCE = {3, 4, 5}
DISARMED, ARMING, ARMED = 'DISARMED', 'ARMING', 'ARMED'
EVENT_LOG_SECONDS = 1.0
RATE_LIMITED = {'heartbeat stop', 'heartbeat resume', 'slow gap'}
WARN_EVENTS = {'refused', 'estop', 'heartbeat stop'}
EVENTS_KEPT = 100                      # not yet taken; the node takes them 100 times a second


def speed_cps(meters_per_second, meters_per_tick):
    """(cps, what): what is '' or 'invalid', 'below minimum', 'clamped'."""
    if not math.isfinite(meters_per_second):
        return 0, 'invalid'
    cps = round(meters_per_second / meters_per_tick)
    if cps != 0 and abs(cps) < MIN_SPEED_CPS:
        return 0, 'below minimum'
    if abs(cps) > MAX_SPEED_CPS:
        return int(math.copysign(MAX_SPEED_CPS, cps)), 'clamped'
    return cps, ''


def steer_percent(linear_x, angular_z, wheelbase, max_angle):
    """(percent, what) for a moving command (finite, linear_x != 0): what is '' or 'clamped'."""
    delta = math.atan(wheelbase * angular_z / linear_x)
    what = ''
    if abs(delta) > max_angle:
        delta, what = math.copysign(max_angle, delta), 'clamped'
    return round(delta / max_angle * 100), what


class Safety:

    def __init__(self, meters_per_tick, wheelbase, max_steering_angle):
        self.meters_per_tick = meters_per_tick
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        self.lock = threading.Lock()
        self.state = DISARMED
        self.hello_ack = None
        self.status = None
        self.status_at = None
        self.command_cps = 0
        self.command_steer = CENTRE          # %, from /cmd_vel (held while the speed is 0)
        self.drive_cps = None                # the speed of the DRIVE being sent; None: no DRIVE
        self.drive_steer = None              # its steering %; None: no DRIVE
        self.heartbeat_at = None
        self.arming_since = None
        self.arm_bit_pending = False
        self.estops_left = 0
        self.shutting_down = False
        self.counts = {'below minimum': 0, 'clamped': 0, 'invalid': 0, 'steering clamped': 0,
                       'steering held': 0, 'steering invalid': 0, 'estops': 0,
                       'heartbeat stops': 0, 'heartbeat estops': 0, 'arms': 0, 'arm refusals': 0}
        self.events = collections.deque(maxlen=EVENTS_KEPT)      # (kind, text), not yet taken
        self.logged = {}                                          # rate-limited kind -> (logged at, held back)
        self.stopped_for_heartbeat = False
        self.stops_since_arm = 0
        self.slow_gaps_since_arm = 0
        self.gap_max = None                  # s, since the last arm
        self.last_stop_gap = None            # s, since the last arm
        self.last_event_at = None              # monotonic s; None: at start
        self._event('started', 'started: DISARMED', None)

    # --- inputs ---

    def on_cmd_vel(self, linear_x, angular_z, now):
        with self.lock:
            cps, what = speed_cps(linear_x, self.meters_per_tick)
            if what:
                self.counts[what] += 1
            if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
                self.counts['steering invalid'] += 1
                self.command_steer = CENTRE
            elif cps == 0:
                if angular_z != 0.0:
                    self.counts['steering held'] += 1
            else:
                self.command_steer, steer_what = steer_percent(linear_x, angular_z, self.wheelbase,
                                                               self.max_steering_angle)
                if steer_what:
                    self.counts['steering ' + steer_what] += 1
            self.command_cps = cps
            if self.state in (ARMING, ARMED) and self.heartbeat_at is not None:
                gap = now - self.heartbeat_at
                self.gap_max = gap if self.gap_max is None else max(self.gap_max, gap)
                if gap >= SLOW_GAP_SECONDS:
                    self.slow_gaps_since_arm += 1
                if self.stopped_for_heartbeat:
                    self.stopped_for_heartbeat = False
                    self.last_stop_gap = gap
                    self._event('heartbeat resume', f'/cmd_vel back after {gap * 1000:.0f} ms: driving again', now)
                elif gap >= SLOW_GAP_SECONDS:
                    self._event('slow gap', f'/cmd_vel gap {gap * 1000:.0f} ms (no stop)', now)
            self.heartbeat_at = now

    def on_hello_ack(self, fields, now):
        with self.lock:
            if self.hello_ack is not None:                    # a second one: the board was reset
                self._disarm('board reset (new HELLO_ACK): motor latched by HELLO', now)
            self.hello_ack = fields

    def on_board_reset(self, now):
        with self.lock:
            self.hello_ack = None
            self.status = None
            self._disarm('board reset (boot line)', now)

    def on_status(self, fields, now):
        with self.lock:
            self.status, self.status_at = fields, now
            if self.state == ARMING and fields['motor_state'] != LATCHED and not self.arm_bit_pending:
                self.state = ARMED
                self._event('armed', 'armed: motor ' + ('OFF' if fields['motor_state'] == 0 else str(fields['motor_state'])), now)
            elif self.state == ARMED and fields['motor_state'] in (LATCHED, FAULT):
                reason = rover_link.LATCH_REASON_NAMES[fields['latch_reason']] \
                    if fields['latch_reason'] < len(rover_link.LATCH_REASON_NAMES) else fields['latch_reason']
                self._disarm(f'the board latched the motor ({reason})' if fields['motor_state'] == LATCHED
                             else 'the board locked the motor out (FAULT)', now)

    # --- requests ---

    def arm(self, now, force=False):
        """(accepted, message). Accepted means the arm DRIVE goes out; ARMED follows once a STATUS
        shows the motor no longer latched (within ARM_TIMEOUT)."""
        with self.lock:
            reason = self._arm_refusal(now, force)
            if reason:
                self.counts['arm refusals'] += 1
                self._event('refused', f'arm refused: {reason}', now)
                return False, reason
            if self.state == ARMED:
                return True, 'already armed'
            self.state = ARMING
            self.arming_since = now
            self.command_steer = CENTRE
            self.arm_bit_pending = True
            self.stopped_for_heartbeat = False
            self.stops_since_arm, self.gap_max, self.last_stop_gap = 0, None, None
            self.slow_gaps_since_arm = 0
            self.counts['arms'] += 1
            self._event('arming', 'arming' + (' (forced)' if force else ''), now)
            return True, 'arming: DRIVE 0 with the arm bit sent; driving once the board reports the motor OFF'

    def _arm_refusal(self, now, force):
        if self.shutting_down:
            return 'the bridge is shutting down'
        if self.hello_ack is None:
            return 'no HELLO_ACK from the board yet'
        if self.hello_ack['build_flags']:
            return f"test build ({rover_link.build_flag_text(self.hello_ack['build_flags'])}): never drive the truck with it"
        if self.status is None or now - self.status_at > STATUS_FRESH:
            return 'no fresh STATUS from the board'
        if self.status['motor_state'] == FAULT:
            return 'motor FAULT (locked out until a board reset)'
        state = self.status['battery_state']
        if state != battery.OK:
            return f'battery {battery.state_name(state)}: arming needs a pack above the 7.0 V warning'
        if self.heartbeat_at is None or now - self.heartbeat_at > HEARTBEAT_STOP:
            return 'no fresh /cmd_vel (heartbeat): start the operator first'
        if self.status['motor_state'] == LATCHED:
            reason = self.status['latch_reason']
            if reason in ARM_NEEDS_FORCE and not force:
                name = rover_link.LATCH_REASON_NAMES[reason]
                return f'latched by {name}: check the truck, then /rover/arm_force'
            if reason not in ARM_CLEARS | ARM_NEEDS_FORCE:
                return f'latch reason {reason} unknown'
        return ''

    def estop(self, now, why='estop requested'):
        with self.lock:
            self._estop(why, now)

    def shutdown(self, now):
        """Every way out: ESTOP x3 (whatever the state) and nothing after it."""
        with self.lock:
            self.shutting_down = True
            self._estop('bridge shutting down', now)

    # --- 50 Hz: what to send ---

    def tick(self, now):
        """The frames to send now: [('ESTOP',)] or [('DRIVE', speed_cps, steer_percent, arm)] or []."""
        with self.lock:
            frames = self._frames(now)
            drive = bool(frames) and frames[0][0] == 'DRIVE'
            self.drive_cps = frames[0][1] if drive else None
            self.drive_steer = frames[0][2] if drive else None
            return frames

    def _frames(self, now):
        if self.estops_left:
            self.estops_left -= 1
            return [('ESTOP',)]
        if self.shutting_down or self.hello_ack is None:
            return []
        age = None if self.heartbeat_at is None else now - self.heartbeat_at
        if self.state in (ARMING, ARMED) and (age is None or age > HEARTBEAT_ESTOP):
            self.counts['heartbeat estops'] += 1
            if age is not None:
                self.gap_max = age if self.gap_max is None else max(self.gap_max, age)
            self._estop(f'no /cmd_vel for {HEARTBEAT_ESTOP:.0f} s (heartbeat)', now)
            self.estops_left -= 1
            return [('ESTOP',)]
        if self.state == ARMING:
            if now - self.arming_since > ARM_TIMEOUT:
                # ESTOP, not just silence: if the board did arm and only its STATUS was lost,
                # this latches it now instead of after the watchdog's 500 ms.
                self._estop('arm timed out: the board still reports the motor latched', now)
                self.estops_left -= 1
                return [('ESTOP',)]
            arm, self.arm_bit_pending = self.arm_bit_pending, False
            return [('DRIVE', 0, CENTRE, arm)]
        if self.state == ARMED:
            if age > HEARTBEAT_STOP:
                if not self.stopped_for_heartbeat:
                    self.counts['heartbeat stops'] += 1
                    self.stops_since_arm += 1
                    self.stopped_for_heartbeat = True
                    self._event('heartbeat stop',
                                f'no /cmd_vel for {HEARTBEAT_STOP * 1000:.0f} ms: DRIVE 0 (still armed)', now)
                self.command_steer = CENTRE
                return [('DRIVE', 0, CENTRE, False)]
            return [('DRIVE', self.command_cps, self.command_steer, False)]
        return []

    # --- helpers (lock held) ---

    def _estop(self, why, now):
        self.estops_left = ESTOP_REPEATS
        self.counts['estops'] += 1
        self.state = DISARMED
        self.arm_bit_pending = False
        self.command_steer = CENTRE
        self._event('estop', f'ESTOP x{ESTOP_REPEATS}: {why}', now)

    def _disarm(self, why, now):
        if self.state != DISARMED:
            self._event('disarmed', f'disarmed: {why}', now)
        self.state = DISARMED
        self.arm_bit_pending = False
        self.command_steer = CENTRE

    def _event(self, kind, text, now):
        self.last_event, self.last_event_at = text, now
        self.events.append((kind, text))

    def take_events(self, now):
        """The events since the last call, as (warn, text) log lines, in order: every one, except
        that a heartbeat stop or resume is logged at most once per EVENT_LOG_SECONDS (each); the
        next one of that kind logged says how many were held back since the previous line."""
        with self.lock:
            lines = []
            while self.events:
                kind, text = self.events.popleft()
                if kind in RATE_LIMITED:
                    logged_at, held = self.logged.get(kind, (None, 0))
                    if logged_at is not None and now - logged_at < EVENT_LOG_SECONDS:
                        self.logged[kind] = (logged_at, held + 1)
                        continue
                    self.logged[kind] = (now, 0)
                    if held:
                        text += f' (+{held} more in the previous {now - logged_at:.1f} s, not logged)'
                lines.append((kind in WARN_EVENTS, text))
            for kind, (logged_at, held) in sorted(self.logged.items()):
                if held and now - logged_at >= EVENT_LOG_SECONDS:     # the second is over: say it now
                    self.logged[kind] = (logged_at, 0)
                    lines.append((kind in WARN_EVENTS,
                                  f'+{held} more "{kind}" in the {now - logged_at:.1f} s after the last such line, '
                                  f'not logged (counted)'))
            return lines

    def snapshot(self, now):
        with self.lock:
            age = None if self.heartbeat_at is None else now - self.heartbeat_at
            return {'state': self.state, 'heartbeat_age': age, 'command_cps': self.command_cps,
                    'command_steer': self.command_steer, 'drive_steer': self.drive_steer,
                    'counts': dict(self.counts), 'last_event': self.last_event,
                    'last_event_age': None if self.last_event_at is None else now - self.last_event_at,
                    'shutting_down': self.shutting_down, 'estops_pending': self.estops_left,
                    'stops_since_arm': self.stops_since_arm, 'gap_max': self.gap_max,
                    'slow_gaps_since_arm': self.slow_gaps_since_arm,
                    'last_stop_gap': self.last_stop_gap, 'drive_cps': self.drive_cps}
