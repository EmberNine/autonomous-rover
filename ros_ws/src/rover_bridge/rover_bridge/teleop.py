"""Keyboard teleop (group 4), the pure part: keys -> /cmd_vel setpoint, and the keyboard-link
deadman. No ROS, no I/O, an injected clock (now arguments, monotonic seconds). teleop_node.py
runs it on the Pi (inside the bridge's container); tools/teleop_keys.py on the laptop sends the
keys, and a keepalive byte every 100 ms, over SSH (docs/ros-bridge.md, Teleop).

Keys (one byte each, from teleop_keys.py; anything else is counted and ignored):
  w / s    speed up / down a step: SPEED_LEVELS, 0 -> 0.05 m/s (skipping the firmware's coast
           band below 0.039 m/s, 1000 cps), then 0.025 steps; forward up to 0.15, reverse to -0.10
  a / d    steering a STEER_STEP (10 %) of full lock to the left / right, up to +/-100 %
  c        steering centre
  space    speed 0 and steering centre
  x        speed 0 and centre, then /rover/estop (the forwarder sends it for Esc too)
  r        /rover/arm
  q        quit: speed 0, /rover/estop, stop publishing (the forwarder sends it for Ctrl+C)
  .        keepalive: only feeds the deadman

Published: linear.x = v, angular.z = v * tan(delta) / wheelbase, delta = steer % / 100 * full
lock (config/vehicle.cfg). The bridge's atan(wheelbase * angular.z / linear.x) gives back delta,
so the percent shown is the percent the servo gets, at any speed. a always means wheels left:
angular.z > 0 going forward, < 0 reversing. At v = 0, angular.z is 0 and the bridge holds its
last steering: a steering change while stopped (c and space included) reaches the wheels once
the truck moves.

Deadman (the keyboard link: SSH over Wi-Fi). The teleop runs on the Pi, next to the bridge, so
a dropped laptop doesn't stop /cmd_vel and the bridge's heartbeat never fires: this is what
stops the truck. Every byte (key or keepalive) feeds it.
  no byte for LINK_STOP_SECONDS (0.5 s)    speed 0 and centre, still publishing (still armed).
                                            Latched: bytes coming back don't drive again, w does.
  no byte for LINK_ESTOP_SECONDS (2 s)     /rover/estop, stop publishing, exit
  end of input (EOF: ssh or docker gone)   the same, at once
The 0.5 s stage will trip now and then on this Wi-Fi (10 min on 2026-10-21: longest steady-state
gap 592 ms; 4 gaps of 500-999 ms, 26 of 300-499 ms): safe, the truck stops and stays armed, w
drives again.
"""
import collections
import math

SPEED_LEVELS = (-0.10, -0.075, -0.05, 0.0, 0.05, 0.075, 0.10, 0.125, 0.15)   # m/s
STEER_STEP = 10                        # % of full lock
STEER_LIMIT = 100
LINK_STOP_SECONDS = 0.5
LINK_ESTOP_SECONDS = 2.0
KEEPALIVE = '.'
KEYS = 'wsadc xrq'
GAP_WINDOW_SECONDS = 10.0             # the status line's 'max': the longest gap in this window


def angular_z(speed, steer_percent, wheelbase, max_angle):
    """The /cmd_vel angular.z that makes the bridge steer steer_percent at this speed."""
    return speed * math.tan(steer_percent / 100 * max_angle) / wheelbase


class Teleop:

    def __init__(self, wheelbase, max_angle, now):
        self.wheelbase = wheelbase
        self.max_angle = max_angle
        self.level = SPEED_LEVELS.index(0.0)
        self.steer_percent = 0
        self.last_byte_at = now              # the deadman counts from the start too
        self.link_stopped = False
        self.finished = None                 # why, once quitting: nothing is published after that
        self.counts = {'keys': 0, 'keepalives': 0, 'ignored': 0, 'link stops': 0}
        self.gap_max = 0.0                   # s, between bytes, since the start
        self.gaps = collections.deque()      # (at, gap) in the last GAP_WINDOW_SECONDS
        self.events = []                     # log lines, not yet taken

    @property
    def speed(self):
        return SPEED_LEVELS[self.level]

    def twist(self):
        """(linear.x, angular.z) to publish now, or None once finished (publish nothing)."""
        if self.finished is not None:
            return None
        return self.speed, angular_z(self.speed, self.steer_percent, self.wheelbase, self.max_angle)

    # --- inputs: each returns the requests for the node: 'arm', 'estop' ---

    def on_bytes(self, text, now):
        requests = []
        for char in text:
            if self.finished is not None:
                break
            gap = now - self.last_byte_at
            self.gap_max = max(self.gap_max, gap)
            self.gaps.append((now, gap))
            self.last_byte_at = now
            if self.link_stopped:
                self.link_stopped = False
                self.events.append(f'keyboard link back after {gap * 1000:.0f} ms: speed stays 0 (w to drive)')
            if char == KEEPALIVE:
                self.counts['keepalives'] += 1
                continue
            if char not in KEYS:
                self.counts['ignored'] += 1
                continue
            self.counts['keys'] += 1
            requests += self._key(char)
        return requests

    def _key(self, char):
        if char == 'w':
            self.level = min(self.level + 1, len(SPEED_LEVELS) - 1)
        elif char == 's':
            self.level = max(self.level - 1, 0)
        elif char == 'a':
            self.steer_percent = min(self.steer_percent + STEER_STEP, STEER_LIMIT)
        elif char == 'd':
            self.steer_percent = max(self.steer_percent - STEER_STEP, -STEER_LIMIT)
        elif char == 'c':
            self.steer_percent = 0
        elif char == ' ':
            self._stop()
        elif char == 'x':
            self._stop()
            self.events.append('x: speed 0, centre, /rover/estop')
            return ['estop']
        elif char == 'r':
            return ['arm']
        elif char == 'q':
            return self._finish('quit (q)')
        return []

    def on_eof(self, now):
        return self._finish('keyboard link closed (end of input)')

    def on_signal(self, name):
        return self._finish(name)

    def tick(self, now):
        """The deadman, every publish period: returns the requests ('estop')."""
        if self.finished is not None:
            return []
        age = now - self.last_byte_at
        if age > LINK_ESTOP_SECONDS:
            self.gap_max = max(self.gap_max, age)
            return self._finish(f'no keyboard byte for {age * 1000:.0f} ms')
        if age > LINK_STOP_SECONDS and not self.link_stopped:
            self.link_stopped = True
            self.counts['link stops'] += 1
            self._stop()
            self.events.append(f'no keyboard byte for {age * 1000:.0f} ms: speed 0, centre (still armed; w to drive)')
        return []

    def take_events(self):
        events, self.events = self.events, []
        return events

    # --- helpers ---

    def _stop(self):
        self.level = SPEED_LEVELS.index(0.0)
        self.steer_percent = 0

    def _finish(self, why):
        if self.finished is not None:
            return []
        self._stop()
        self.finished = why
        self.events.append(f'{why}: speed 0, /rover/estop, publishing stops')
        return ['estop']

    def recent_gap_max(self, now):
        """s: the longest gap between bytes in the last GAP_WINDOW_SECONDS, the open one included."""
        while self.gaps and now - self.gaps[0][0] > GAP_WINDOW_SECONDS:
            self.gaps.popleft()
        return max([gap for _at, gap in self.gaps] + [now - self.last_byte_at])

    def status_text(self, now, cps_per_meter):
        """The status line's teleop part."""
        speed = self.speed
        delta = self.steer_percent / 100 * self.max_angle
        steer = f'steer {self.steer_percent:+d}% ({delta:+.3f} rad)'
        if speed == 0.0 and self.steer_percent:
            steer += ' applies when moving'
        link = 'LINK STOP (w to drive)' if self.link_stopped else 'link ok'
        return (f'v {speed:+.3f} m/s ({round(speed * cps_per_meter)} cps) {steer} '
                f'w {angular_z(speed, self.steer_percent, self.wheelbase, self.max_angle):+.3f} rad/s | '
                f'{link}, last byte {(now - self.last_byte_at) * 1000:.0f} ms, max {self.recent_gap_max(now) * 1000:.0f} ms/10 s')
