#!/usr/bin/env python3
"""Log steps of this machine's wall clock and, optionally, the gaps in /cmd_vel.

Why: WSL's clock steps back ~0.9 s about every 32 s (seen 2026-10-17, the
bridge's clock fit), and `ros2 topic pub -r` times its publishing on the ROS
clock, which is the wall clock (ros2topic/verb/pub.py: node.create_timer
without clock=; rclpy Node._clock is a ROSClock). After a backward step of S
its next message waits S longer, and the bridge's heartbeat sees a gap of
S + up to one period: over 300 ms it stops the wheel, over 1 s it ESTOPs.

Clock steps (always): a thread reads monotonic, wall, monotonic every 10 ms;
a change of more than STEP_SECONDS (the bridge's clock_fit.HOST_STEP_SECONDS)
in wall - monotonic is a step, printed with its size, direction and the time
since the previous one. A read where the two monotonic readings are more than
READ_SLACK_SECONDS apart (the thread was preempted mid-read) is dropped.

/cmd_vel (needs ROS 2 sourced):
  --listen     subscribe (best effort, depth 1, like the bridge) and print each
               gap longer than GAP_SECONDS, with the clock step inside it, if any
  --publish    publish Twist linear.x = --speed at --rate on a STEADY-clock
               timer (a step of the wall clock can't delay it); also listens,
               to its own messages (a node receives its own publications):
               then the gaps are LOCAL delivery on this machine only (the timer,
               rclpy, DDS here), not the network to the Pi. Run --listen on the
               Pi as well to see what arrives there.
               --pause-every N --pause P: publish nothing for P s every N s
               (a heartbeat pause the bridge must log).

The summary at the end: steps by direction, their sizes and intervals, the
rate they imply (sum of back steps / time: how fast the clock runs between
corrections, if that is what they are), monotonic against the corrected wall
clock (between two steps: the wall clock just after each is the corrected
time, so how fast this machine's monotonic clock runs; WSL 2026-10-19: +3.1 %,
so this probe's own step and gap sizes read that much long), how many
/cmd_vel messages arrived (and at what rate) and how many gaps had a step inside.
--csv PATH also writes every step and gap (kind, monotonic, wall, value s).

Usage (WSL):
  python3 tools/clock_step_probe.py --seconds 180                 # step 0: clock only, no ROS
  python3 tools/clock_step_probe.py --seconds 200 --listen        # next to ros2 topic pub
  python3 tools/clock_step_probe.py --seconds 200 --publish --speed 0.1 --pause-every 10 --pause 0.5
"""
import argparse
import csv
import statistics
import sys
import threading
import time

STEP_SECONDS = 0.05              # clock_fit.HOST_STEP_SECONDS
SAMPLE_PERIOD_SECONDS = 0.01
READ_SLACK_SECONDS = 0.005
GAP_SECONDS = 0.15               # 20 Hz is 50 ms between messages
COMMAND_TOPIC = '/cmd_vel'


class StepDetector:
    """A step: wall - monotonic changed by more than threshold since the previous sample.
    The size is signed: negative = the wall clock went back."""

    def __init__(self, threshold=STEP_SECONDS):
        self.threshold = threshold
        self.offset = None

    def add(self, wall, steady):
        offset = wall - steady
        previous, self.offset = self.offset, offset
        if previous is not None and abs(offset - previous) > self.threshold:
            return offset - previous
        return None


class GapDetector:
    """The time since the previous arrival, when longer than threshold."""

    def __init__(self, threshold=GAP_SECONDS):
        self.threshold = threshold
        self.last = None

    def add(self, steady):
        previous, self.last = self.last, steady
        if previous is not None and steady - previous > self.threshold:
            return steady - previous
        return None


def read_clocks(wall_clock=time.time, steady_clock=time.monotonic, slack=READ_SLACK_SECONDS):
    """(wall, steady), or None if the read was interrupted for longer than slack."""
    before = steady_clock()
    wall = wall_clock()
    after = steady_clock()
    if after - before > slack:
        return None
    return wall, (before + after) / 2


def step_inside(gap_start, gap_end, steps):
    """The first step (at, size) with gap_start <= at <= gap_end, or None."""
    return next(((at, size) for at, size in steps if gap_start <= at <= gap_end), None)


def publishing_now(elapsed, every, pause):
    """False during the P s pause that starts at every multiple of N s (not at 0)."""
    if not every or not pause or elapsed < every:
        return True
    return elapsed % every >= pause


def monotonic_rate(steps, walls):
    """How fast monotonic runs against the corrected wall clock, as a fraction (+0.031: 3.1 %
    fast), from the first to the last step: the wall clock just after each step is the corrected
    time. None with fewer than two steps."""
    if len(steps) < 2 or len(walls) != len(steps):
        return None
    return (steps[-1][0] - steps[0][0]) / (walls[-1] - walls[0]) - 1.0


def summary(steps, gaps, span, walls=(), received=None):
    """The closing lines. steps: [(at, size)], gaps: [(start, length, step or None)], span: s;
    walls: the wall time just after each step; received: /cmd_vel messages that arrived."""
    lines = []
    back = [-size for _at, size in steps if size < 0]
    forward = [size for _at, size in steps if size > 0]
    lines.append(f'{len(steps)} steps in {span:.0f} s: {len(back)} back, {len(forward)} forward')
    for name, sizes in (('back', back), ('forward', forward)):
        if sizes:
            lines.append(f'  {name} sizes ms: min {min(sizes) * 1000:.0f}, median '
                         f'{statistics.median(sizes) * 1000:.0f}, max {max(sizes) * 1000:.0f}')
    times = [at for at, _size in steps]
    intervals = [later - earlier for earlier, later in zip(times, times[1:])]
    if intervals:
        lines.append(f'  intervals s: min {min(intervals):.1f}, median {statistics.median(intervals):.1f}, '
                     f'max {max(intervals):.1f}')
    if back and span > 0:
        lines.append(f'  implied rate: {sum(back) / span * 100:+.2f} % (back steps per time: the clock runs '
                     f'that much fast between corrections, if that is what they are)')
    rate = monotonic_rate(steps, walls)
    if rate is not None:
        lines.append(f'  monotonic vs the corrected wall clock: {rate * 100:+.2f} % (first to last step; '
                     f"this probe's own sizes read that much long)")
    if gaps is not None:
        received = received or 0
        hertz = f' ({received / span:.1f} Hz)' if span > 0 else ''
        lines.append(f'{received} /cmd_vel messages received here{hertz}: local delivery on this machine, '
                     f'not the network')
        with_step = sum(1 for _start, _length, step in gaps if step is not None)
        longest = max((length for _start, length, _step in gaps), default=0.0)
        lines.append(f'{len(gaps)} /cmd_vel gaps over {GAP_SECONDS * 1000:.0f} ms ({with_step} with a clock step '
                     f'inside, {len(gaps) - with_step} without); longest {longest * 1000:.0f} ms')
    return lines


class Probe:

    def __init__(self, csv_path=None, print_function=print):
        self.start = time.monotonic()
        self.lock = threading.Lock()
        self.steps = []                      # (monotonic, size)
        self.walls = []                      # the wall time just after each step
        self.received = 0                    # /cmd_vel messages that arrived
        self.gaps = []                       # (start, length, step or None)
        self.detector = StepDetector()
        self.gap_detector = GapDetector()
        self.print = print_function
        self.csv_file = open(csv_path, 'w', newline='', encoding='ascii') if csv_path else None
        self.csv = csv.writer(self.csv_file) if self.csv_file else None
        if self.csv:
            self.csv.writerow(['kind', 'monotonic', 'wall', 'value_s'])

    def _row(self, kind, steady, wall, value):
        if self.csv:
            self.csv.writerow([kind, f'{steady:.6f}', f'{wall:.6f}', f'{value:.6f}'])

    def sample_clocks(self, stop):
        while not stop.is_set():
            read = read_clocks()
            if read is not None:
                wall, steady = read
                size = self.detector.add(wall, steady)
                if size is not None:
                    with self.lock:
                        since = steady - self.steps[-1][0] if self.steps else None
                        self.steps.append((steady, size))
                        self.walls.append(wall)
                        self._row('step', steady, wall, size)
                    self.print(f'{steady - self.start:8.2f} s  step {"BACK" if size < 0 else "FWD "} '
                               f'{abs(size) * 1000:7.1f} ms' +
                               ('' if since is None else f'  ({since:.2f} s since the previous)'))
            stop.wait(SAMPLE_PERIOD_SECONDS)

    def on_command(self):
        steady = time.monotonic()
        with self.lock:
            self.received += 1
        length = self.gap_detector.add(steady)
        if length is None:
            return
        with self.lock:
            step = step_inside(steady - length, steady, self.steps)
            self.gaps.append((steady - length, length, step))
            self._row('gap', steady, time.time(), length)
        inside = 'no clock step inside' if step is None else \
            f'step {step[1] * 1000:+.0f} ms at {step[0] - self.start:.2f} s inside'
        self.print(f'{steady - self.start:8.2f} s  {COMMAND_TOPIC} gap {length * 1000:6.0f} ms ({inside})')

    def finish(self, with_gaps):
        span = time.monotonic() - self.start
        # A gap still open at the end isn't reported (it may be the operator stopping).
        for line in summary(self.steps, self.gaps if with_gaps else None, span, self.walls, self.received):
            self.print(line)
        if self.csv_file:
            self.csv_file.close()


def run_ros(probe, arguments, stop):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node('clock_step_probe')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Twist, COMMAND_TOPIC, lambda message: probe.on_command(), qos)
    if arguments.publish:
        publisher = node.create_publisher(Twist, COMMAND_TOPIC, qos)
        message = Twist()
        message.linear.x = arguments.speed

        def publish():
            if publishing_now(time.monotonic() - probe.start, arguments.pause_every, arguments.pause):
                publisher.publish(message)
        # STEADY: a step of the wall clock doesn't move this timer (the point of the probe).
        node.create_timer(1.0 / arguments.rate, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
    end = probe.start + arguments.seconds
    try:
        while not stop.is_set() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=180.0)
    parser.add_argument('--listen', action='store_true', help=f'measure {COMMAND_TOPIC} gaps (needs ROS 2)')
    parser.add_argument('--publish', action='store_true', help=f'publish {COMMAND_TOPIC} on a steady timer')
    parser.add_argument('--speed', type=float, default=0.1, help='linear.x, m/s (--publish)')
    parser.add_argument('--rate', type=float, default=20.0, help='Hz (--publish)')
    parser.add_argument('--pause-every', type=float, default=0.0, help='s between pauses (--publish)')
    parser.add_argument('--pause', type=float, default=0.0, help='pause length, s (--publish)')
    parser.add_argument('--csv', help='also write every step and gap to this CSV')
    arguments = parser.parse_args(argv)
    probe = Probe(arguments.csv)
    probe.print(f'sampling wall - monotonic every {SAMPLE_PERIOD_SECONDS * 1000:.0f} ms for {arguments.seconds:.0f} s; '
                f'a step is a change over {STEP_SECONDS * 1000:.0f} ms')
    stop = threading.Event()
    sampler = threading.Thread(target=probe.sample_clocks, args=(stop,), daemon=True)
    sampler.start()
    ros = arguments.listen or arguments.publish
    try:
        if ros:
            run_ros(probe, arguments, stop)
        else:
            stop.wait(arguments.seconds)
    except KeyboardInterrupt:
        pass
    stop.set()
    sampler.join()
    probe.finish(with_gaps=ros)
    return 0


if __name__ == '__main__':
    sys.exit(main())
