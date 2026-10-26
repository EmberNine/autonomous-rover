"""Tests for clock_step_probe.py. Run from the repo root: python3 -m pytest tools/

Synthetic clocks: a wall clock ~1.7e9 s with a monotonic one, sampled every
10 ms, with and without steps. No ROS, no real time.
"""
import clock_step_probe as probe

WALL0, STEADY0 = 1_760_000_000.0, 5_000.0


def run(detector, offsets, period=0.01, rate=1.0):
    """Feed samples where wall = steady * rate + offset; the steps found, as (index, size)."""
    found = []
    for index, offset in enumerate(offsets):
        steady = STEADY0 + index * period
        size = detector.add(WALL0 + (steady - STEADY0) * rate + offset, steady)
        if size is not None:
            found.append((index, size))
    return found


def test_a_backward_step_of_0_9_s_is_found_once_with_its_size():
    offsets = [0.0] * 50 + [-0.9] * 50
    found = run(probe.StepDetector(), offsets)
    assert len(found) == 1
    index, size = found[0]
    assert index == 50 and abs(size + 0.9) < 1e-6


def test_a_forward_step_is_positive():
    found = run(probe.StepDetector(), [0.0] * 10 + [0.2] * 10)
    assert len(found) == 1 and found[0][1] > 0.19


def test_jitter_below_the_threshold_is_not_a_step():
    offsets = [0.04 * (index % 2) for index in range(200)]        # +-40 ms between samples
    assert run(probe.StepDetector(), offsets) == []


def test_a_clock_running_3_percent_fast_is_not_a_step_by_itself():
    # 0.3 ms per 10 ms sample: the drift between corrections; only the correction is a step.
    assert run(probe.StepDetector(), [0.0] * 1000, rate=1.03) == []
    found = run(probe.StepDetector(), [0.0] * 500 + [-0.9] * 500, rate=1.03)
    assert [index for index, _size in found] == [500]


def test_a_slow_loop_is_not_a_step():
    # Both clocks advance 0.5 s between two samples (the thread was descheduled): no step.
    detector = probe.StepDetector()
    assert detector.add(WALL0, STEADY0) is None
    assert detector.add(WALL0 + 0.5, STEADY0 + 0.5) is None


def test_an_interrupted_read_is_dropped():
    steady = iter([10.0, 10.020])                                  # 20 ms between the two reads
    assert probe.read_clocks(lambda: WALL0, lambda: next(steady)) is None
    steady = iter([10.0, 10.001])
    wall, middle = probe.read_clocks(lambda: WALL0, lambda: next(steady))
    assert wall == WALL0 and abs(middle - 10.0005) < 1e-9


def test_gaps_longer_than_the_threshold():
    detector = probe.GapDetector(threshold=0.15)
    arrivals = [0.0, 0.05, 0.10, 1.05, 1.10, 1.24, 1.40]
    gaps = [detector.add(at) for at in arrivals]
    assert gaps[0] is None and gaps[1] is None and gaps[2] is None
    assert abs(gaps[3] - 0.95) < 1e-9
    assert gaps[4] is None and gaps[5] is None                    # 140 ms: not a gap
    assert abs(gaps[6] - 0.16) < 1e-9


def test_the_step_inside_a_gap():
    steps = [(10.0, -0.9), (42.0, -0.88)]
    assert probe.step_inside(9.98, 10.93, steps) == (10.0, -0.9)
    assert probe.step_inside(42.0, 42.9, steps) == (42.0, -0.88)  # at the gap's start counts
    assert probe.step_inside(20.0, 20.5, steps) is None


def test_the_pause_schedule():
    assert probe.publishing_now(5.0, 10.0, 0.5)
    assert probe.publishing_now(9.99, 10.0, 0.5)
    assert not probe.publishing_now(10.0, 10.0, 0.5)
    assert not probe.publishing_now(10.49, 10.0, 0.5)
    assert probe.publishing_now(10.5, 10.0, 0.5)
    assert not probe.publishing_now(20.2, 10.0, 0.5)
    assert probe.publishing_now(0.1, 10.0, 0.5)                  # no pause at the start
    assert probe.publishing_now(10.2, 0.0, 0.5) and probe.publishing_now(10.2, 10.0, 0.0)


def test_the_summary():
    steps = [(32.0, -0.9), (64.0, -0.85), (96.5, -1.0), (100.0, 0.2)]
    gaps = [(31.98, 0.95, (32.0, -0.9)), (50.0, 0.2, None)]
    lines = probe.summary(steps, gaps, 128.0)
    text = '\n'.join(lines)
    assert lines[0] == '4 steps in 128 s: 3 back, 1 forward'
    assert 'back sizes ms: min 850, median 900, max 1000' in text
    assert 'forward sizes ms: min 200, median 200, max 200' in text
    assert 'intervals s: min 3.5, median 32.0, max 32.5' in text
    assert 'implied rate: +2.15 %' in text                       # 2.75 s back in 128 s
    assert '2 /cmd_vel gaps over 150 ms (1 with a clock step inside, 1 without); longest 950 ms' in text
    assert 'gaps' not in '\n'.join(probe.summary(steps, None, 128.0))


def test_monotonic_against_the_corrected_wall_clock():
    # bench_s3 2026-10-19: monotonic 32.115 s between two steps, the corrected wall 31.146 s.
    steps = [(8757.216862, -1.12), (8789.332163, -0.97), (8821.741045, -1.10)]
    walls = [1792443740.214592, 1792443771.360886, 1792443802.668587]
    rate = probe.monotonic_rate(steps, walls)
    assert abs(rate - (64.524183 / 62.453995 - 1)) < 1e-9 and 0.032 < rate < 0.034
    assert probe.monotonic_rate(steps[:1], walls[:1]) is None
    text = '\n'.join(probe.summary(steps, None, 70.0, walls))
    assert 'monotonic vs the corrected wall clock: +3.31 %' in text
    assert 'monotonic vs' not in '\n'.join(probe.summary(steps[:1], None, 70.0, walls[:1]))


def test_the_received_count_is_local_delivery():
    lines = []
    counter = probe.Probe(print_function=lines.append)
    for _ in range(3):
        counter.on_command()
    counter.finish(with_gaps=True)
    assert any(line.startswith('3 /cmd_vel messages received here (') and
               line.endswith('local delivery on this machine, not the network') for line in lines), lines
    assert '0 /cmd_vel messages received here: local delivery' in '\n'.join(probe.summary([], [], 0.0))
