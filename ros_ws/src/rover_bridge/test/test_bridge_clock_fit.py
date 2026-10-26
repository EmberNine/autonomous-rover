"""rover_bridge.clock_fit against a synthetic board: its clock 2000-4000 ppm fast, frames every
10 ms of board time, each reaching the Pi 1.5 ms + exponential jitter later (2 % of them 20-100 ms
late, like a USB or scheduler burst), received on the next 1 ms USB frame (full speed polls every
1 ms; the reader's read(in_waiting or 1) returns as soon as bytes arrive, so the receive time is the
arrival: in b5_session_vbat_1 the reads came every ~10 ms, one per frame). Pure (CI included)."""
import math
import random

import pytest

from rover_bridge.clock_fit import WRAP, ClockFit

LATENCY_FLOOR = 0.0015
USB_FRAME = 0.001
HOST_START = 1_792_000_000.0


def board_stream(seconds, drift_ppm, seed=1, start_t_ms=500_000, host_start=HOST_START, drift_after=None,
                 ramp_ppm_per_s=0.0):
    """[(t_ms, received_at, made_at)]: made_at is the true Pi time the frame was made.
    drift_after: (board seconds, new drift ppm) to change the drift mid-run (temperature)."""
    rng = random.Random(seed)
    frames = []
    made = host_start
    t_ms = start_t_ms
    usb_phase = rng.random() * USB_FRAME
    for _ in range(int(seconds * 100)):
        drift = drift_ppm
        if drift_after is not None and (t_ms - start_t_ms) / 1000 >= drift_after[0]:
            drift = drift_after[1]
        drift += ramp_ppm_per_s * (t_ms - start_t_ms) / 1000         # warming up (thermal drift)
        made += 0.010 / (1 + drift * 1e-6)              # 10 ms of a fast board clock is less on the Pi's
        t_ms += 10
        latency = LATENCY_FLOOR + rng.expovariate(1 / 0.002)
        if rng.random() < 0.02:
            latency += rng.uniform(0.02, 0.1)
        arrival = made + latency
        received = host_start + usb_phase + math.ceil((arrival - host_start - usb_phase) / USB_FRAME) * USB_FRAME
        frames.append((t_ms & (WRAP - 1), received, made))
    return frames


def fed(frames, fit=None):
    fit = fit or ClockFit()
    for t_ms, received, _ in frames:
        fit.add(t_ms, received)
    return fit


@pytest.mark.parametrize('drift_ppm', (2000, 3000, 4000))
def test_drift_offset_and_residual_from_earliest_arrivals(drift_ppm):
    frames = board_stream(30, drift_ppm)
    fit = fed(frames)
    assert fit.fitted and fit.drift_ppm() == pytest.approx(drift_ppm, abs=100)
    assert fit.residual() < 0.001                               # the minima lie on a line
    assert 0.0 < fit.latency() < 0.02                           # a typical frame: a few ms above the line
    t_ms, received, made = frames[-1]
    assert fit.stamp(t_ms, received) == pytest.approx(made + LATENCY_FLOOR, abs=0.002)


def test_stamps_follow_the_making_not_the_arrival_and_never_come_after_it():
    frames = board_stream(40, 3000, seed=2)
    fit = ClockFit()
    errors = []
    for index, (t_ms, received, made) in enumerate(frames):
        fit.add(t_ms, received)
        value = fit.stamp(t_ms, received)
        assert value <= received
        if index > 1000:                                        # after 10 s of fitting
            errors.append(abs(value - (made + LATENCY_FLOOR)))
    errors.sort()
    assert errors[int(len(errors) * 0.95)] < 0.002              # 95 % within 2 ms of the truth
    late = [received - made for _, received, made in frames]
    assert max(late) > 0.05                                     # the bursts were there, and didn't matter


def test_before_two_seconds_the_earliest_offset_alone():
    frames = board_stream(0.5, 3000)
    t_ms, received, made = frames[-1]
    frames.append((t_ms + 10, received + 0.010 + 0.050, made + 0.010))   # the last one 50 ms late
    fit = fed(frames)
    assert not fit.fitted and fit.drift_ppm() is None and fit.residual() is None
    t_ms, received, _ = frames[-1]
    best = min(received - t_ms / 1000 for t_ms, received, _ in frames)
    # abs=: a relative tolerance on epoch seconds (1.8e9) would allow ~30 minutes
    assert fit.stamp(t_ms, received) == pytest.approx(min(t_ms / 1000 + best, received), abs=1e-6)
    slowest = max(frames, key=lambda frame: frame[1] - frame[0] / 1000)  # the frame that came latest
    assert fit.stamp(*slowest[:2]) == pytest.approx(slowest[0] / 1000 + best, abs=1e-6)  # the earliest offset
    assert fit.stamp(*slowest[:2]) < slowest[1] - 0.001
    assert ClockFit().stamp(1234, 99.0) == 99.0                 # no point at all: the receive time


def test_a_board_reset_starts_the_fit_over():
    fit = fed(board_stream(20, 3000))
    assert fit.fitted and fit.resets == 0
    after = board_stream(5, 3000, seed=3, start_t_ms=0, host_start=HOST_START + 30)  # t_ms back near 0
    fit.add(*after[0][:2])
    assert fit.resets == 1 and not fit.fitted                   # nothing from before the reset survives
    fed(after[1:], fit)
    assert fit.fitted and fit.drift_ppm() == pytest.approx(3000, abs=1000)   # 5 s: a rougher estimate
    t_ms, received, made = after[-1]
    assert fit.stamp(t_ms, received) == pytest.approx(made + LATENCY_FLOOR, abs=0.003)


def test_explicit_reset_counts():
    fit = fed(board_stream(5, 3000))
    fit.reset()
    assert fit.resets == 1 and fit.a is None and fit.span() == 0.0


def test_a_u32_wrap_is_not_a_reset():
    frames = board_stream(30, 3000, start_t_ms=WRAP - 15_000)   # wraps after 15 s
    assert frames[-1][0] < frames[0][0]
    fit = fed(frames)
    assert fit.resets == 0 and fit.wraps == 1
    assert fit.drift_ppm() == pytest.approx(3000, abs=100)
    t_ms, received, made = frames[-1]
    assert fit.stamp(t_ms, received) == pytest.approx(made + LATENCY_FLOOR, abs=0.002)


def test_the_window_follows_a_drift_change():
    frames = board_stream(140, 2000, drift_after=(60.0, 4000))
    fit = ClockFit()
    for index, (t_ms, received, _) in enumerate(frames):
        fit.add(t_ms, received)
        if index == 5900:                                       # 59 s: all at 2000
            assert fit.drift_ppm() == pytest.approx(2000, abs=100)
    assert fit.drift_ppm() == pytest.approx(4000, abs=150)      # 80 s later: the old drift is out of the window
    assert fit.span() <= 21.0                                   # the 20 s window


def test_a_reader_stall_does_not_move_the_fit():
    frames = board_stream(30, 3000, seed=4)
    stalled = [(t_ms, max(received, frames[1500][1] + 2.0) if 1500 <= index < 1700 else received, made)
               for index, (t_ms, received, made) in enumerate(frames)]   # 2 s of frames delivered late at once
    fit = fed(stalled)
    assert fit.drift_ppm() == pytest.approx(3000, abs=150) and fit.residual() < 0.002
    assert fit.rejected >= 1                                    # the stalled seconds are out of the line


def test_one_late_frame_alone_in_a_new_bin_does_not_move_the_fit():
    frames = board_stream(20, 3000, seed=3)
    t_ms, received, _ = frames[-1]
    next_t_ms = (t_ms // 1000 + 1) * 1000                       # the first frame of the next second
    on_time = received + (next_t_ms - t_ms) / 1000
    fits = []
    for delay in (0.0, 0.08):                                   # on time, or 80 ms late (a burst)
        fit = fed(frames)
        fit.add(next_t_ms, on_time + delay)
        fits.append(fit)
    assert fits[1].drift_ppm() == pytest.approx(fits[0].drift_ppm(), abs=0.01)
    assert fits[1].a == pytest.approx(fits[0].a, abs=1e-6)     # alone in its bin: it isn't in the line


def test_state_stamps_never_go_backwards_even_when_the_line_moves():
    frames = board_stream(10, 3000, seed=5)
    fit = ClockFit()
    stamps = []
    for index, (t_ms, received, _) in enumerate(frames):
        fit.add(t_ms, received)
        if index == 600:
            fit.a -= 0.020                                    # the line jumps 20 ms back (a refit)
        stamps.append(fit.state_stamp(t_ms, received))
    assert all(later > earlier for earlier, later in zip(stamps, stamps[1:]))
    assert fit.clamps >= 1
    assert fit.state_stamp(*frames[-1][:2]) > stamps[-1]     # the same frame again: still forward
    after = board_stream(3, 3000, seed=6, start_t_ms=0, host_start=frames[-1][1] - 0.5)  # reset, mapped earlier
    for t_ms, received, _ in after[:50]:
        fit.add(t_ms, received)
        assert fit.state_stamp(t_ms, received) > stamps[-1]  # not across a board reset either
    assert fit.resets == 1


def with_steady(frames, step_at=None, step=0.0, slew_ppm=0.0):
    """(t_ms, received, made, steady): steady is the read's monotonic time. From frame step_at on,
    the wall clock is step seconds off (an NTP / WSL correction); slew_ppm: the wall clock runs
    that much fast against monotonic (NTP slewing), which is no step."""
    out = []
    for index, (t_ms, received, made) in enumerate(frames):
        steady = 5000.0 + (received - HOST_START)
        wall = received + (received - HOST_START) * slew_ppm * 1e-6
        shift = step if step_at is not None and index >= step_at else 0.0
        out.append((t_ms, wall + shift, made + shift, steady))
    return out


@pytest.mark.parametrize('step', (-0.95, +0.67))
def test_a_host_clock_step_restarts_the_fit_and_stamps_follow_the_clock(step):
    frames = with_steady(board_stream(20, 3000, seed=7), step_at=1000, step=step)
    fit = ClockFit()
    for t_ms, received, made, steady in frames:
        fit.add(t_ms, received, steady)
        assert fit.state_stamp(t_ms, received) <= received + 1e-9   # never after its arrival, even right after the step
    assert fit.host_steps == 1 and fit.resets == 0
    assert fit.drift_ppm() == pytest.approx(3000, abs=300)          # refitted on the 10 s since
    t_ms, received, made, _ = frames[-1]
    assert fit.stamp(t_ms, received) == pytest.approx(made + LATENCY_FLOOR, abs=0.002)   # on the new clock


def test_ntp_slewing_is_not_a_step():
    frames = with_steady(board_stream(40, 3000, seed=8), slew_ppm=500)
    fit = ClockFit()
    for t_ms, received, _, steady in frames:
        fit.add(t_ms, received, steady)
    assert fit.host_steps == 0 and fit.drift_ppm() == pytest.approx(2500, abs=100)   # the slew shows as drift


def test_a_drift_ramping_like_the_bench_warm_up():
    """Group 2 bench under dd: +2800 -> +4469 ppm in ~8 min, ~3.3 ppm/s. A line over W seconds is off
    by ~r*W^2/12 at the newest point: ~0.1 ms for 20 s, ~1 ms for 60 s (measured here: 0.49 vs 1.34 ms
    median, of which ~0.4 ms is the same for any window: the earliest arrivals sit above the floor)."""
    def median_error(window):
        fit = ClockFit(window_seconds=window)
        errors = []
        for index, (t_ms, received, made) in enumerate(board_stream(240, 2800, seed=9, ramp_ppm_per_s=3.3)):
            fit.add(t_ms, received)
            if index > 100 * (window + 5):
                errors.append(fit.stamp(t_ms, received) - (made + LATENCY_FLOOR))
        errors.sort()
        return errors[len(errors) // 2]
    twenty, sixty = median_error(20.0), median_error(60.0)
    assert abs(twenty) < 0.0007
    assert sixty - twenty > 0.0006                                  # the curvature a 60 s line can't follow
