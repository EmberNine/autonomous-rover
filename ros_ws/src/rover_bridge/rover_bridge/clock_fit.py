"""Board time (STATE t_ms) -> Pi time, fitted continuously. Pure: no ROS, no I/O.

The Nucleo's clock runs a few thousand ppm fast (+0.2-0.4 %, drifting with
temperature), and every frame reaches the Pi some time after it was made: USB
polling, the ST-LINK bridge, the reader thread. Those delays only ever add, so
the earliest arrivals lie closest to the true mapping. The fit:

  y = receive_time - board_seconds      (one point per STATE)
  per BIN_SECONDS of board time, keep the point with the smallest y (the
  earliest arrival in that second), then a least-squares line y = a + b * board
  through those minima over the last WINDOW_SECONDS of board time. Only
  complete bins count (not the one still filling, and only with at least
  MIN_BIN_POINTS frames): one late frame alone in a bin isn't an "earliest".
  Bins whose minimum lies more than REJECT_ABOVE above the line are dropped
  and the line fitted again (up to REJECT_ROUNDS times): in a reader stall
  every frame of those seconds arrives late, so even their minimum is.
  Delays only add, so only bins above the line are rejected.

stamp(t_ms) = board_seconds + a + b * board_seconds: when the frame was made,
on the Pi's clock. Never later than the frame's own receive time (a frame
can't be stamped after it arrived). Until two complete bins exist (about
3 s), the offset is the smallest y so far and the slope 0.

Reported: drift_ppm = how fast the board clock runs (positive = fast; -b * 1e6
for small b), offset at the newest point, residual = RMS distance of the bin
minima from the line (how well the line fits), latency = median of y above
the line over the newest bin (the typical delay of one frame).

t_ms going backwards (beyond a u32 wrap) means the board restarted: the fit
starts over (reset()). A u32 wrap (after 49.7 days) is unwrapped.

The host clock can step too (NTP, or WSL's clock being corrected: on
2026-10-17 time.time() went back 0.95 s in the middle of a test). With the
read's monotonic time (add(..., steady_at)), a change of wall minus
monotonic by more than HOST_STEP_SECONDS is a step: the fit starts over and
the stamps follow the system clock (the one case they may go back, as ROS
time itself does). NTP's slewing (<= 500 ppm) stays far below that.
"""
import statistics

BIN_SECONDS = 1.0
WINDOW_SECONDS = 20.0             # s; at a drift moving 3.3 ppm/s (warming): ~0.1 ms at the newest point (60 s: ~1 ms)
MIN_BIN_POINTS = 20               # of the 100 STATE per second
REJECT_ABOVE = 0.002              # s
REJECT_ROUNDS = 3
HOST_STEP_SECONDS = 0.05
WRAP = 1 << 32
WRAP_GUARD_MS = 600_000           # a backwards step from the top of the u32 range to near 0 is a wrap


class ClockFit:

    def __init__(self, bin_seconds=BIN_SECONDS, window_seconds=WINDOW_SECONDS):
        self.bin_seconds = bin_seconds
        self.window_seconds = window_seconds
        self.last_stamp = None           # the last STATE stamp (state_stamp); kept across board resets
        self.clamps = 0
        self.wall_minus_steady = None    # time.time() - time.monotonic() at the last read
        self.host_steps = 0
        self.resets = 0
        self.reset(board=False)

    def reset(self, board=True):
        """Forget the fit (a board reset: its clock started over; board=False: the host's
        clock stepped, counted in host_steps instead of resets)."""
        self.bins = {}                  # bin index -> [board_s, y, count]: its earliest arrival, frames in it
        self.used = []                  # (board_s, y) of the bins in the current line
        self.rejected = 0               # complete bins left out of it (above the line)
        self.last_t_ms = None
        self.wraps = 0
        self.min_y = None
        self.a, self.b = None, 0.0
        self.newest_board = None
        self.newest_ys = []             # (board_s, y) of the newest bin, for the latency
        self.newest_bin = None
        if board:
            self.resets += 1

    def board_seconds(self, t_ms):
        """Unwrapped board time in seconds. Detects a restart (t_ms backwards) and resets."""
        if self.last_t_ms is not None and t_ms < self.last_t_ms:
            if self.last_t_ms - t_ms > WRAP - WRAP_GUARD_MS:
                self.wraps += 1
            else:
                self.reset()
        self.last_t_ms = t_ms
        return (t_ms + self.wraps * WRAP) / 1000.0

    def add(self, t_ms, received_at, steady_at=None):
        """One STATE: board t_ms and the host time its bytes were read (and, if known, the
        monotonic time of that read, to tell a host clock step)."""
        if steady_at is not None:
            difference = received_at - steady_at
            if self.wall_minus_steady is not None and abs(difference - self.wall_minus_steady) > HOST_STEP_SECONDS:
                self.host_steps += 1
                self.reset(board=False)
                self.last_stamp = None   # follow the system clock, even backwards
            self.wall_minus_steady = difference
        board = self.board_seconds(t_ms)
        y = received_at - board
        self.min_y = y if self.min_y is None else min(self.min_y, y)
        index = int(board // self.bin_seconds)
        best = self.bins.get(index)
        if best is None:
            self.bins[index] = [board, y, 1]
        else:
            best[2] += 1
            if y < best[1]:
                best[0], best[1] = board, y
        if index != self.newest_bin:
            self.newest_bin = index
            self.newest_ys = []
        self.newest_ys.append((board, y))
        self.newest_board = board
        oldest_kept = int((board - self.window_seconds) // self.bin_seconds)
        for old in [key for key in self.bins if key < oldest_kept]:
            del self.bins[old]
        self._refit()

    def _refit(self):
        complete = [(x, y) for index, (x, y, count) in self.bins.items()
                    if index != self.newest_bin and count >= MIN_BIN_POINTS]
        line = _line(complete)
        used = complete
        for _ in range(REJECT_ROUNDS):
            if line is None:
                break
            a, b = line
            kept = [(x, y) for x, y in complete if y - (a + b * x) <= REJECT_ABOVE]
            if len(kept) == len(used) or len(kept) < 2:
                break
            used, line = kept, _line(kept)
        if line is None:
            self.a, self.b, self.used, self.rejected = self.min_y, 0.0, [], 0
            return
        (self.a, self.b), self.used, self.rejected = line, used, len(complete) - len(used)

    @property
    def fitted(self):
        return len(self.used) >= 2

    def stamp(self, t_ms, received_at):
        """Pi time at which the board made the frame with this t_ms (seconds since the epoch).
        Call add() for the frame first if it is a STATE. Without any point yet: received_at."""
        if self.a is None:
            return received_at
        board = (t_ms + self.wraps * WRAP) / 1000.0
        return min(board + self.a + self.b * board, received_at)

    def state_stamp(self, t_ms, received_at):
        """stamp() for the STATE stream, never going backwards: a refit can move the line (a new
        bin, a rejected one, the switch from the start-up offset), and a later frame must not get
        an earlier stamp than the one before it. Then it gets that one + 1 us, and clamps counts."""
        value = self.stamp(t_ms, received_at)
        if self.last_stamp is not None and value <= self.last_stamp:
            value = self.last_stamp + 1e-6
            self.clamps += 1
        self.last_stamp = value
        return value

    # --- for /diagnostics ---

    def drift_ppm(self):
        """How fast the board clock runs relative to the Pi's: +3000 = 0.3 % fast. None until fitted."""
        if not self.fitted:
            return None
        return -self.b / (1.0 + self.b) * 1e6

    def offset(self):
        """y of the line at the newest point (s): Pi time minus board time, now."""
        if self.a is None or self.newest_board is None:
            return None
        return self.a + self.b * self.newest_board

    def residual(self):
        """RMS distance of the bin minima in the line from it (s). None until fitted."""
        if not self.fitted:
            return None
        return statistics.fmean((y - (self.a + self.b * x)) ** 2 for x, y in self.used) ** 0.5

    def latency(self):
        """Median delay above the line in the newest bin (s): a frame's typical way to the Pi."""
        if self.a is None or not self.newest_ys:
            return None
        return statistics.median(y - (self.a + self.b * x) for x, y in self.newest_ys)

    def span(self):
        """Board seconds covered by the bins in the line."""
        if not self.used:
            return 0.0
        xs = [x for x, _ in self.used]
        return max(xs) - min(xs)


def _line(points):
    """(a, b) of the least-squares line y = a + b * x, or None for fewer than 2 distinct x."""
    if len(points) < 2:
        return None
    n = len(points)
    mean_x = sum(x for x, _ in points) / n
    mean_y = sum(y for _, y in points) / n
    sxx = sum((x - mean_x) ** 2 for x, _ in points)
    if sxx == 0:
        return None
    b = sum((x - mean_x) * (y - mean_y) for x, y in points) / sxx
    return mean_y - b * mean_x, b
