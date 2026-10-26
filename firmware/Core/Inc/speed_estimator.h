/**
  ******************************************************************************
  * @file    speed_estimator.h
  * @brief   Wheel speed from the encoder count: a 20 ms sliding window,
  *          updated every ms, with a validity flag the speed controller must
  *          check. No hardware access: the caller passes the count and the
  *          read results right after the encoder read.
  *
  * Estimate: count[now] - count[now - 20 ms], in speed units:
  *   1 unit = 1 count per 20 ms = 50 counts/s (the same unit as the "v"
  *   setpoint, so there is no conversion and no rounding in firmware).
  * Resolution is 1 unit; the delay about 10 ms (half the window) plus the
  * AS5600's own output filter. A difference of two counts, so sensor jitter
  * doesn't add up over the window: only the two ends matter. The count is
  * the unwrapper's 32-bit odometer, which wraps; differences are taken in
  * unsigned arithmetic, so a wrap inside the window is harmless.
  *
  * Any count source works: today the raw angle's unwrapper. A later
  * raw-angle correction (the 4-per-revolution sensor error) feeds the
  * corrected angle into its own unwrapper and passes that count here; this
  * module doesn't change.
  *
  * Valid only when the whole window is trustworthy: the SPEED_WINDOW_MS + 1
  * samples it spans were all good updates, i.e. none of these happened in
  * any of them (one bad update spoils the next SPEED_WINDOW_MS + 1 updates,
  * then it is valid again by itself):
  *   - the angle read failed (the count is stale);
  *   - the count moved more than SPEED_MAX_DELTA_COUNTS since the previous
  *     update (20 counts in 1 ms = 20,000 counts/s, four times what the 30 %
  *     cap reaches on the lift: a glitch, not motion);
  *   - MD = 0 at the last magnet health read (no magnet: the angle is
  *     garbage);
  *   - the time since the previous update wasn't exactly 1 ms (a resync, an
  *     overrun or a catch-up iteration: the window would span the wrong
  *     time).
  * ML and AGC are not validity inputs: they say the field is weak-ish, not
  * that the angle is wrong.
  * Before the first SPEED_WINDOW_MS + 1 updates it is invalid (STARTING).
  *
  * Reason: the most recent bad update's cause (if one update has several:
  * TIME_STEP, then READ_FAILED, NO_MAGNET, JUMP). invalid_ms: how long it has
  * been invalid without a break (0 at the first invalid update, and while
  * valid); the speed controller holds for a while, then gives up.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_speed_estimator.c. No globals.
  ******************************************************************************
  */
#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#define SPEED_WINDOW_MS          20u   /* 1 unit = 1 count per window = 50 counts/s */
#define SPEED_MAX_DELTA_COUNTS   20    /* per 1 ms update; more is a glitch */
#define SPEED_RING_SIZE          (SPEED_WINDOW_MS + 1u)

typedef enum
{
  SPEED_ESTIMATE_VALID = 0,
  SPEED_ESTIMATE_STARTING,     /* fewer than SPEED_RING_SIZE updates since Init */
  SPEED_ESTIMATE_READ_FAILED,  /* an angle read failed in the window */
  SPEED_ESTIMATE_JUMP,         /* a per-ms delta above SPEED_MAX_DELTA_COUNTS in the window */
  SPEED_ESTIMATE_NO_MAGNET,    /* MD = 0 at the last health read, in the window */
  SPEED_ESTIMATE_TIME_STEP     /* an update not exactly 1 ms after the previous one, in the window */
} SpeedEstimateReason;

/* What the caller hands over every ms, right after the encoder read. */
typedef struct
{
  uint32_t now_ms;           /* the iteration's real time (HAL tick), not the scheduled time */
  int32_t  count;            /* unwrapped count (stale when read_ok is false) */
  bool     read_ok;          /* this ms's angle read succeeded */
  bool     magnet_detected;  /* MD at the last health read */
} SpeedEstimatorInput;

typedef struct
{
  /* Results, after each update */
  bool                valid;
  int32_t             speed_units;      /* window difference; meaningful only when valid (shown in
                                           telemetry anyway once the ring is full, 0 before) */
  SpeedEstimateReason reason;           /* VALID, or why not */
  uint32_t            invalid_ms;       /* ms invalid without a break; 0 while valid */
  uint32_t            bad_update_count; /* bad updates since Init (diagnostics) */

  /* State */
  int32_t  counts[SPEED_RING_SIZE];     /* the last SPEED_RING_SIZE counts, oldest at index next */
  uint32_t next;                        /* where the next count goes */
  uint32_t filled;                      /* counts stored so far, up to SPEED_RING_SIZE */
  uint32_t good_streak;                 /* consecutive good updates, this one included (capped) */
  SpeedEstimateReason last_bad_reason;  /* cause of the most recent bad update */
  uint32_t previous_ms;
  int32_t  previous_count;
  uint32_t invalid_since_ms;
  bool     was_valid;
} SpeedEstimator;

void SpeedEstimator_Init(SpeedEstimator *estimator);

/* Every ms, right after the encoder read. */
void SpeedEstimator_Update(SpeedEstimator *estimator, const SpeedEstimatorInput *input);

#endif /* SPEED_ESTIMATOR_H */
