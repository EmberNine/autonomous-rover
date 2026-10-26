/**
  ******************************************************************************
  * @file    speed_estimator.c
  * @brief   20 ms sliding-window wheel speed with validity (see the header).
  ******************************************************************************
  */
#include "speed_estimator.h"

/* Difference of two odometer counts, correct across the 32-bit wrap. */
static int32_t SpeedEstimator_CountDifference(int32_t newer, int32_t older)
{
  return (int32_t)((uint32_t)newer - (uint32_t)older);
}

void SpeedEstimator_Init(SpeedEstimator *estimator)
{
  *estimator = (SpeedEstimator){0};
  estimator->valid = false;
  estimator->reason = SPEED_ESTIMATE_STARTING;
  estimator->last_bad_reason = SPEED_ESTIMATE_VALID;
}

/* Why this update is bad, or SPEED_ESTIMATE_VALID if it is good. */
static SpeedEstimateReason SpeedEstimator_Check(const SpeedEstimator *estimator, const SpeedEstimatorInput *input)
{
  bool first = (estimator->filled == 0u);
  if (!first && input->now_ms - estimator->previous_ms != 1u)
  {
    return SPEED_ESTIMATE_TIME_STEP;
  }
  if (!input->read_ok)
  {
    return SPEED_ESTIMATE_READ_FAILED;
  }
  if (!input->magnet_detected)
  {
    return SPEED_ESTIMATE_NO_MAGNET;
  }
  if (!first)
  {
    int32_t delta = SpeedEstimator_CountDifference(input->count, estimator->previous_count);
    if (delta > SPEED_MAX_DELTA_COUNTS || delta < -SPEED_MAX_DELTA_COUNTS)
    {
      return SPEED_ESTIMATE_JUMP;
    }
  }
  return SPEED_ESTIMATE_VALID;
}

void SpeedEstimator_Update(SpeedEstimator *estimator, const SpeedEstimatorInput *input)
{
  bool first = (estimator->filled == 0u);
  SpeedEstimateReason bad = SpeedEstimator_Check(estimator, input);

  estimator->counts[estimator->next] = input->count;
  estimator->next = (estimator->next + 1u) % SPEED_RING_SIZE;
  if (estimator->filled < SPEED_RING_SIZE)
  {
    estimator->filled++;
  }

  if (bad != SPEED_ESTIMATE_VALID)
  {
    estimator->good_streak = 0u;
    estimator->last_bad_reason = bad;
    estimator->bad_update_count++;
  }
  else if (estimator->good_streak < SPEED_RING_SIZE)
  {
    estimator->good_streak++;
  }

  /* Full ring: counts[next] is now the oldest, SPEED_WINDOW_MS updates ago. */
  estimator->speed_units = (estimator->filled == SPEED_RING_SIZE)
                               ? SpeedEstimator_CountDifference(input->count, estimator->counts[estimator->next])
                               : 0;

  estimator->valid = (estimator->filled == SPEED_RING_SIZE) && (estimator->good_streak >= SPEED_RING_SIZE);
  if (estimator->valid)
  {
    estimator->reason = SPEED_ESTIMATE_VALID;
  }
  else if (estimator->good_streak < estimator->filled)
  {
    estimator->reason = estimator->last_bad_reason;  /* a bad update is still in the window */
  }
  else
  {
    estimator->reason = SPEED_ESTIMATE_STARTING;
  }

  if (estimator->valid)
  {
    estimator->invalid_ms = 0u;
    estimator->was_valid = true;
  }
  else if (first || estimator->was_valid)
  {
    estimator->invalid_since_ms = input->now_ms;  /* invalid from this update on */
    estimator->invalid_ms = 0u;
    estimator->was_valid = false;
  }
  else
  {
    estimator->invalid_ms = input->now_ms - estimator->invalid_since_ms;
  }

  estimator->previous_ms = input->now_ms;
  estimator->previous_count = input->count;
}
