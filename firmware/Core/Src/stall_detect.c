/**
  ******************************************************************************
  * @file    stall_detect.c
  * @brief   Stalled-motor detection (see the header).
  ******************************************************************************
  */
#include "stall_detect.h"

void StallDetect_Init(StallDetector *detector)
{
  *detector = (StallDetector){0};
}

/* Either rule fired: stop at once and latch, until m 0 / v 0. reason says
 * which rule (MOTOR_LATCH_STALL_LOCKED or MOTOR_LATCH_STALL_SATURATION). */
static void StallDetect_Stall(StallDetector *detector, MotorController *motor, MotorLatchReason reason,
                              uint32_t now_ms)
{
  detector->holding = false;
  detector->sat_holding = false;
  detector->stall_count++;
  if (reason == MOTOR_LATCH_STALL_SATURATION)
  {
    detector->saturation_stall_count++;
  }
  else
  {
    detector->locked_stall_count++;
  }
  MotorController_Latch(motor, reason);

  /* The oldest of the last STALL_REPEAT_COUNT stalls (this one included) is
   * the oldest stored one, at next_slot; if it is within the window, this
   * stall is one too many. Then this one takes its slot. */
  uint32_t oldest = detector->stall_times_ms[detector->next_slot];
  detector->stall_times_ms[detector->next_slot] = now_ms;
  detector->next_slot = (detector->next_slot + 1u) % (STALL_REPEAT_COUNT - 1u);
  if (detector->stall_count >= STALL_REPEAT_COUNT && now_ms - oldest <= STALL_REPEAT_WINDOW_MS)
  {
    MotorController_LockOut(motor, MOTOR_FAULT_REPEATED_STALL);
  }
}

void StallDetect_Update(StallDetector *detector, MotorController *motor, const SpeedController *speed,
                        const SpeedEstimator *estimate, uint32_t now_ms)
{
  bool running = (motor->state == MOTOR_STATE_RUNNING);
  int32_t duty = motor->applied_ticks;
  int32_t measured = estimate->speed_units;

  /* Rule 1: duty on, wheel not turning. */
  bool stalled_now = running &&
                     (duty >= STALL_MIN_DUTY_TICKS || duty <= -STALL_MIN_DUTY_TICKS) &&
                     estimate->valid &&
                     (measured < STALL_MAX_SPEED_UNITS && measured > -STALL_MAX_SPEED_UNITS);
  if (!stalled_now)
  {
    detector->holding = false;
  }
  else
  {
    if (!detector->holding)
    {
      detector->holding = true;
      detector->since_ms = now_ms;
    }
    if (now_ms - detector->since_ms >= STALL_HOLD_MS)
    {
      StallDetect_Stall(detector, motor, MOTOR_LATCH_STALL_LOCKED, now_ms);
      return;
    }
  }

  /* Rule 2 ("v" only): at the cap in the setpoint's direction, and the speed
   * in that direction below half the setpoint. No effective setpoint
   * (direction 0) never passes the cap test. */
  int32_t setpoint = speed->effective_units;
  int32_t direction = (setpoint > 0) - (setpoint < 0);
  bool saturated_now = running &&
                       speed->mode == SPEED_MODE_CLOSED_LOOP &&
                       direction * duty >= MOTOR_DUTY_CAP_TICKS &&
                       estimate->valid &&
                       direction * measured * STALL_SAT_SPEED_DENOMINATOR <
                           direction * setpoint * STALL_SAT_SPEED_NUMERATOR;
  if (!saturated_now)
  {
    detector->sat_holding = false;
    return;
  }
  if (!detector->sat_holding)
  {
    detector->sat_holding = true;
    detector->sat_since_ms = now_ms;
  }
  if (now_ms - detector->sat_since_ms >= STALL_SAT_HOLD_MS)
  {
    StallDetect_Stall(detector, motor, MOTOR_LATCH_STALL_SATURATION, now_ms);
  }
}
