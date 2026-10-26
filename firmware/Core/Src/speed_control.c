/**
  ******************************************************************************
  * @file    speed_control.c
  * @brief   Closed-loop wheel speed, PI through motor_control (see the header).
  ******************************************************************************
  */
#include "speed_control.h"

#define CAP_Q8  ((int32_t)MOTOR_DUTY_CAP_TICKS * SPEED_Q8_ONE)

static int32_t SpeedControl_Sign(int32_t value)
{
  return (value > 0) - (value < 0);
}

static int32_t SpeedControl_Clamp(int32_t value, int32_t low, int32_t high)
{
  if (value < low)  { return low; }
  if (value > high) { return high; }
  return value;
}

/* Q8 ticks -> ticks, rounded half away from zero. */
static int32_t SpeedControl_RoundQ8(int32_t value_q8)
{
  return (value_q8 >= 0) ? (value_q8 + SPEED_Q8_ONE / 2) / SPEED_Q8_ONE
                         : -((-value_q8 + SPEED_Q8_ONE / 2) / SPEED_Q8_ONE);
}

/* Nothing in progress: stopped, starting or gated. */
static void SpeedControl_Reset(SpeedController *speed)
{
  speed->integrator_q8 = 0;
  speed->direction = 0;
  speed->gate_waiting = false;
  speed->due_set = false;
}

void SpeedControl_Init(SpeedController *speed)
{
  *speed = (SpeedController){0};
  speed->mode = SPEED_MODE_OPEN_LOOP;
  speed->preload_ticks = SPEED_PRELOAD_TICKS;
}

void SpeedControl_SelectOpenLoop(SpeedController *speed)
{
  speed->mode = SPEED_MODE_OPEN_LOOP;
  speed->setpoint_units = 0;
  speed->effective_units = 0;
  speed->output_ticks = 0;
  SpeedControl_Reset(speed);
}

void SpeedControl_SetSetpoint(SpeedController *speed, int32_t setpoint_units, int32_t applied_ticks)
{
  int32_t magnitude = (setpoint_units < 0) ? -setpoint_units : setpoint_units;
  int32_t effective = (magnitude < SPEED_MIN_SETPOINT_UNITS) ? 0 : setpoint_units;
  bool was_open = (speed->mode == SPEED_MODE_OPEN_LOOP);
  int32_t previous = speed->effective_units;

  speed->mode = SPEED_MODE_CLOSED_LOOP;
  speed->setpoint_units = setpoint_units;
  speed->effective_units = effective;   /* a keepalive (same value) changes nothing below */

  if (effective == 0)
  {
    SpeedControl_Reset(speed);
    return;
  }
  if (was_open)
  {
    /* m -> v while moving the same way: carry on from the duty applied now. */
    if (applied_ticks != 0 && SpeedControl_Sign(applied_ticks) == SpeedControl_Sign(effective))
    {
      speed->direction = SpeedControl_Sign(effective);
      speed->integrator_q8 = SpeedControl_Clamp(applied_ticks * SPEED_Q8_ONE, -CAP_Q8, CAP_Q8);
      speed->gate_waiting = false;
      speed->due_set = false;
      return;
    }
    SpeedControl_Reset(speed);  /* from rest, or the other way: a (gated) start */
    return;
  }
  if (SpeedControl_Sign(effective) != SpeedControl_Sign(previous))
  {
    SpeedControl_Reset(speed);  /* from 0, or a sign change: a (gated) start */
  }
  /* Same sign, new magnitude: integrator and direction carry on. */
}

void SpeedControl_Update(SpeedController *speed, MotorController *motor, const SpeedEstimator *estimate,
                         uint32_t now_ms)
{
  if (speed->mode != SPEED_MODE_CLOSED_LOOP)
  {
    return;
  }
  if (motor->state == MOTOR_STATE_LATCHED || motor->state == MOTOR_STATE_FAULT)
  {
    SpeedControl_Reset(speed);
    speed->output_ticks = 0;
    return;  /* targets are ignored anyway; "v 0" / "m 0" re-arms */
  }
  if (speed->effective_units == 0)
  {
    SpeedControl_Reset(speed);
    speed->output_ticks = 0;
    MotorController_SetClosedLoopTarget(motor, 0);  /* coast, like m 0 */
    return;
  }

  /* Never act on an invalid estimate: hold the target, freeze the
   * integrator; give up (latch) if it stays invalid. */
  if (!estimate->valid)
  {
    if (estimate->invalid_ms >= SPEED_INVALID_HOLD_MS)
    {
      MotorController_Latch(motor, MOTOR_LATCH_ESTIMATE_INVALID);
      speed->enc_bad_count++;
      SpeedControl_Reset(speed);
      speed->output_ticks = 0;
    }
    return;
  }

  /* Start (from rest, after a sign change, after a latch): only once the
   * wheel has (nearly) stopped or already turns the new way. */
  if (speed->direction == 0)
  {
    int32_t wanted = SpeedControl_Sign(speed->effective_units);
    int32_t measured = estimate->speed_units;
    int32_t magnitude = (measured < 0) ? -measured : measured;
    if (magnitude > SPEED_GATE_UNITS && SpeedControl_Sign(measured) != wanted)
    {
      speed->gate_waiting = true;
      speed->output_ticks = 0;
      MotorController_SetClosedLoopTarget(motor, 0);
      return;
    }
    speed->gate_waiting = false;
    speed->direction = wanted;
    if (motor->applied_ticks == 0)
    {
      speed->integrator_q8 = wanted * speed->preload_ticks * SPEED_Q8_ONE;  /* from rest: breakaway */
    }
    else if (SpeedControl_Sign(motor->applied_ticks) == wanted)
    {
      speed->integrator_q8 = SpeedControl_Clamp(motor->applied_ticks * SPEED_Q8_ONE, -CAP_Q8, CAP_Q8);
    }
    else
    {
      speed->integrator_q8 = 0;
    }
    speed->due_set = false;  /* run the PI now */
  }

  if (speed->due_set && (int32_t)(now_ms - speed->next_due_ms) < 0)
  {
    return;
  }
  speed->next_due_ms = now_ms + SPEED_CONTROL_PERIOD_MS;
  speed->due_set = true;

  int32_t error = speed->effective_units - estimate->speed_units;
  int32_t low = (speed->direction > 0) ? 0 : -CAP_Q8;
  int32_t high = (speed->direction > 0) ? CAP_Q8 : 0;
  int32_t proportional_q8 = SPEED_KP_Q8 * error;

  bool may_integrate = (motor->state == MOTOR_STATE_RUNNING) && (motor->applied_ticks == motor->target_ticks);
  if (may_integrate)
  {
    /* Anti-windup: integrate only up to where P + I reaches the output
     * limit in the error's direction (so a saturated output sits exactly
     * at the limit), and never move I against the error. */
    int32_t candidate = SpeedControl_Clamp(speed->integrator_q8 + SPEED_KI_Q8 * error, low, high);
    if (error > 0)
    {
      int32_t limit = high - proportional_q8;
      if (candidate > limit) { candidate = (limit > speed->integrator_q8) ? limit : speed->integrator_q8; }
    }
    else if (error < 0)
    {
      int32_t limit = low - proportional_q8;
      if (candidate < limit) { candidate = (limit < speed->integrator_q8) ? limit : speed->integrator_q8; }
    }
    speed->integrator_q8 = candidate;
  }

  int32_t output_q8 = SpeedControl_Clamp(proportional_q8 + speed->integrator_q8, low, high);
  speed->error_units = error;
  speed->output_ticks = SpeedControl_RoundQ8(output_q8);
  MotorController_SetClosedLoopTarget(motor, speed->output_ticks);
}

SpeedControlState SpeedControl_State(const SpeedController *speed)
{
  if (speed->mode != SPEED_MODE_CLOSED_LOOP) { return SPEED_CTL_OPEN_LOOP; }
  if (speed->effective_units == 0)           { return SPEED_CTL_STOPPED; }
  if (speed->gate_waiting)                   { return SPEED_CTL_GATE; }
  return SPEED_CTL_RUNNING;
}

int32_t SpeedControl_IntegratorTicks(const SpeedController *speed)
{
  return SpeedControl_RoundQ8(speed->integrator_q8);
}
