/**
  ******************************************************************************
  * @file    steering_control.c
  * @brief   Steering servo state machine, mapping and output guard (see the
  *          header).
  ******************************************************************************
  */
#include "steering_control.h"

static uint32_t Steering_ClampPulse(uint32_t pulse_us)
{
  if (pulse_us < (uint32_t)SERVO_PULSE_MIN_US) { return (uint32_t)SERVO_PULSE_MIN_US; }
  if (pulse_us > (uint32_t)SERVO_PULSE_MAX_US) { return (uint32_t)SERVO_PULSE_MAX_US; }
  return pulse_us;
}

uint32_t Steering_PulseForPercent(int32_t percent, bool invert_direction)
{
  if (percent > STEERING_MAX_PERCENT)  { percent = STEERING_MAX_PERCENT; }
  if (percent < -STEERING_MAX_PERCENT) { percent = -STEERING_MAX_PERCENT; }
  if (invert_direction)
  {
    percent = -percent;
  }
  /* Each half scaled on its own, so the centre and both endpoints are exact
   * even when the calibrated halves differ in width. Integer division rounds
   * toward the centre. */
  int32_t pulse_us;
  if (percent >= 0)
  {
    pulse_us = SERVO_PULSE_CENTER_US + ((SERVO_PULSE_MAX_US - SERVO_PULSE_CENTER_US) * percent) / STEERING_MAX_PERCENT;
  }
  else
  {
    pulse_us = SERVO_PULSE_CENTER_US - ((SERVO_PULSE_CENTER_US - SERVO_PULSE_MIN_US) * -percent) / STEERING_MAX_PERCENT;
  }
  return Steering_ClampPulse((uint32_t)pulse_us);
}

void SteeringController_Init(SteeringController *controller, uint32_t now_ms, bool invert_direction)
{
  controller->state = STEERING_STATE_OFF;
  controller->invert_direction = invert_direction;
  controller->requested_percent = 0;
  controller->last_command_ms = now_ms;
  controller->pulse_us = 0u;
  controller->valid_command_count = 0u;
  controller->timeout_count = 0u;
}

void SteeringController_AcceptCommand(SteeringController *controller, int32_t percent, uint32_t now_ms)
{
  controller->valid_command_count++;
  controller->requested_percent = percent;
  if (controller->state == STEERING_STATE_FAULT)
  {
    return;  /* counted above, never acted on: only a reset leaves FAULT */
  }
  controller->state = STEERING_STATE_ACTIVE;
  controller->last_command_ms = now_ms;
}

uint32_t SteeringController_Update(SteeringController *controller, uint32_t now_ms)
{
  switch (controller->state)
  {
    case STEERING_STATE_ACTIVE:
      /* Unsigned difference: stays correct when the ms tick wraps. */
      if (now_ms - controller->last_command_ms >= STEERING_COMMAND_TIMEOUT_MS)
      {
        controller->timeout_count++;
        controller->state = STEERING_STATE_TIMED_OUT;
        controller->pulse_us = (uint32_t)SERVO_PULSE_CENTER_US;
      }
      else
      {
        controller->pulse_us = Steering_PulseForPercent(controller->requested_percent, controller->invert_direction);
      }
      break;

    case STEERING_STATE_TIMED_OUT:
      controller->pulse_us = (uint32_t)SERVO_PULSE_CENTER_US;
      break;

    case STEERING_STATE_OFF:
    case STEERING_STATE_FAULT:
    default:
      controller->pulse_us = 0u;
      break;
  }
  return controller->pulse_us;
}

void SteeringController_LockOut(SteeringController *controller)
{
  controller->state = STEERING_STATE_FAULT;
  controller->pulse_us = 0u;
}

ServoPlan Steering_PlanOutput(uint32_t pulse_us, uint32_t timer_tick_hz, uint32_t period_ticks)
{
  ServoPlan plan = { .compare_ticks = 0u, .timer_refused = false, .out_of_range = false };

  /* Rule 1 */
  if (timer_tick_hz != SERVO_TIMER_TICK_HZ || period_ticks != SERVO_PWM_PERIOD_TICKS)
  {
    plan.timer_refused = true;
    return plan;
  }
  /* Rule 2 */
  if (pulse_us == 0u)
  {
    return plan;
  }
  /* Rule 3 */
  uint32_t clamped_us = Steering_ClampPulse(pulse_us);
  plan.out_of_range = (clamped_us != pulse_us);
  /* Rule 4 */
  plan.compare_ticks = clamped_us;
  return plan;
}
