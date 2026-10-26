/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   Motor state machine (see the header).
  ******************************************************************************
  */
#include "motor_control.h"

/* ---- Motor state machine ------------------------------------------------ */

static MotorDirection MotorDirectionOf(int32_t signed_ticks)
{
  if (signed_ticks > 0) { return MOTOR_DIRECTION_FORWARD; }
  if (signed_ticks < 0) { return MOTOR_DIRECTION_REVERSE; }
  return MOTOR_DIRECTION_COAST;
}

static MotorOutput MotorOutput_Disabled(void)
{
  MotorOutput output = { .driver_enabled = false, .direction = MOTOR_DIRECTION_COAST, .duty_ticks = 0u };
  return output;
}

static MotorOutput MotorOutput_Enabled(MotorDirection direction, int32_t signed_ticks)
{
  uint32_t magnitude = (signed_ticks < 0) ? (uint32_t)(-signed_ticks) : (uint32_t)signed_ticks;
  MotorOutput output = { .driver_enabled = true, .direction = direction, .duty_ticks = magnitude };
  return output;
}

/* Moves value toward goal by at most step (step >= 0), without overshooting. */
static int32_t MotorRamp_StepToward(int32_t value, int32_t goal, int32_t step)
{
  if (value < goal)
  {
    return (goal - value > step) ? value + step : goal;
  }
  if (value > goal)
  {
    return (value - goal > step) ? value - step : goal;
  }
  return value;
}

/* Back to "driver disabled" with nothing pending. The caller sets the state. */
static void MotorController_ClearMotion(MotorController *controller)
{
  controller->target_ticks = 0;
  controller->applied_ticks = 0;
  controller->bridge_direction = MOTOR_DIRECTION_COAST;
  controller->dwell_active = false;
}

void MotorController_Init(MotorController *controller, uint32_t now_ms)
{
  controller->state = MOTOR_STATE_LATCHED;  /* needs "m 0" before anything moves */
  controller->requested_percent = 0;
  MotorController_ClearMotion(controller);
  controller->dwell_start_ms = now_ms;
  controller->last_command_ms = now_ms;
  controller->last_update_ms = now_ms;
  controller->fault_reason = MOTOR_FAULT_NONE;
  controller->latch_reason = MOTOR_LATCH_BOOT;
  controller->valid_command_count = 0u;
  controller->rejected_command_count = 0u;
  controller->clamped_command_count = 0u;
  controller->watchdog_trip_count = 0u;
  controller->latch_count = 0u;
}

void MotorController_AcceptCommand(MotorController *controller, int32_t percent, uint32_t now_ms)
{
  controller->valid_command_count++;
  controller->last_command_ms = now_ms;
  controller->requested_percent = percent;

  int32_t magnitude_percent = (percent < 0) ? -percent : percent;
  if (magnitude_percent > MOTOR_DUTY_CAP_PERCENT)
  {
    controller->clamped_command_count++;
    magnitude_percent = MOTOR_DUTY_CAP_PERCENT;
  }
  int32_t capped_ticks = magnitude_percent * MOTOR_TICKS_PER_PERCENT;
  if (percent < 0)
  {
    capped_ticks = -capped_ticks;
  }

  if (controller->state == MOTOR_STATE_FAULT)
  {
    return;  /* counted above, never acted on: only a reset leaves FAULT */
  }
  if (controller->state == MOTOR_STATE_LATCHED)
  {
    /* Only "m 0" re-arms; anything else is ignored (but was a valid command). */
    if (percent == 0)
    {
      controller->state = MOTOR_STATE_OFF;
      controller->latch_reason = MOTOR_LATCH_NONE;
      controller->target_ticks = 0;
    }
    return;
  }
  controller->target_ticks = capped_ticks;
}

void MotorController_CountRejected(MotorController *controller)
{
  controller->rejected_command_count++;
}

MotorOutput MotorController_Update(MotorController *controller, uint32_t now_ms)
{
  /* Unsigned differences stay correct when the ms tick wraps (49.7 days). */
  uint32_t elapsed_ms = now_ms - controller->last_update_ms;
  controller->last_update_ms = now_ms;
  if (elapsed_ms > MOTOR_RAMP_MAX_STEP_MS)
  {
    elapsed_ms = MOTOR_RAMP_MAX_STEP_MS;
  }

  switch (controller->state)
  {
    case MOTOR_STATE_FAULT:
    case MOTOR_STATE_LATCHED:
      return MotorOutput_Disabled();

    case MOTOR_STATE_OFF:
      if (controller->target_ticks == 0)
      {
        return MotorOutput_Disabled();
      }
      /* Wake-up: enable the driver and set the direction pins with duty 0
       * for this one ms, so STBY is settled before any current flows. The
       * ramp starts on the next update. */
      controller->state = MOTOR_STATE_RUNNING;
      controller->bridge_direction = MotorDirectionOf(controller->target_ticks);
      controller->applied_ticks = 0;
      controller->dwell_active = false;
      return MotorOutput_Enabled(controller->bridge_direction, 0);

    case MOTOR_STATE_RUNNING:
    default:
      break;
  }

  /* Watchdog: stop at once, no ramp, and latch. */
  if (now_ms - controller->last_command_ms >= MOTOR_COMMAND_TIMEOUT_MS)
  {
    controller->watchdog_trip_count++;
    controller->state = MOTOR_STATE_LATCHED;
    controller->latch_reason = MOTOR_LATCH_WATCHDOG;
    MotorController_ClearMotion(controller);
    return MotorOutput_Disabled();
  }

  /* A target on the other side of zero is approached via zero. */
  bool target_on_bridge_side = (controller->target_ticks == 0) ||
                               (MotorDirectionOf(controller->target_ticks) == controller->bridge_direction);
  int32_t ramp_goal_ticks = target_on_bridge_side ? controller->target_ticks : 0;
  int32_t ramp_step_ticks = (int32_t)elapsed_ms * MOTOR_RAMP_TICKS_PER_MS;
  controller->applied_ticks = MotorRamp_StepToward(controller->applied_ticks, ramp_goal_ticks, ramp_step_ticks);

  if (target_on_bridge_side)
  {
    controller->dwell_active = false;  /* a reversal that was changed back before it happened */
  }

  if (controller->applied_ticks == 0)
  {
    if (controller->target_ticks == 0)
    {
      /* Ramp finished at zero with nothing else to do: stopped = driver disabled. */
      controller->state = MOTOR_STATE_OFF;
      MotorController_ClearMotion(controller);
      return MotorOutput_Disabled();
    }
    if (!target_on_bridge_side)
    {
      if (!controller->dwell_active)
      {
        controller->dwell_active = true;
        controller->dwell_start_ms = now_ms;
      }
      else if (now_ms - controller->dwell_start_ms >= MOTOR_REVERSE_DWELL_MS)
      {
        /* Switch the pins in a ms of their own, with duty 0. The ramp in
         * the new direction starts on the next update. */
        controller->dwell_active = false;
        controller->bridge_direction = MotorDirectionOf(controller->target_ticks);
        return MotorOutput_Enabled(controller->bridge_direction, 0);
      }
      /* Dwell: driver enabled, old direction, duty 0 = short brake. */
      return MotorOutput_Enabled(controller->bridge_direction, 0);
    }
  }

  return MotorOutput_Enabled(controller->bridge_direction, controller->applied_ticks);
}

void MotorController_LockOut(MotorController *controller, MotorFaultReason reason)
{
  if (controller->state != MOTOR_STATE_FAULT)
  {
    controller->fault_reason = reason;  /* the first cause is the useful one */
  }
  controller->state = MOTOR_STATE_FAULT;
  controller->latch_reason = MOTOR_LATCH_NONE;  /* FAULT, not a latch */
  MotorController_ClearMotion(controller);
}

void MotorController_AcceptSpeedCommand(MotorController *controller, int32_t setpoint_units, uint32_t now_ms)
{
  controller->valid_command_count++;
  controller->last_command_ms = now_ms;   /* "v" feeds the watchdog, like "m" */
  if (controller->state == MOTOR_STATE_LATCHED && setpoint_units == 0)
  {
    controller->state = MOTOR_STATE_OFF;  /* "v 0" re-arms, like "m 0" */
    controller->latch_reason = MOTOR_LATCH_NONE;
    controller->target_ticks = 0;
  }
}

void MotorController_AcceptDriveCommand(MotorController *controller, bool arm, uint32_t now_ms)
{
  controller->valid_command_count++;
  controller->last_command_ms = now_ms;   /* DRIVE feeds the watchdog, like "v" */
  if (controller->state == MOTOR_STATE_LATCHED && arm)
  {
    controller->state = MOTOR_STATE_OFF;
    controller->latch_reason = MOTOR_LATCH_NONE;
    controller->target_ticks = 0;
  }
}

void MotorController_SetClosedLoopTarget(MotorController *controller, int32_t target_ticks)
{
  if (controller->state != MOTOR_STATE_OFF && controller->state != MOTOR_STATE_RUNNING)
  {
    return;
  }
  if (target_ticks > MOTOR_DUTY_CAP_TICKS)  { target_ticks = MOTOR_DUTY_CAP_TICKS; }
  if (target_ticks < -MOTOR_DUTY_CAP_TICKS) { target_ticks = -MOTOR_DUTY_CAP_TICKS; }
  controller->target_ticks = target_ticks;
}

void MotorController_Latch(MotorController *controller, MotorLatchReason reason)
{
  if (controller->state == MOTOR_STATE_FAULT || controller->state == MOTOR_STATE_LATCHED)
  {
    return;
  }
  controller->latch_count++;
  controller->state = MOTOR_STATE_LATCHED;
  controller->latch_reason = reason;
  MotorController_ClearMotion(controller);
}
