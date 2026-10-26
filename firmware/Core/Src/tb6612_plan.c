/**
  ******************************************************************************
  * @file    tb6612_plan.c
  * @brief   Output-stage decisions for the TB6612FNG (see the header).
  *
  * Known edge, accepted: rule 4 relies on the 0 written by the previous call
  * already being the active compare value, i.e. on the previous call being at
  * least one PWM period (50 us) ago. Calls are normally 1 ms apart. In a
  * catch-up iteration after an overrun, two calls can be as close as one
  * iteration's work, which with a NACKing encoder and no IMU might be under
  * 50 us. Then, after a rule-3 coast, the rule-4 pin change on the next call
  * could still meet the old active duty for the rest of that PWM period.
  * That needs a state-machine bug (rule 3 never fires otherwise) AND a
  * sub-50-us catch-up iteration, and in this firmware it can't get as far
  * as rule 4: main.c latches FAULT as soon as rule 3 fires, so the next
  * call is a disable.
  ******************************************************************************
  */
#include "tb6612_plan.h"

static TB6612_PinPlan TB6612_PlanDisabled(void)
{
  TB6612_PinPlan plan = {
    .standby_high = false, .in1_high = false, .in2_high = false, .compare_ticks = 0u,
    .period_refused = false, .guard_coasted = false, .drive_on_pin_change = false, .capped = false };
  return plan;
}

TB6612_PinPlan TB6612_Plan(const MotorOutput *requested, bool invert_direction,
                           bool previous_in1_high, bool previous_in2_high,
                           uint32_t previous_compare_ticks, uint32_t period_ticks)
{
  TB6612_PinPlan plan = TB6612_PlanDisabled();

  /* Rule 1 */
  if (!requested->driver_enabled)
  {
    return plan;
  }

  /* Rule 2 */
  if (period_ticks != (uint32_t)MOTOR_PWM_PERIOD_TICKS)
  {
    plan.period_refused = true;
    return plan;
  }

  /* Checked before the pin rules so that a bad duty is reported even in a
   * call that doesn't apply it. */
  uint32_t duty_ticks = requested->duty_ticks;
  if (duty_ticks > (uint32_t)MOTOR_DUTY_CAP_TICKS)
  {
    plan.capped = true;
    duty_ticks = (uint32_t)MOTOR_DUTY_CAP_TICKS;
  }

  bool in1_high = (requested->direction == MOTOR_DIRECTION_FORWARD);
  bool in2_high = (requested->direction == MOTOR_DIRECTION_REVERSE);
  if (invert_direction)
  {
    bool swap = in1_high;
    in1_high = in2_high;
    in2_high = swap;
  }
  bool pins_change = (in1_high != previous_in1_high) || (in2_high != previous_in2_high);

  plan.standby_high = true;

  /* Rule 3: IN1 = IN2 = low and compare 0 (already set). */
  if (pins_change && previous_compare_ticks != 0u)
  {
    plan.guard_coasted = true;
    return plan;
  }

  plan.in1_high = in1_high;
  plan.in2_high = in2_high;

  /* Rule 4: compare 0 (already set). The state machine always asks for duty
   * 0 on the ms the pins change; a non-zero duty here would be applied on
   * the next ms with no ramp, so it is reported. */
  if (pins_change)
  {
    plan.drive_on_pin_change = (requested->duty_ticks != 0u);
    return plan;
  }

  /* Rule 5 */
  plan.compare_ticks = duty_ticks;
  return plan;
}

MotorFaultReason TB6612_PlanFaultReason(const TB6612_PinPlan *plan)
{
  if (plan->period_refused) { return MOTOR_FAULT_PERIOD_MISMATCH; }
  if (plan->guard_coasted || plan->drive_on_pin_change) { return MOTOR_FAULT_DIRECTION_GUARD; }
  if (plan->capped)         { return MOTOR_FAULT_CAP_GUARD; }
  return MOTOR_FAULT_NONE;
}
