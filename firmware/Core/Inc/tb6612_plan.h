/**
  ******************************************************************************
  * @file    tb6612_plan.h
  * @brief   The output stage's decisions for the TB6612FNG, with no hardware
  *          access: given the MotorOutput the state machine asks for and what
  *          the pins and compare register hold now, says what to write. This
  *          is the second layer of protection behind motor_control.c, so it
  *          does not trust its input. tb6612.c reads the registers, calls
  *          TB6612_Plan and writes the result.
  *
  * Rules, in order (each one ends the decision):
  *   1. Driver disabled asked for            -> disabled (STBY low, IN1 = IN2
  *                                              = low, compare 0)
  *   2. Timer period != MOTOR_PWM_PERIOD_TICKS -> disabled, period_refused.
  *      Duty is in ticks, so with another period the cap would mean another
  *      percentage (a lower ARR makes 480 ticks MORE than 30 %).
  *   3. The IN pins would change and the compare register is not 0
  *                                           -> coast: STBY high, IN1 = IN2 =
  *                                              low, compare 0, guard_coasted.
  *      With compare preload on, a compare value written now only takes
  *      effect at the next PWM period (up to 50 us), but the IN pins change at
  *      once: the old duty would drive the bridge in the new direction for
  *      up to 50 us. IN1 = IN2 = low is OFF whatever PWM does (TB6612FNG
  *      datasheet, H-SW control function table), so coasting is safe at
  *      once. The new direction is applied on a later call (rule 4).
  *   4. The IN pins would change, compare is 0 -> new pins, compare 0 ("never
  *      drive in the same ms the direction pins change"; the 0 was written
  *      on an earlier call, so it is already the active value). If a
  *      non-zero duty was asked for, drive_on_pin_change is set: the state
  *      machine always asks for 0 here, and the duty would otherwise be
  *      applied on the next call with no ramp.
  *   5. Otherwise                            -> same pins, compare = duty,
  *      clamped to MOTOR_DUTY_CAP_TICKS (capped is set if it had to be).
  *
  * Cannot coast forever: rule 3 needs a non-zero compare and always writes
  * 0, so the call after a coast can't coast again. A constant request is
  * reached within 2 calls of a coast (rule 4, then rule 5).
  *
  * The state machine never triggers rules 2, 3, drive_on_pin_change or the
  * clamp in rule 5 (tests/test_tb6612_plan.c checks this against the old
  * output stage, ms by ms). If one fires, motor_control.c has a bug: main.c then puts the
  * controller in FAULT (TB6612_PlanFaultReason).
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_tb6612_plan.c.
  ******************************************************************************
  */
#ifndef TB6612_PLAN_H
#define TB6612_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control.h"

/* What to write to the TB6612 pins and the PWM compare register. */
typedef struct
{
  bool     standby_high;
  bool     in1_high;
  bool     in2_high;
  uint32_t compare_ticks;
  /* Guards that fired in this call. Each one means the request was wrong. */
  bool     period_refused;  /* rule 2 */
  bool     guard_coasted;   /* rule 3 */
  bool     drive_on_pin_change;  /* rule 4 with a non-zero duty asked (applied as 0) */
  bool     capped;          /* a duty above MOTOR_DUTY_CAP_TICKS was asked for */
} TB6612_PinPlan;

/* previous_in1_high / previous_in2_high: what the IN outputs are set to now.
 * previous_compare_ticks: the compare register as read back now (the last
 * value written). period_ticks: timer ARR + 1. invert_direction: forward is
 * IN1 low / IN2 high instead of IN1 high / IN2 low. */
TB6612_PinPlan TB6612_Plan(const MotorOutput *requested, bool invert_direction,
                           bool previous_in1_high, bool previous_in2_high,
                           uint32_t previous_compare_ticks, uint32_t period_ticks);

/* MOTOR_FAULT_NONE if no guard fired, else the reason to pass to
 * MotorController_LockOut. If several fired: period, then direction, then cap. */
MotorFaultReason TB6612_PlanFaultReason(const TB6612_PinPlan *plan);

#endif /* TB6612_PLAN_H */
