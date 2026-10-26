/**
  ******************************************************************************
  * @file    tb6612.h
  * @brief   Thin hardware layer for the Toshiba TB6612FNG H-bridge: writes a
  *          MotorOutput (from motor_control.h) to the PWM compare register
  *          and the IN1, IN2 and STBY pins. All the decisions are made in
  *          host-tested pure C: ramp, reversal and watchdog in motor_control.c,
  *          and the output stage's own guards (duty cap, direction change,
  *          timer period) in tb6612_plan.c.
  *
  * TB6612FNG truth table (Toshiba datasheet), per channel:
  *   STBY low                       -> outputs off (high impedance): coast
  *   IN1 = IN2 = low                -> outputs off: coast
  *   IN1 high, IN2 low, PWM high    -> drive one way ("CW")
  *   IN1 low, IN2 high, PWM high    -> drive the other way ("CCW")
  *   IN1 != IN2, PWM low            -> short brake (both outputs low)
  * So during the low part of each PWM period the motor is short-braked
  * ("slow decay"), not coasting.
  *
  * Our board has both channels in parallel (PWMA+PWMB, AIN1+BIN1, AIN2+BIN2),
  * so one set of pins drives both.
  *
  * No globals: the pins and timer are passed in a TB6612_Device.
  ******************************************************************************
  */
#ifndef TB6612_H
#define TB6612_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "motor_control.h"
#include "tb6612_plan.h"

typedef struct
{
  TIM_HandleTypeDef *pwm_timer;
  uint32_t           pwm_channel;        /* TIM_CHANNEL_x */
  GPIO_TypeDef      *in1_port;
  uint16_t           in1_pin;
  GPIO_TypeDef      *in2_port;
  uint16_t           in2_pin;
  GPIO_TypeDef      *standby_port;
  uint16_t           standby_pin;
  /* false: forward = IN1 high / IN2 low. true: swapped, for when the bench
   * test shows the wheels turn backwards on a positive command. */
  bool               invert_direction;
} TB6612_Device;

/* Puts the bridge in the safe state (STBY low, IN1 = IN2 = low), sets the
 * duty to 0 and only then starts the PWM. With the compare value at 0 the
 * PWM output is held low from the first instant (PWM mode 1: high only while
 * counter < compare), so the pin never pulses high. For TIM1, which is an
 * advanced timer, HAL_TIM_PWM_Start also sets the main output enable (MOE).
 * The timer then runs forever; the motor is stopped with STBY and the IN
 * pins, never by stopping the timer. */
HAL_StatusTypeDef TB6612_Start(const TB6612_Device *device);

/* Writes one MotorOutput. Call once per loop iteration. Reads the IN outputs,
 * the compare register and the timer period, lets TB6612_Plan decide (duty
 * cap, coast on a direction change with a duty still set, period check; see
 * tb6612_plan.h) and writes the result. Disabling writes STBY first, as the
 * single most decisive pin. Returns the plan, so the caller can see whether
 * a guard fired (TB6612_PlanFaultReason). */
TB6612_PinPlan TB6612_Apply(const TB6612_Device *device, const MotorOutput *output);

#endif /* TB6612_H */
