/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Thin hardware layer for the steering servo on TIM15 CH2 (PA3):
  *          writes a pulse width to the compare register. All the decisions
  *          are made in host-tested pure C (steering_control.c): the state
  *          machine, the mapping, the endpoint limits, and the output guard
  *          that this layer runs on every write.
  *
  * Timer (CubeMX, MX_TIM15_Init): 32 MHz / (PSC 31 + 1) = 1 MHz tick, ARR
  * 19999 = 20 ms period, PWM mode 1, so the compare value is the pulse
  * width in us. Output-compare preload is on (HAL_TIM_PWM_ConfigChannel
  * always sets OC2PE): a new pulse width takes effect at the next period, so
  * a pulse is never cut short or stretched mid-period. A cut-short pulse
  * would itself be a command, toward one endpoint or beyond it.
  *
  * No globals: the timer is passed in a Servo_Device.
  ******************************************************************************
  */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "steering_control.h"

typedef struct
{
  TIM_HandleTypeDef *pwm_timer;
  uint32_t           pwm_channel;  /* TIM_CHANNEL_x */
} Servo_Device;

/* Sets the compare value to 0 and only then starts the PWM, so the line is
 * driven low (no pulses) from the first instant: PWM mode 1 is high only
 * while counter < compare. TIM15 has a break unit, so HAL_TIM_PWM_Start
 * also sets MOE. The timer then runs forever; "no pulses" is compare 0. */
HAL_StatusTypeDef Servo_Start(const Servo_Device *device);

/* Writes one pulse width (us, 0 = no pulses). Reads the timer's real tick
 * rate and period, lets Steering_PlanOutput decide (refuse a wrong timer,
 * clamp to the endpoints) and writes the result. Returns the plan, so the
 * caller can see whether a guard fired. Call once per loop iteration. */
ServoPlan Servo_Apply(const Servo_Device *device, uint32_t pulse_us);

#endif /* SERVO_H */
