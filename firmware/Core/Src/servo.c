/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Steering servo PWM writes on TIM15 (see the header).
  ******************************************************************************
  */
#include "servo.h"

/* TIM15 counts at the APB2 timer clock: PCLK2 when the APB2 prescaler is 1,
 * twice PCLK2 otherwise (the usual STM32 timer clock rule; RM0394 RCC clock
 * tree, section number not verified). Here APB2 = /1, so 32 MHz. */
static uint32_t Servo_TimerTickHz(const Servo_Device *device)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
  {
    timer_clock_hz *= 2u;
  }
  return timer_clock_hz / (device->pwm_timer->Instance->PSC + 1u);
}

HAL_StatusTypeDef Servo_Start(const Servo_Device *device)
{
  __HAL_TIM_SET_COMPARE(device->pwm_timer, device->pwm_channel, 0u);
  return HAL_TIM_PWM_Start(device->pwm_timer, device->pwm_channel);
}

ServoPlan Servo_Apply(const Servo_Device *device, uint32_t pulse_us)
{
  ServoPlan plan = Steering_PlanOutput(pulse_us, Servo_TimerTickHz(device),
                                       __HAL_TIM_GET_AUTORELOAD(device->pwm_timer) + 1u);
  __HAL_TIM_SET_COMPARE(device->pwm_timer, device->pwm_channel, plan.compare_ticks);
  return plan;
}
