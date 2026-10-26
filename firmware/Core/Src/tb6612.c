/**
  ******************************************************************************
  * @file    tb6612.c
  * @brief   TB6612FNG pin and PWM writes (see the header).
  ******************************************************************************
  */
#include "tb6612.h"

/* Reads back what a push-pull output is set to (ODR, not the pin level). */
static bool TB6612_OutputIsHigh(GPIO_TypeDef *port, uint16_t pin)
{
  return (port->ODR & pin) != 0u;
}

static void TB6612_Disable(const TB6612_Device *device)
{
  HAL_GPIO_WritePin(device->standby_port, device->standby_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->in1_port, device->in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->in2_port, device->in2_pin, GPIO_PIN_RESET);
  /* With output-compare preload on, this takes effect at the next period
   * start (at most 50 us later); STBY above has already disabled the bridge. */
  __HAL_TIM_SET_COMPARE(device->pwm_timer, device->pwm_channel, 0u);
}

HAL_StatusTypeDef TB6612_Start(const TB6612_Device *device)
{
  TB6612_Disable(device);
  return HAL_TIM_PWM_Start(device->pwm_timer, device->pwm_channel);
}

TB6612_PinPlan TB6612_Apply(const TB6612_Device *device, const MotorOutput *output)
{
  /* The compare register reads back the last value written (RM0394, TIM1
   * capture/compare channels: with preload on, reads access the preload
   * register; not verified on the page yet). The plan doesn't depend on
   * which register a read returns: the last write was a whole loop iteration
   * ago, so the update event has already copied it to the active register. */
  TB6612_PinPlan plan = TB6612_Plan(output, device->invert_direction,
                                    TB6612_OutputIsHigh(device->in1_port, device->in1_pin),
                                    TB6612_OutputIsHigh(device->in2_port, device->in2_pin),
                                    __HAL_TIM_GET_COMPARE(device->pwm_timer, device->pwm_channel),
                                    __HAL_TIM_GET_AUTORELOAD(device->pwm_timer) + 1u);
  if (!plan.standby_high)
  {
    TB6612_Disable(device);
    return plan;
  }

  HAL_GPIO_WritePin(device->in1_port, device->in1_pin, plan.in1_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->in2_port, device->in2_pin, plan.in2_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->standby_port, device->standby_pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(device->pwm_timer, device->pwm_channel, plan.compare_ticks);
  return plan;
}
