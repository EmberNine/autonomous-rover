/**
  ******************************************************************************
  * @file    as5600.h
  * @brief   AS5600 magnetic angle sensor: register reads + angle unwrapping.
  *
  * Everything here is in raw sensor counts (4096 per revolution). Converting
  * counts to meters is done on the Pi using vehicle.cfg, so wheel size never
  * gets baked into the firmware.
  ******************************************************************************
  */
#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define AS5600_I2C_ADDRESS     (0x36 << 1)  /* HAL wants the 7-bit address shifted left by 1 */
#define AS5600_COUNTS_PER_REV  4096

/* A normal read takes ~0.1 ms at 400 kHz. HAL's tick-based timeout can expire
 * anywhere between N and N+1 ms, so 2 ms is the smallest value that can't
 * fire spuriously while still limiting how long a stuck bus stalls the loop. */
#define AS5600_I2C_TIMEOUT_MS  2

/* The three magnet flags from the STATUS register (0x0B). */
typedef struct
{
  uint8_t magnet_detected;    /* MD: 0 means no magnet, and the angle is garbage */
  uint8_t magnet_too_weak;    /* ML: telemetry only, never a validity or fault input. Not seen set on the
                               * carrier build (2026-10-14, ~25 s of driving), though AGC reaches 128 (its
                               * 3.3 V maximum) at the weakest point of each turn: the field is weak-ish */
  uint8_t magnet_too_strong;  /* MH */
} AS5600_Status;

/* Turns the 0..4095 angle, which wraps every revolution, into a running
 * count that keeps going up (or down) across revolutions. */
typedef struct
{
  uint16_t previous_raw_angle;
  bool     has_first_sample;
  int32_t  continuous_count;   /* see as5600.c for why 32 bits is enough */
} AS5600_Unwrapper;

/* Each read writes its output only when it returns HAL_OK, so a failed read
 * never leaves a half-written value behind. */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *i2c_handle, uint16_t *raw_angle);
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *i2c_handle, uint16_t *magnitude);
HAL_StatusTypeDef AS5600_ReadAgc(I2C_HandleTypeDef *i2c_handle, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *i2c_handle, AS5600_Status *status);

void AS5600_UnwrapperInit(AS5600_Unwrapper *unwrapper);
void AS5600_UnwrapperUpdate(AS5600_Unwrapper *unwrapper, uint16_t raw_angle);

#endif /* AS5600_H */
