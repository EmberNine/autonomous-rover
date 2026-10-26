/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688-P 6-axis IMU over 4-wire SPI: register access, init,
  *          and a one-transaction burst read of temperature + accel + gyro.
  *
  * Everything here is in raw sensor counts. Converting to deg/s, g and deg C
  * is done on the Pi (scale factors are in README.md), so the firmware never
  * has to know which units the rest of the system wants.
  *
  * Register addresses, bit fields and timings are from TDK datasheet
  * DS-000347 rev 1.2; section numbers are cited next to each one in
  * icm42688.c.
  ******************************************************************************
  */
#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define ICM42688_WHO_AM_I_VALUE  0x47  /* DS-000347 section 14.58 */

/* Same reasoning as AS5600_I2C_TIMEOUT_MS: HAL's tick timeout can expire
 * anywhere between N and N+1 ms, so 2 ms is the smallest value that can't
 * fire spuriously. A normal 15-byte burst takes well under 0.1 ms. */
#define ICM42688_SPI_TIMEOUT_MS  2

/* Longest single read: TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A). */
#define ICM42688_MAX_BURST_BYTES 14

/* Which bus and which chip-select pin the IMU is on. Passed into every call
 * so this module has no globals, same as the AS5600 driver. */
typedef struct
{
  SPI_HandleTypeDef *spi_handle;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
} ICM42688_Device;

/* Init result. Each failure gets its own code so the boot line says which
 * kind of problem to look for. */
typedef enum
{
  ICM42688_INIT_OK = 0,
  ICM42688_INIT_ERR_SPI,       /* HAL reported a transfer error or timeout */
  ICM42688_INIT_ERR_WHO_AM_I,  /* chip ID wrong: wiring, power, or not an ICM-42688-P */
  ICM42688_INIT_ERR_VERIFY     /* a config register did not read back as written */
} ICM42688_InitResult;

/* One sample as the chip reports it: signed 16-bit counts. */
typedef struct
{
  int16_t temperature;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} ICM42688_RawSample;

/* Register access. Each call is one CS-low ... CS-high frame. Reads write
 * their output only when they return HAL_OK, like the AS5600 reads. */
HAL_StatusTypeDef ICM42688_ReadRegisters(const ICM42688_Device *device, uint8_t first_register,
                                         uint8_t *values, uint16_t count);
HAL_StatusTypeDef ICM42688_WriteRegister(const ICM42688_Device *device, uint8_t register_address,
                                         uint8_t value);

/* Resets and configures the chip: gyro +/-500 dps, accel +/-4 g, both 1 kHz,
 * low-noise mode, I2C interface off. Blocks for about 40 ms (mostly the gyro
 * start-up time). who_am_i receives the ID byte read, or 0 if the read
 * never happened, so a failure can still be printed with what was seen. */
ICM42688_InitResult ICM42688_Init(const ICM42688_Device *device, uint8_t *who_am_i);

/* Short lowercase name for the boot line, e.g. "ok" or "err_who_am_i". */
const char *ICM42688_InitResultName(ICM42688_InitResult result);

/* Reads temperature, accel and gyro in one 15-byte SPI frame, so all seven
 * values come from the same sample. Returns HAL_ERROR (sample untouched) if
 * the 14 data bytes are all 0x00 or all 0xFF: a dead MISO line, which the SPI
 * hardware itself can't detect. */
HAL_StatusTypeDef ICM42688_ReadRawSample(const ICM42688_Device *device, ICM42688_RawSample *sample);

#endif /* ICM42688_H */
