/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 magnetic angle sensor: register reads + angle unwrapping.
  ******************************************************************************
  */
#include "as5600.h"

#define AS5600_REG_STATUS     0x0B
#define AS5600_REG_RAW_ANGLE  0x0C  /* 2 bytes: 0x0C high, 0x0D low */
#define AS5600_REG_AGC        0x1A
#define AS5600_REG_MAGNITUDE  0x1B  /* 2 bytes: 0x1B high, 0x1C low */

#define AS5600_STATUS_MD_BIT  5
#define AS5600_STATUS_ML_BIT  4
#define AS5600_STATUS_MH_BIT  3

#define AS5600_12BIT_MASK     0x0FFF
#define AS5600_HALF_REV       (AS5600_COUNTS_PER_REV / 2)

/* Both 12-bit registers arrive high byte first, which is the order HAL fills the buffer in. */
static HAL_StatusTypeDef AS5600_Read12BitRegister(I2C_HandleTypeDef *i2c_handle,
                                                  uint16_t register_address, uint16_t *value)
{
  uint8_t bytes[2];
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(i2c_handle, AS5600_I2C_ADDRESS, register_address,
                                              I2C_MEMADD_SIZE_8BIT, bytes, 2, AS5600_I2C_TIMEOUT_MS);
  if (result == HAL_OK)
  {
    *value = (uint16_t)(((bytes[0] << 8) | bytes[1]) & AS5600_12BIT_MASK);
  }
  return result;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *i2c_handle, uint16_t *raw_angle)
{
  return AS5600_Read12BitRegister(i2c_handle, AS5600_REG_RAW_ANGLE, raw_angle);
}

HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *i2c_handle, uint16_t *magnitude)
{
  return AS5600_Read12BitRegister(i2c_handle, AS5600_REG_MAGNITUDE, magnitude);
}

HAL_StatusTypeDef AS5600_ReadAgc(I2C_HandleTypeDef *i2c_handle, uint8_t *agc)
{
  uint8_t agc_byte;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(i2c_handle, AS5600_I2C_ADDRESS, AS5600_REG_AGC,
                                              I2C_MEMADD_SIZE_8BIT, &agc_byte, 1,
                                              AS5600_I2C_TIMEOUT_MS);
  if (result == HAL_OK)
  {
    *agc = agc_byte;
  }
  return result;
}

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *i2c_handle, AS5600_Status *status)
{
  uint8_t status_byte;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(i2c_handle, AS5600_I2C_ADDRESS, AS5600_REG_STATUS,
                                              I2C_MEMADD_SIZE_8BIT, &status_byte, 1,
                                              AS5600_I2C_TIMEOUT_MS);
  if (result == HAL_OK)
  {
    status->magnet_detected   = (status_byte >> AS5600_STATUS_MD_BIT) & 1u;
    status->magnet_too_weak   = (status_byte >> AS5600_STATUS_ML_BIT) & 1u;
    status->magnet_too_strong = (status_byte >> AS5600_STATUS_MH_BIT) & 1u;
  }
  return result;
}

void AS5600_UnwrapperInit(AS5600_Unwrapper *unwrapper)
{
  unwrapper->previous_raw_angle = 0;
  unwrapper->has_first_sample = false;
  unwrapper->continuous_count = 0;
}

/*
 * Assumption: the wheel turns less than half a revolution (2048 counts)
 * between two good samples. Then the shortest way around the circle is the
 * true motion: a jump of e.g. 4090 -> 5 is really +11, not -4085.
 * At 1 kHz this holds up to 500 rev/s (~85 m/s with this wheel), far beyond
 * the truck. A few missed samples in a row still leave a huge margin.
 *
 * Why int32 for the count: int32 holds +/-2,147,483,647 counts, which is
 * 524,288 revolutions, or ~88.9 km at 0.1696 m/rev, in one direction without
 * reversing. That's hours of flat-out driving, well past a battery. int32 also
 * has two practical advantages over int64:
 *   - the Cortex-M4 reads/writes it in one instruction, so it can be shared
 *     safely with an interrupt later (an int64 could be read half-updated);
 *   - the nano printf we link with does not support %lld.
 * The add below is done in unsigned math so that an overflow wraps like a car
 * odometer instead of being undefined behavior in C. The Pi should work with
 * differences between readings, which stay correct across a wrap.
 */
void AS5600_UnwrapperUpdate(AS5600_Unwrapper *unwrapper, uint16_t raw_angle)
{
  if (!unwrapper->has_first_sample)
  {
    unwrapper->previous_raw_angle = raw_angle;
    unwrapper->has_first_sample = true;
    return;
  }

  int32_t delta = (int32_t)raw_angle - (int32_t)unwrapper->previous_raw_angle;
  if (delta > AS5600_HALF_REV)
  {
    delta -= AS5600_COUNTS_PER_REV;
  }
  else if (delta < -AS5600_HALF_REV)
  {
    delta += AS5600_COUNTS_PER_REV;
  }

  unwrapper->continuous_count =
      (int32_t)((uint32_t)unwrapper->continuous_count + (uint32_t)delta);
  unwrapper->previous_raw_angle = raw_angle;
}
