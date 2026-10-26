/*
 * Host-test stand-in for the STM32 HAL header, with only what
 * Core/Src/as5600.c needs to compile on the PC, so host tests can use the
 * real AS5600_Unwrapper. Put this directory first on the include path
 * (-Itests/stubs -ICore/Inc). The test that links as5600.c provides
 * HAL_I2C_Mem_Read (it is never called by the unwrapper).
 */
#ifndef STM32L4XX_HAL_H_HOST_STUB
#define STM32L4XX_HAL_H_HOST_STUB

#include <stdint.h>

typedef enum
{
  HAL_OK = 0x00,
  HAL_ERROR = 0x01,
  HAL_BUSY = 0x02,
  HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct
{
  int unused;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT  0x00000001U

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif /* STM32L4XX_HAL_H_HOST_STUB */
