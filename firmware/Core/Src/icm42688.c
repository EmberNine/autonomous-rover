/**
  ******************************************************************************
  * @file    icm42688.c
  * @brief   ICM-42688-P 6-axis IMU over 4-wire SPI.
  *
  * All register facts are from TDK DS-000347 rev 1.2 ("DS" below). User bank
  * 0 is the only bank this driver touches; the bank 1 interface registers
  * (INTF_CONFIG4/6) already reset to the values DS 12.3 lists for SPI.
  ******************************************************************************
  */
#include "icm42688.h"

/* ---- User bank 0 registers (DS 13.1 register map, 14.x descriptions) ---- */
#define ICM42688_REG_DEVICE_CONFIG   0x11  /* DS 14.1 */
#define ICM42688_REG_DRIVE_CONFIG    0x13  /* DS 14.2 */
#define ICM42688_REG_TEMP_DATA1      0x1D  /* DS 14.5; first of the 14 data bytes */
#define ICM42688_REG_INTF_CONFIG0    0x4C  /* DS 14.34 */
#define ICM42688_REG_PWR_MGMT0       0x4E  /* DS 14.36 */
#define ICM42688_REG_GYRO_CONFIG0    0x4F  /* DS 14.37 */
#define ICM42688_REG_ACCEL_CONFIG0   0x50  /* DS 14.38 */
#define ICM42688_REG_WHO_AM_I        0x75  /* DS 14.58 */
#define ICM42688_REG_BANK_SEL        0x76  /* DS 14.59; reachable from every bank */

/* First byte of every frame: bit 7 = R/W (1 = read), bits 6:0 = address (DS 9.6). */
#define ICM42688_SPI_READ_BIT        0x80

/* DEVICE_CONFIG (DS 14.1) */
#define ICM42688_SOFT_RESET_CONFIG   0x01  /* bit 0 */

/* DRIVE_CONFIG (DS 14.2): bits 2:0 SPI_SLEW_RATE. Reset value 101 (< 2 ns).
 * 001 = 12-36 ns. At 2 MHz a bit lasts 500 ns, so slower edges cost no timing
 * margin, and on ~21 cm of unshielded jumper they ring far less. */
#define ICM42688_SPI_SLEW_RATE_MASK  0x07
#define ICM42688_SPI_SLEW_12_36NS    0x01

/* INTF_CONFIG0 (DS 14.34) */
#define ICM42688_SENSOR_DATA_BIG_ENDIAN  0x10  /* bit 4; reset value is already 1 */
#define ICM42688_UI_SIFS_CFG_MASK        0x03  /* bits 1:0 */
#define ICM42688_UI_SIFS_DISABLE_I2C     0x03  /* 11 = disable I2C */

/* GYRO_CONFIG0 (DS 14.37): bits 7:5 GYRO_FS_SEL, bits 3:0 GYRO_ODR */
#define ICM42688_GYRO_CONFIG0_MASK   0xEF  /* leave reserved bit 4 alone */
#define ICM42688_GYRO_FS_500DPS      (0x2 << 5)  /* 010; 65.5 LSB/(deg/s), DS 3.1 */
#define ICM42688_GYRO_ODR_1KHZ       0x6         /* 0110 */

/* ACCEL_CONFIG0 (DS 14.38): bits 7:5 ACCEL_FS_SEL, bits 3:0 ACCEL_ODR */
#define ICM42688_ACCEL_CONFIG0_MASK  0xEF  /* leave reserved bit 4 alone */
#define ICM42688_ACCEL_FS_4G         (0x2 << 5)  /* 010; 8192 LSB/g, DS 3.2 */
#define ICM42688_ACCEL_ODR_1KHZ      0x6         /* 0110 (LN mode) */

/* PWR_MGMT0 (DS 14.36): bit 5 TEMP_DIS, bit 4 IDLE, bits 3:2 GYRO_MODE,
 * bits 1:0 ACCEL_MODE. Only the two mode fields are changed; TEMP_DIS stays
 * 0 (temperature on) because the temperature is reported in telemetry. */
#define ICM42688_SENSOR_MODES_MASK   0x0F
#define ICM42688_GYRO_MODE_LN        (0x3 << 2)
#define ICM42688_ACCEL_MODE_LN       0x3

/* ---- Timings ----------------------------------------------------------- */
/* DS 3.3: start-up time for register read/write, from power-up: 1 ms. */
#define ICM42688_POWER_UP_MS         1
/* DS 14.1: after setting SOFT_RESET_CONFIG, wait 1 ms before any register access. */
#define ICM42688_SOFT_RESET_MS       1
/* DS 3.1: gyroscope start-up time 30 ms (accel needs 10 ms, DS 3.2, so this
 * covers both). It also covers DS 14.36's "no register writes for 200 us"
 * after leaving OFF mode: nothing is written after PWR_MGMT0 anyway. */
#define ICM42688_GYRO_STARTUP_MS     30

/* One CS-low ... CS-high frame. */
static HAL_StatusTypeDef ICM42688_Transfer(const ICM42688_Device *device, const uint8_t *tx_bytes,
                                           uint8_t *rx_bytes, uint16_t length)
{
  /* HAL only switches the SPI peripheral on inside its first transfer, which
   * here would be *after* CS goes low. Until then SCK may sit low, and in
   * mode 3 its jump to the idle-high level would be an extra clock edge
   * inside the frame. Enabling it first (CS still high) makes SCK settle at
   * idle while the chip isn't listening. Also covers a HAL error path that
   * left the peripheral disabled. */
  if ((device->spi_handle->Instance->CR1 & SPI_CR1_SPE) == 0u)
  {
    __HAL_SPI_ENABLE(device->spi_handle);
  }

  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(device->spi_handle, (uint8_t *)tx_bytes,
                                                     rx_bytes, length, ICM42688_SPI_TIMEOUT_MS);
  /* Raise CS even after a failure, so the chip drops the half-finished frame
   * and the next one starts cleanly. */
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
  return result;
}

/* Burst reads work because the chip auto-increments the address (DS 9.6:
 * "Supports Single or Burst Reads"). */
HAL_StatusTypeDef ICM42688_ReadRegisters(const ICM42688_Device *device, uint8_t first_register,
                                         uint8_t *values, uint16_t count)
{
  if (count == 0u || count > ICM42688_MAX_BURST_BYTES)
  {
    return HAL_ERROR;
  }

  /* Byte 0 out: address. Bytes 1..count out: don't-care (zeros), while the
   * chip shifts the register contents back on MISO. */
  uint8_t tx_bytes[1 + ICM42688_MAX_BURST_BYTES] = {0};
  uint8_t rx_bytes[1 + ICM42688_MAX_BURST_BYTES];
  tx_bytes[0] = (uint8_t)(ICM42688_SPI_READ_BIT | (first_register & 0x7Fu));

  HAL_StatusTypeDef result = ICM42688_Transfer(device, tx_bytes, rx_bytes, (uint16_t)(count + 1u));
  if (result == HAL_OK)
  {
    for (uint16_t i = 0; i < count; i++)
    {
      values[i] = rx_bytes[i + 1u];  /* rx_bytes[0] arrived while the address went out: junk */
    }
  }
  return result;
}

/* DS 9.6 item 7: SPI writes are single-register only, so each write is its own frame. */
HAL_StatusTypeDef ICM42688_WriteRegister(const ICM42688_Device *device, uint8_t register_address,
                                         uint8_t value)
{
  uint8_t tx_bytes[2] = { (uint8_t)(register_address & 0x7Fu), value };
  uint8_t rx_bytes[2];
  return ICM42688_Transfer(device, tx_bytes, rx_bytes, 2);
}

/* Read-modify-write of only the bits in field_mask, then read back to prove
 * the write landed. Read-back is the cheapest way to catch a marginal wire
 * at boot: a corrupted write shows up here instead of as odd data later.
 * Leaves the other bits (including reserved ones) exactly as the chip had them. */
static ICM42688_InitResult ICM42688_UpdateRegister(const ICM42688_Device *device, uint8_t register_address,
                                                   uint8_t field_mask, uint8_t field_value)
{
  uint8_t current;
  if (ICM42688_ReadRegisters(device, register_address, &current, 1) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }

  uint8_t updated = (uint8_t)((current & (uint8_t)~field_mask) | (field_value & field_mask));
  if (ICM42688_WriteRegister(device, register_address, updated) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }

  uint8_t read_back;
  if (ICM42688_ReadRegisters(device, register_address, &read_back, 1) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }
  if ((read_back & field_mask) != (field_value & field_mask))
  {
    return ICM42688_INIT_ERR_VERIFY;
  }
  return ICM42688_INIT_OK;
}

ICM42688_InitResult ICM42688_Init(const ICM42688_Device *device, uint8_t *who_am_i)
{
  ICM42688_InitResult result;
  *who_am_i = 0;

  /* 1. CS high. The breakout has no pull-up on CS, so it floats until the
   *    MCU drives it; MX_GPIO_Init already set it high, this makes the
   *    driver correct on its own. */
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

  /* 2. The IMU and the MCU share the 3V3 rail, so time since MCU boot is
   *    time since IMU power-up. HAL_Delay rounds up, so this is >= 1 ms
   *    from here, which is more than enough. */
  HAL_Delay(ICM42688_POWER_UP_MS);

  /* 3. Bank 0, where every register below lives. A warm MCU reset (e.g.
   *    reflashing) doesn't reset the IMU, which may have been left in
   *    another bank. */
  if (ICM42688_WriteRegister(device, ICM42688_REG_BANK_SEL, 0x00) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }

  /* 4. Soft reset, so every register starts from its datasheet reset value
   *    no matter what a previous firmware run left behind. Read-modify-write
   *    keeps SPI_MODE (bit 4) at its current value: 0 = modes 0 and 3, which
   *    is what SPI1 uses. No read-back: the register clears itself. */
  uint8_t device_config;
  if (ICM42688_ReadRegisters(device, ICM42688_REG_DEVICE_CONFIG, &device_config, 1) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }
  if (ICM42688_WriteRegister(device, ICM42688_REG_DEVICE_CONFIG,
                             (uint8_t)(device_config | ICM42688_SOFT_RESET_CONFIG)) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }
  HAL_Delay(ICM42688_SOFT_RESET_MS);

  /* 5. Identity check. With MISO unconnected or the chip unpowered this
   *    reads 0x00 or 0xFF: SPI itself never reports an error for that. */
  uint8_t id;
  if (ICM42688_ReadRegisters(device, ICM42688_REG_WHO_AM_I, &id, 1) != HAL_OK)
  {
    return ICM42688_INIT_ERR_SPI;
  }
  *who_am_i = id;
  if (id != ICM42688_WHO_AM_I_VALUE)
  {
    return ICM42688_INIT_ERR_WHO_AM_I;
  }

  /* 6. Turn off the I2C side (UI_SIFS_CFG = 11), so SPI traffic can never be
   *    mistaken for an I2C start condition. Also set SENSOR_DATA_ENDIAN to
   *    big-endian explicitly: it's the reset value, but ReadRawSample's
   *    decoding depends on it, so don't rely on the default. */
  result = ICM42688_UpdateRegister(device, ICM42688_REG_INTF_CONFIG0,
                                   ICM42688_UI_SIFS_CFG_MASK | ICM42688_SENSOR_DATA_BIG_ENDIAN,
                                   ICM42688_UI_SIFS_DISABLE_I2C | ICM42688_SENSOR_DATA_BIG_ENDIAN);
  if (result != ICM42688_INIT_OK) { return result; }

  /* 7. Slower MISO edges for the long jumpers (see ICM42688_SPI_SLEW_12_36NS). */
  result = ICM42688_UpdateRegister(device, ICM42688_REG_DRIVE_CONFIG,
                                   ICM42688_SPI_SLEW_RATE_MASK, ICM42688_SPI_SLEW_12_36NS);
  if (result != ICM42688_INIT_OK) { return result; }

  /* 8. Full scale and ODR, written while the sensors are still off. */
  result = ICM42688_UpdateRegister(device, ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_CONFIG0_MASK,
                                   ICM42688_GYRO_FS_500DPS | ICM42688_GYRO_ODR_1KHZ);
  if (result != ICM42688_INIT_OK) { return result; }

  result = ICM42688_UpdateRegister(device, ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG0_MASK,
                                   ICM42688_ACCEL_FS_4G | ICM42688_ACCEL_ODR_1KHZ);
  if (result != ICM42688_INIT_OK) { return result; }

  /* 9. Gyro and accel on, low-noise mode. Must be the last write: DS 14.36
   *    forbids register writes for 200 us after leaving OFF mode. (The read-
   *    back inside UpdateRegister is a read, which is allowed.) */
  result = ICM42688_UpdateRegister(device, ICM42688_REG_PWR_MGMT0, ICM42688_SENSOR_MODES_MASK,
                                   ICM42688_GYRO_MODE_LN | ICM42688_ACCEL_MODE_LN);
  if (result != ICM42688_INIT_OK) { return result; }

  /* 10. Gyro data isn't valid until the drive is up (DS 3.1). */
  HAL_Delay(ICM42688_GYRO_STARTUP_MS);

  return ICM42688_INIT_OK;
}

const char *ICM42688_InitResultName(ICM42688_InitResult result)
{
  switch (result)
  {
    case ICM42688_INIT_OK:           return "ok";
    case ICM42688_INIT_ERR_SPI:      return "err_spi";
    case ICM42688_INIT_ERR_WHO_AM_I: return "err_who_am_i";
    case ICM42688_INIT_ERR_VERIFY:   return "err_verify";
    default:                         return "err_unknown";
  }
}

/* Big-endian (DS 14.34 SENSOR_DATA_ENDIAN = 1): high byte first. The cast
 * through uint16_t gives the two's-complement value without relying on
 * signed shifts. */
static int16_t ICM42688_BigEndianToInt16(const uint8_t *bytes)
{
  return (int16_t)(uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

/* True if every byte is 0x00 or every byte is 0xFF: what MISO delivers when
 * nothing is driving it (stuck low, or pulled/floating high). */
static bool ICM42688_IsStuckLine(const uint8_t *bytes, uint16_t count)
{
  bool all_zero = true;
  bool all_ones = true;
  for (uint16_t i = 0; i < count; i++)
  {
    if (bytes[i] != 0x00u) { all_zero = false; }
    if (bytes[i] != 0xFFu) { all_ones = false; }
  }
  return all_zero || all_ones;
}

/* 0x1D..0x2A in one frame (DS 13.1): TEMP, ACCEL X/Y/Z, GYRO X/Y/Z, two
 * bytes each. One frame rather than seven reads so the values can't come
 * from two different samples if the chip updates mid-read. */
HAL_StatusTypeDef ICM42688_ReadRawSample(const ICM42688_Device *device, ICM42688_RawSample *sample)
{
  uint8_t data[ICM42688_MAX_BURST_BYTES];
  HAL_StatusTypeDef result = ICM42688_ReadRegisters(device, ICM42688_REG_TEMP_DATA1, data,
                                                    ICM42688_MAX_BURST_BYTES);
  /* A master never notices a missing slave: with MISO cut, or the chip
   * unpowered, HAL still returns HAL_OK and the bytes are a constant line
   * level. A real sample can't be all 0x00 (accel z alone reads ~+8192 at
   * rest) or all 0xFF (seven readings of exactly -1), so treat either as a
   * failed read. The caller then keeps its previous sample. */
  if (result == HAL_OK && ICM42688_IsStuckLine(data, ICM42688_MAX_BURST_BYTES))
  {
    result = HAL_ERROR;
  }
  if (result == HAL_OK)
  {
    sample->temperature = ICM42688_BigEndianToInt16(&data[0]);
    sample->accel_x     = ICM42688_BigEndianToInt16(&data[2]);
    sample->accel_y     = ICM42688_BigEndianToInt16(&data[4]);
    sample->accel_z     = ICM42688_BigEndianToInt16(&data[6]);
    sample->gyro_x      = ICM42688_BigEndianToInt16(&data[8]);
    sample->gyro_y      = ICM42688_BigEndianToInt16(&data[10]);
    sample->gyro_z      = ICM42688_BigEndianToInt16(&data[12]);
  }
  return result;
}
