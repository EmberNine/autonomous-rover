/**
  ******************************************************************************
  * @file    link_protocol.c
  * @brief   The Pi link's message layouts, checks and packing (see the
  *          header).
  ******************************************************************************
  */
#include "link_protocol.h"

/* Compile-time checks of the numbers the header derives. */
typedef char link_cps_per_unit_is_50[(LINK_CPS_PER_UNIT == 50) ? 1 : -1];
typedef char link_min_speed_is_1000[(LINK_MIN_SPEED_CPS == 1000) ? 1 : -1];
typedef char link_status_fits[(LINK_STATUS_LENGTH <= LINK_MAX_PAYLOAD) ? 1 : -1];

/* ---- Little-endian writers, saturating to the wire type ------------------ */

static uint32_t SaturateUnsigned(uint32_t value, uint32_t maximum)
{
  return (value > maximum) ? maximum : value;
}

static int32_t SaturateSigned(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum) { return minimum; }
  if (value > maximum) { return maximum; }
  return value;
}

static size_t PutU8(uint8_t *out, size_t at, uint32_t value)
{
  out[at] = (uint8_t)SaturateUnsigned(value, 0xFFu);
  return at + 1u;
}

static size_t PutI8(uint8_t *out, size_t at, int32_t value)
{
  out[at] = (uint8_t)(int8_t)SaturateSigned(value, INT8_MIN, INT8_MAX);
  return at + 1u;
}

static size_t PutU16(uint8_t *out, size_t at, uint32_t value)
{
  uint32_t v = SaturateUnsigned(value, 0xFFFFu);
  out[at] = (uint8_t)(v & 0xFFu);
  out[at + 1u] = (uint8_t)(v >> 8);
  return at + 2u;
}

static size_t PutI16(uint8_t *out, size_t at, int32_t value)
{
  return PutU16(out, at, (uint16_t)(int16_t)SaturateSigned(value, INT16_MIN, INT16_MAX));
}

static size_t PutU32(uint8_t *out, size_t at, uint32_t value)
{
  out[at] = (uint8_t)(value & 0xFFu);
  out[at + 1u] = (uint8_t)((value >> 8) & 0xFFu);
  out[at + 2u] = (uint8_t)((value >> 16) & 0xFFu);
  out[at + 3u] = (uint8_t)(value >> 24);
  return at + 4u;
}

static size_t PutI32(uint8_t *out, size_t at, int32_t value)
{
  return PutU32(out, at, (uint32_t)value);
}

static int32_t GetI16(const uint8_t *in, size_t at)
{
  return (int32_t)(int16_t)(uint16_t)(in[at] | ((uint16_t)in[at + 1u] << 8));
}

/* ---- cps -> units ---------------------------------------------------------- */

int32_t LinkProtocol_CpsToUnits(int32_t cps)
{
  /* The minimum on the raw cps first, so it is exactly the HELLO_ACK's
   * min_cps. Then nearest, ties toward zero: (|cps| + 24) / 50 gives
   * 1025 -> 20, 1026 -> 21. On the magnitude, so both signs round alike. */
  int32_t magnitude = (cps < 0) ? -cps : cps;
  if (magnitude < LINK_MIN_SPEED_CPS)
  {
    return 0;
  }
  int32_t units = (magnitude + LINK_CPS_PER_UNIT / 2 - 1) / LINK_CPS_PER_UNIT;
  return (cps < 0) ? -units : units;
}

/* ---- What the Pi sends ------------------------------------------------------ */

LinkParseResult LinkProtocol_ParseHello(const LinkFrame *frame)
{
  if (frame->payload_length != LINK_HELLO_LENGTH)
  {
    return LINK_PARSE_BAD_LENGTH;
  }
  if (frame->payload[0] != LINK_PROTOCOL_VERSION)
  {
    return LINK_PARSE_BAD_VERSION;
  }
  return LINK_PARSE_OK;
}

LinkParseResult LinkProtocol_ParseDrive(const LinkFrame *frame, LinkDrive *drive)
{
  if (frame->payload_length != LINK_DRIVE_LENGTH)
  {
    return LINK_PARSE_BAD_LENGTH;
  }
  int32_t speed_cps = GetI16(frame->payload, 0u);
  int32_t steer_percent = (int32_t)(int8_t)frame->payload[2];
  uint32_t flags = frame->payload[3];
  if ((flags & LINK_DRIVE_RESERVED) != 0u)
  {
    return LINK_PARSE_BAD_FLAGS;
  }
  if (speed_cps > LINK_MAX_SPEED_CPS || speed_cps < -LINK_MAX_SPEED_CPS ||
      steer_percent > LINK_MAX_STEER_PERCENT || steer_percent < -LINK_MAX_STEER_PERCENT)
  {
    return LINK_PARSE_OUT_OF_RANGE;
  }
  bool arm = (flags & LINK_DRIVE_ARM) != 0u;
  if (arm && speed_cps != 0)
  {
    return LINK_PARSE_ARM_WITH_SPEED;
  }
  drive->speed_cps = speed_cps;
  drive->steer_percent = steer_percent;
  drive->steer_valid = (flags & LINK_DRIVE_STEER_VALID) != 0u;
  drive->arm = arm;
  return LINK_PARSE_OK;
}

LinkParseResult LinkProtocol_ParseEstop(const LinkFrame *frame)
{
  return (frame->payload_length == LINK_ESTOP_LENGTH) ? LINK_PARSE_OK : LINK_PARSE_BAD_LENGTH;
}

size_t LinkProtocol_PackDrive(int32_t speed_cps, int32_t steer_percent, uint32_t flags, uint8_t *out)
{
  size_t at = PutI16(out, 0u, speed_cps);
  at = PutI8(out, at, steer_percent);
  return PutU8(out, at, flags);
}

/* ---- What the Nucleo sends -------------------------------------------------- */

size_t LinkProtocol_PackHelloAck(const LinkHelloAckMessage *message, uint8_t *out)
{
  size_t at = PutU8(out, 0u, LINK_PROTOCOL_VERSION);
  at = PutU8(out, at, message->build_flags);
  at = PutU8(out, at, message->reset_flags);
  at = PutU8(out, at, message->imu_ok ? 1u : 0u);
  at = PutU8(out, at, message->imu_who);
  at = PutU16(out, at, (uint32_t)LINK_MIN_SPEED_CPS);
  at = PutU16(out, at, (uint32_t)LINK_MAX_SPEED_CPS);
  at = PutU16(out, at, (uint32_t)LINK_CPS_PER_UNIT);
  at = PutU8(out, at, message->state_period_ms);
  at = PutU8(out, at, message->status_period_ms);
  return at;
}

size_t LinkProtocol_PackState(const LinkStateMessage *message, uint8_t *out)
{
  size_t at = PutU32(out, 0u, message->t_ms);
  at = PutI32(out, at, message->count);
  at = PutI16(out, at, message->speed_cps);
  at = PutU8(out, at, message->enc_flags);
  at = PutU8(out, at, message->odometer_epoch & 0xFFu);  /* a wrapping counter: wraps, never saturates */
  for (size_t axis = 0u; axis < 3u; axis++)
  {
    at = PutI32(out, at, message->gyro_sum[axis]);
  }
  for (size_t axis = 0u; axis < 3u; axis++)
  {
    at = PutI32(out, at, message->accel_sum[axis]);
  }
  at = PutU8(out, at, message->imu_samples);
  at = PutU8(out, at, message->imu_flags);
  at = PutI16(out, at, message->duty_ticks);
  at = PutU8(out, at, SaturateUnsigned(message->motor_state, 0x0Fu) | (SaturateUnsigned(message->control_state, 0x0Fu) << 4));
  at = PutU8(out, at, message->drive_seq_echo & 0xFFu);
  return at;
}

size_t LinkProtocol_PackStatus(const LinkStatusMessage *message, uint8_t *out)
{
  size_t at = PutU32(out, 0u, message->t_ms);
  at = PutI16(out, at, message->setpoint_cps);
  at = PutI16(out, at, message->integrator_ticks);
  at = PutI8(out, at, message->steer_percent);
  at = PutU16(out, at, message->steer_pulse_us);
  at = PutU8(out, at, message->steer_state);
  at = PutU8(out, at, message->motor_state);
  at = PutU8(out, at, message->control_state);
  at = PutU8(out, at, message->fault_reason);
  at = PutU8(out, at, message->latch_reason);
  at = PutU8(out, at, message->battery_state);
  at = PutU8(out, at, message->imu_ok ? 1u : 0u);
  at = PutU8(out, at, message->mag_flags);
  at = PutU8(out, at, message->agc);
  at = PutU8(out, at, message->reset_flags);
  at = PutU16(out, at, message->vbat_mv);
  at = PutU16(out, at, message->vdda_mv);
  at = PutI16(out, at, message->imu_temperature);
  at = PutU16(out, at, message->magnitude_min);
  at = PutU16(out, at, message->magnitude_max);
  at = PutU16(out, at, message->loop_us_max);
  at = PutU16(out, at, message->i2c_us_max);
  at = PutU16(out, at, message->spi_us_max);
  for (size_t i = 0u; i < LINK_STATUS_COUNTERS; i++)
  {
    at = PutU32(out, at, message->counters[i]);
  }
  return at;
}
