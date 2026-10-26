/**
  ******************************************************************************
  * @file    encoder_fault.c
  * @brief   TEST BUILD ONLY: encoder fault injection (see the header).
  ******************************************************************************
  */
#include "encoder_fault.h"

void EncoderFault_Init(EncoderFault *fault)
{
  *fault = (EncoderFault){0};
  fault->kind = ENCODER_FAULT_NONE;
}

bool EncoderFault_Command(EncoderFault *fault, int32_t value, uint32_t now_ms)
{
  uint32_t duration_ms;
  switch (value)
  {
    case ENCODER_FAULT_SHORT_OUTAGE: duration_ms = ENCODER_FAULT_SHORT_OUTAGE_MS; break;
    case ENCODER_FAULT_LONG_OUTAGE:  duration_ms = ENCODER_FAULT_LONG_OUTAGE_MS;  break;
    case ENCODER_FAULT_FROZEN_ANGLE: duration_ms = ENCODER_FAULT_FROZEN_ANGLE_MS; break;
    default: return false;
  }
  if (fault->kind != ENCODER_FAULT_NONE)
  {
    return false;  /* one at a time */
  }
  fault->kind = (EncoderFaultKind)value;
  fault->start_ms = now_ms;
  fault->duration_ms = duration_ms;
  fault->frozen_angle_set = false;
  fault->started_count++;
  return true;
}

void EncoderFault_Apply(EncoderFault *fault, bool *read_ok, uint16_t *raw_angle, uint32_t now_ms)
{
  if (fault->kind == ENCODER_FAULT_NONE)
  {
    return;
  }
  if (now_ms - fault->start_ms >= fault->duration_ms)
  {
    fault->kind = ENCODER_FAULT_NONE;  /* over: this read is the real one */
    return;
  }
  if (fault->kind != ENCODER_FAULT_FROZEN_ANGLE)
  {
    *read_ok = false;
    return;
  }
  if (!*read_ok)
  {
    return;  /* a real failure stays a failure */
  }
  if (!fault->frozen_angle_set)
  {
    fault->frozen_angle = *raw_angle;
    fault->frozen_angle_set = true;
  }
  *raw_angle = fault->frozen_angle;
}
