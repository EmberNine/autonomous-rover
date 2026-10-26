/**
  ******************************************************************************
  * @file    loop_schedule.c
  * @brief   Due-time scheduling of the main loop's periodic tasks and the
  *          heartbeat LED patterns. See loop_schedule.h for why.
  ******************************************************************************
  */
#include "loop_schedule.h"

/* The first mark (t % period == phase) at or after time_ms. Unsigned math:
 * near the top of the range the mark may wrap past 0, which the signed
 * comparison in PeriodicSchedule_IsDue handles. */
static uint32_t FirstMarkAtOrAfter(const PeriodicSchedule *schedule, uint32_t time_ms)
{
  uint32_t mark = time_ms - (time_ms % schedule->period_ms) + schedule->phase_ms;
  if ((int32_t)(mark - time_ms) < 0)
  {
    mark += schedule->period_ms;
  }
  return mark;
}

void PeriodicSchedule_Init(PeriodicSchedule *schedule, uint32_t period_ms, uint32_t phase_ms, uint32_t now_ms)
{
  schedule->period_ms = period_ms;
  schedule->phase_ms = phase_ms;
  schedule->next_due_ms = FirstMarkAtOrAfter(schedule, now_ms);
}

bool PeriodicSchedule_IsDue(PeriodicSchedule *schedule, uint32_t now_ms)
{
  if ((int32_t)(now_ms - schedule->next_due_ms) < 0)
  {
    return false;
  }
  /* Computed from now, not "next_due += period": after a long stall that
   * skips the marks in between instead of firing once per missed mark. */
  schedule->next_due_ms = FirstMarkAtOrAfter(schedule, now_ms + 1u);
  return true;
}

HeartbeatMode Heartbeat_Mode(bool motor_fault, bool imu_ok)
{
  if (motor_fault)
  {
    return HEARTBEAT_MOTOR_FAULT;
  }
  return imu_ok ? HEARTBEAT_NORMAL : HEARTBEAT_IMU_MISSING;
}

bool Heartbeat_LedOn(HeartbeatMode mode, uint32_t tick)
{
  switch (mode)
  {
    case HEARTBEAT_MOTOR_FAULT:
      return (tick % 2u) == 0u;              /* 100 on, 100 off */
    case HEARTBEAT_IMU_MISSING:
    {
      uint32_t phase = tick % 10u;           /* 1 s cycle */
      return phase == 0u || phase == 2u;     /* on, off, on, then 700 ms off */
    }
    case HEARTBEAT_NORMAL:
    default:
      return (tick % 10u) < 5u;              /* 500 on, 500 off */
  }
}
