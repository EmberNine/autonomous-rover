/**
  ******************************************************************************
  * @file    loop_schedule.h
  * @brief   When the main loop's periodic tasks (telemetry report, magnet
  *          health reads, heartbeat LED) are due, and which LED pattern the
  *          heartbeat shows. No hardware access: the caller passes the
  *          iteration's scheduled time in ms.
  *
  * Why not "t % period == phase": that is only true if the loop actually
  * runs at the millisecond where it holds. Normally it does (one iteration
  * per ms). But when an iteration takes much longer, the loop restarts its
  * schedule from "now" (resync), and the iteration times jump. With a
  * stuck encoder I2C bus, each HAL read waits its 25 ms busy timeout
  * (I2C_TIMEOUT_BUSY), so every iteration takes ~52 ms and the times step by
  * exactly 52 ms (the HAL timeouts end on a tick). gcd(52, 100) = 4, so if
  * the first jump lands on a time that isn't a multiple of 4, "t % 100 == 0"
  * and "t % 500 == 0" are never true again: no telemetry and no heartbeat,
  * while the loop runs and refreshes the IWDG, so the board looks dead but
  * the motor can still be armed. This is the likely (unconfirmed) cause of
  * the silent boot on 2026-10-14 with the IMU and encoder unplugged.
  *
  * Due-time scheduling instead: a task is due once the scheduled time has
  * reached its next mark (the marks are still the times with
  * t % period == phase), and then the next mark is the first one after now.
  *   - Normal 1 ms steps: due exactly at the marks, as before.
  *   - Long steps (52 ms): due at the first iteration at or after each mark,
  *     so still about once per period.
  *   - A big jump (resync): due once, never a burst of missed ones.
  *   - Times compared as a signed difference, so the 49.7-day wrap of the
  *     ms tick is handled. Across the wrap one interval is 96 ms instead of
  *     100 (2^32 is not a multiple of 100), exactly as with the old "%".
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_loop_schedule.c. No globals.
  ******************************************************************************
  */
#ifndef LOOP_SCHEDULE_H
#define LOOP_SCHEDULE_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Periodic tasks --------------------------------------------------------- */

typedef struct
{
  uint32_t period_ms;    /* > 0 */
  uint32_t phase_ms;     /* 0 .. period_ms - 1: marks are the times with t % period_ms == phase_ms */
  uint32_t next_due_ms;  /* the next mark */
} PeriodicSchedule;

/* The first mark at or after now_ms is the first due time. */
void PeriodicSchedule_Init(PeriodicSchedule *schedule, uint32_t period_ms, uint32_t phase_ms, uint32_t now_ms);

/* True if the task is due at now_ms (call exactly once per iteration, with
 * the iteration's scheduled time). When it returns true, the next due time
 * becomes the first mark after now_ms. */
bool PeriodicSchedule_IsDue(PeriodicSchedule *schedule, uint32_t now_ms);

/* ---- Heartbeat LED ------------------------------------------------------------ */

/* The heartbeat is driven by its own 100 ms schedule; each pattern below is
 * a sequence of 100 ms ticks. */
#define HEARTBEAT_TICK_MS  100u

typedef enum
{
  HEARTBEAT_NORMAL = 0,     /* loop alive: 500 ms on, 500 ms off */
  HEARTBEAT_IMU_MISSING,    /* IMU init failed (imu_ok=0): double blink, 100 on, 100 off, 100 on, 700 off */
  HEARTBEAT_MOTOR_FAULT     /* motor FAULT (mstate=3, needs a reset): 100 ms on, 100 ms off */
} HeartbeatMode;

/* Which pattern to show. FAULT wins over IMU missing: it is the state that
 * needs action (a reset) before the motor can run again, while a missing
 * IMU changes nothing on the MCU (it keeps running without it). */
HeartbeatMode Heartbeat_Mode(bool motor_fault, bool imu_ok);

/* LED on (true) or off for this tick of the pattern; tick counts the
 * heartbeat schedule's due times since boot. */
bool Heartbeat_LedOn(HeartbeatMode mode, uint32_t tick);

#endif /* LOOP_SCHEDULE_H */
