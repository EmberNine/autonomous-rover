/**
  ******************************************************************************
  * @file    steering_control.h
  * @brief   Steering servo decisions, with no hardware access: the caller
  *          passes "now" in ms and gets back the pulse width to send, in us
  *          (0 = no pulses). firmware/Core/Src/servo.c writes it to TIM15.
  *
  * Commands: "s <percent>" lines (grammar in command_line.h), routed here by
  * command_dispatch.c. +100 = full left, -100 = full right, 0 = centre
  * (ROS REP 103: positive yaw is counter-clockwise seen from above, a left
  * turn). Which pulse direction is "left" depends on how the servo is
  * mounted: invert_direction swaps it.
  *
  * Behavior:
  *   - OFF at boot: no pulses until the first valid "s" command, so the servo
  *     never moves unless commanded (not on a reset either).
  *   - ACTIVE: the pulse follows the last command, mapped linearly between
  *     the centre and each endpoint (the two halves are scaled separately,
  *     so 0, +100 and -100 land exactly on CENTER, one endpoint and the
  *     other).
  *   - Link loss: no valid "s" for STEERING_COMMAND_TIMEOUT_MS while ACTIVE
  *     -> TIMED_OUT: centre, counted, until the next "s". This timeout is the
  *     steering's own; it is fed only by "s" and feeds nothing else (the
  *     motor's watchdog is fed only by "m").
  *   - FAULT (SteeringController_LockOut): no pulses until reset, whatever is
  *     commanded. main.c enters it when TIM15's period doesn't give 1 us
  *     ticks over 20 ms, or when the output guard below fires.
  *
  * Endpoints and centre are firmware safety limits: whatever the Pi sends,
  * the pulse never leaves [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US], so the
  * servo is never driven into its mechanical stops (a stalled servo heats
  * up and draws current). They are calibrated on the truck (below, and
  * README, "Steering calibration"). The
  * steering-angle <-> percent mapping belongs to the Pi (config/vehicle.cfg).
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_steering_control.c. No globals.
  ******************************************************************************
  */
#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Pulse limits in us, calibrated on the bench 2026-10-10 (stock AS-1 on its
 * linkage, wheels off the ground; README, "Steering calibration"):
 *   - CENTER: 1500 looked dead straight sighted from behind; +-10 us showed
 *     no movement (servo deadband), so it stays 1500.
 *   - MIN/MAX: a temporary build with 1150/1850 reached both locks cleanly,
 *     no buzzing or visible linkage flex; the limits are set 50 us inside
 *     that, as margin for slop and flex under load. s +-100 = full usable lock.
 * Changing them means recalibrating: tools/test_firmware_sources.py pins them. */
#define SERVO_PULSE_MIN_US       1200
#define SERVO_PULSE_CENTER_US    1500
#define SERVO_PULSE_MAX_US       1800
/* The calibrated limits must stay inside the standard RC servo range. */
#define SERVO_PULSE_HARD_MIN_US  1000
#define SERVO_PULSE_HARD_MAX_US  2000

#if !(SERVO_PULSE_HARD_MIN_US <= SERVO_PULSE_MIN_US && SERVO_PULSE_MIN_US < SERVO_PULSE_CENTER_US && \
      SERVO_PULSE_CENTER_US < SERVO_PULSE_MAX_US && SERVO_PULSE_MAX_US <= SERVO_PULSE_HARD_MAX_US)
#error "servo limits must satisfy HARD_MIN <= MIN < CENTER < MAX <= HARD_MAX"
#endif

/* TIM15: 1 MHz tick (32 MHz / (PSC 31 + 1)) and 20000 ticks per period
 * (ARR 19999 + 1) = 50 Hz, the standard RC servo frame. The pulse width in
 * us is then the compare value; with any other timer setup it isn't. */
#define SERVO_TIMER_TICK_HZ      1000000u
#define SERVO_PWM_PERIOD_TICKS   20000u

/* No valid "s" for this long while ACTIVE = centre. Same as the motor's
 * watchdog: the PC resends both commands every 100 ms. */
#define STEERING_COMMAND_TIMEOUT_MS  500

/* |percent| above this is never mapped (the parser already rejects it). */
#define STEERING_MAX_PERCENT     100

typedef enum
{
  STEERING_STATE_OFF = 0,        /* no pulses: nothing commanded since boot */
  STEERING_STATE_ACTIVE = 1,     /* following "s" commands */
  STEERING_STATE_TIMED_OUT = 2,  /* no "s" for STEERING_COMMAND_TIMEOUT_MS: centre */
  STEERING_STATE_FAULT = 3       /* no pulses until reset, whatever is commanded */
} SteeringState;

typedef struct
{
  SteeringState state;
  bool          invert_direction;   /* true: +percent gives a SHORTER pulse */
  int32_t       requested_percent;  /* last valid command as received */
  uint32_t      last_command_ms;    /* last valid command: the timeout's reference */
  uint32_t      pulse_us;           /* what the last Update returned (0 = no pulses) */

  /* Totals since boot (never reset) */
  uint32_t      valid_command_count;
  uint32_t      timeout_count;
} SteeringController;

/* Starts OFF (no pulses). */
void SteeringController_Init(SteeringController *controller, uint32_t now_ms, bool invert_direction);

/* A valid "s" command arrived. Starts or keeps ACTIVE (except in FAULT) and
 * resets the steering timeout. Counted in every state. */
void SteeringController_AcceptCommand(SteeringController *controller, int32_t percent, uint32_t now_ms);

/* Advances the timeout to now_ms and returns the pulse width to send, in us,
 * or 0 for no pulses. Call once per loop iteration. */
uint32_t SteeringController_Update(SteeringController *controller, uint32_t now_ms);

/* Enters FAULT from any state: no pulses from the next Update on. */
void SteeringController_LockOut(SteeringController *controller);

/* The pulse width for a command, always within [SERVO_PULSE_MIN_US,
 * SERVO_PULSE_MAX_US]; percent beyond +-STEERING_MAX_PERCENT is treated as
 * the limit. Exposed for the host test. */
uint32_t Steering_PulseForPercent(int32_t percent, bool invert_direction);

/* ---- Output guard (second layer, used by servo.c) ------------------------- */

/* What to write to the TIM15 compare register. */
typedef struct
{
  uint32_t compare_ticks;   /* 0 = no pulses */
  bool     timer_refused;   /* tick or period wrong: nothing is driven */
  bool     out_of_range;    /* a non-zero pulse outside [MIN, MAX] was asked for (clamped) */
} ServoPlan;

/* Does not trust its input. timer_tick_hz and period_ticks are what the
 * timer is really set to. Rules, in order:
 *   1. tick != SERVO_TIMER_TICK_HZ or period != SERVO_PWM_PERIOD_TICKS
 *                                  -> compare 0, timer_refused
 *   2. pulse 0                     -> compare 0 (no pulses)
 *   3. pulse outside [MIN, MAX]    -> clamped, out_of_range
 *   4. otherwise                   -> compare = pulse */
ServoPlan Steering_PlanOutput(uint32_t pulse_us, uint32_t timer_tick_hz, uint32_t period_ticks);

#endif /* STEERING_CONTROL_H */
