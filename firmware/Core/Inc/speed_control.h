/**
  ******************************************************************************
  * @file    speed_control.h
  * @brief   Closed-loop wheel speed ("v <units>"): a PI controller that
  *          writes the motor's target THROUGH motor_control
  *          (MotorController_SetClosedLoopTarget), so the cap, the ramp, the
  *          reverse dwell, the watchdog, the latch, FAULT and the output-stage
  *          guards all still apply. No hardware access.
  *
  * Units: speed in units of 1 count per 20 ms = 50 counts/s (the speed
  * estimator's), duty in timer ticks (16 per %, cap 480).
  *
  * The Pi only ever uses "v"; "m" (open loop) is a bench/diagnostic command.
  * Last command wins: any "m" switches the controller off (open loop) and
  * clears its integrator; "v" switches it on. Both feed the motor watchdog
  * (motor_control.c); the controller itself never does.
  *
  * Gains (lambda tuning for a first-order plant with dead time,
  * K e^(-theta s) / (tau s + 1)):  Kp = tau / (K (lambda + theta)),
  * Ki = Kp / tau = 1 / (K (lambda + theta)).
  *   K      = 11.7 counts/s per tick on the lift = 0.234 units per tick
  *   tau    = 30 ms (design point: the top of the 0-30 ms range the step
  *            captures allow; a smaller real tau only adds margin)
  *   theta  = 50 ms (the loop's own ~18 ms: estimator window 10, controller
  *            period 5, sensor + read 3; plus margin for tau)
  *   lambda = 150 ms (closed-loop time constant)
  *   Ki = 1 / (0.234 x 200 ms) = 0.0214 ticks per unit per ms
  *      = 0.214 per 10 ms  -> Q8 55
  *   Kp = 30 x 0.0214      = 0.641 ticks per unit -> Q8 164
  * Crossover ~ Ki K = 5 rad/s (0.8 Hz): even 48 ms of delay costs only ~14
  * degrees of phase there. Simulated against tau 0-30 ms in the host test.
  *
  * Rules (every one host-tested, tests/test_speed_control.c):
  *   - PI every SPEED_CONTROL_PERIOD_MS (due time, not "%"), integer Q8, no D.
  *   - Output and integrator clamped to the setpoint's sign: [0, +cap] for
  *     v > 0, [-cap, 0] for v < 0. It never asks for the other direction to
  *     slow down: a lower duty already brakes (slow decay).
  *   - Conditional integration: not while the output is at its limit in the
  *     error's direction, not while motor_control is still ramping toward
  *     the target, not unless the motor is RUNNING, never on an invalid
  *     estimate.
  *   - Integrator reset: v 0, a setpoint sign change, any "m", LATCHED,
  *     FAULT. m -> v starts it at the duty applied now (bumpless).
  *   - Start from rest: the integrator is preloaded to the breakaway duty
  *     (SPEED_PRELOAD_TICKS), so the wheel doesn't wait ~0.4 s for the
  *     integrator to climb through the static friction (see below).
  *   - |v| below SPEED_MIN_SETPOINT_UNITS is 0. v 0 coasts (target 0: the
  *     ramp down brakes, then the driver is off), like m 0.
  *   - Sign change (and any start): zero-speed gate, target 0 until
  *     |estimate| <= SPEED_GATE_UNITS (or it already turns the new way).
  *   - Invalid estimate: target held and integrator frozen for up to
  *     SPEED_INVALID_HOLD_MS; still invalid -> target 0, MotorController_Latch
  *     (needs m 0 / v 0), enc_bad + 1.
  *
  * Preload, for and against. For: from rest the wheel doesn't move until
  * ~156-188 ticks (lift breakaway); without a preload the integrator climbs
  * there at ~4 ticks per 10 ms (v 20), ~0.4 s of nothing, then overshoots
  * as the static friction lets go. Against: if the real breakaway is lower
  * (on the floor it is probably higher), the preload can overshoot a low
  * setpoint briefly; bounded, it is only 10 % duty and the PI pulls it back
  * within ~0.2 s. Only on a start from rest (applied duty 0), never while
  * moving.
  *
  * Pure C; uses motor_control (its API only) and the speed estimator's
  * result. Host-tested by firmware/tests/test_speed_control.c. No globals.
  ******************************************************************************
  */
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control.h"
#include "speed_estimator.h"

#define SPEED_CONTROL_PERIOD_MS    10u
#define SPEED_KP_Q8                164   /* 0.641 ticks per unit */
#define SPEED_KI_Q8                55    /* 0.214 ticks per unit per 10 ms */
#define SPEED_MIN_SETPOINT_UNITS   20    /* below: 0 (lift breakaway ~21-23 units) */
#define SPEED_GATE_UNITS           2     /* zero-speed gate before a (re)start */
#define SPEED_INVALID_HOLD_MS      50u   /* hold on an invalid estimate, then latch */
#define SPEED_PRELOAD_TICKS        160   /* integrator start from rest: breakaway duty (lift 156-188) */

#define SPEED_Q8_ONE               256

typedef enum
{
  SPEED_MODE_OPEN_LOOP = 0,   /* "m": the controller writes nothing */
  SPEED_MODE_CLOSED_LOOP      /* "v" */
} SpeedMode;

typedef struct
{
  SpeedMode mode;
  int32_t   setpoint_units;   /* last "v" as received */
  int32_t   effective_units;  /* after the minimum-setpoint rule: 0 or |v| >= SPEED_MIN_SETPOINT_UNITS */
  int32_t   direction;        /* +1 / -1 while driving, 0 while stopped, starting or gated */
  int32_t   integrator_q8;    /* ticks x 256, same sign as direction */
  int32_t   output_ticks;     /* last target written */
  int32_t   error_units;      /* last error seen by the PI */
  int32_t   preload_ticks;    /* SPEED_PRELOAD_TICKS (a field so the test can compare with 0) */
  bool      gate_waiting;     /* a start is waiting for the wheel to (nearly) stop */
  uint32_t  next_due_ms;
  bool      due_set;
  uint32_t  enc_bad_count;    /* latches because the estimate stayed invalid */
} SpeedController;

void SpeedControl_Init(SpeedController *speed);

/* A valid "m": open loop from now on; integrator cleared. */
void SpeedControl_SelectOpenLoop(SpeedController *speed);

/* A valid "v": closed loop with this setpoint. applied_ticks: the duty
 * motor_control applies right now (bumpless m -> v, and "from rest"). A
 * keepalive with the same setpoint changes nothing. */
void SpeedControl_SetSetpoint(SpeedController *speed, int32_t setpoint_units, int32_t applied_ticks);

/* Every ms, before MotorController_Update, with the estimator's latest
 * result. Writes the motor's target in closed loop; does nothing in open
 * loop. */
void SpeedControl_Update(SpeedController *speed, MotorController *motor, const SpeedEstimator *estimate,
                         uint32_t now_ms);

/* ---- Telemetry (v5: "ctl" and "vi") ------------------------------------------ */

/* What the controller is doing, as the "ctl" code. The values are part of
 * the telemetry format (tools/motor_console.py decodes them): append only. */
typedef enum
{
  SPEED_CTL_OPEN_LOOP = 0,   /* "m" (or nothing yet): the controller writes nothing */
  SPEED_CTL_RUNNING   = 1,   /* "v" with an effective setpoint (mstate says whether the motor follows) */
  SPEED_CTL_STOPPED   = 2,   /* "v 0", or a setpoint below SPEED_MIN_SETPOINT_UNITS: coasting */
  SPEED_CTL_GATE      = 3    /* "v" start waiting for the wheel to (nearly) stop */
} SpeedControlState;

SpeedControlState SpeedControl_State(const SpeedController *speed);

/* The integrator in whole ticks (rounded half away from zero), as "vi". */
int32_t SpeedControl_IntegratorTicks(const SpeedController *speed);

#endif /* SPEED_CONTROL_H */
