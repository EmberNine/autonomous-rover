/**
  ******************************************************************************
  * @file    stall_detect.h
  * @brief   Stalled-motor detection: duty on, wheel not turning. Acts on the
  *          motor through motor_control's API only. No hardware access.
  *
  * Rule, the same in open loop ("m") and closed loop ("v"):
  *   motor RUNNING, |applied duty| >= STALL_MIN_DUTY_TICKS, the speed
  *   estimate VALID and |estimate| < STALL_MAX_SPEED_UNITS, without a break
  *   for STALL_HOLD_MS -> a stall:
  *     - stop at once (driver off) and latch until "m 0" / "v 0"
  *       (MotorController_Latch: mstate=2, MOTOR_LATCH_STALL_LOCKED),
  *       stall_count + 1 and locked_stall_count + 1;
  *     - the STALL_REPEAT_COUNT-th stall within STALL_REPEAT_WINDOW_MS ->
  *       FAULT (MOTOR_FAULT_REPEATED_STALL, reset only).
  * A stall is an expected event (a wall, a stuck wheel), so the first ones
  * only latch: the sender has to stop and re-arm on purpose. Repeated stalls
  * are the one thermal risk the firmware can bound while the stall current is
  * unknown (it depends on the winding resistance, not measured yet), so they
  * end in a reset-only FAULT.
  *
  * Why these numbers (lift, 2026-10-15 captures): 128 ticks (8 %) is about
  * twice the running dead zone (66 ticks), where a free wheel already turns
  * at ~640 counts/s (13 units); 2 units (100 counts/s) is far below that and
  * above the estimator's +-1 unit noise; 500 ms is well past a normal start
  * (the duty passes 128 ticks after 64 ms of ramp and the wheel breaks away
  * around 156-188 ticks, i.e. within ~30 ms more). Below 8 % duty a stalled
  * motor isn't detected; its current is small then (at most 8 % of the
  * battery voltage across the winding).
  *
  * An invalid estimate is "can't tell", not "stopped": the condition is
  * false, so detection is suspended (and the 500 ms start again). In closed
  * loop the speed controller latches on its own if the estimate stays
  * invalid (speed_control.h). A sensor that reads fine but is frozen is
  * indistinguishable from a stall, and treated as one (the safe way round).
  *
  * Second rule, "saturation stall", closed loop ("v") only:
  *   motor RUNNING, |applied duty| at the cap (MOTOR_DUTY_CAP_TICKS) in the
  *   setpoint's direction, the estimate VALID and the speed in the
  *   setpoint's direction below STALL_SAT_SPEED_NUMERATOR /
  *   STALL_SAT_SPEED_DENOMINATOR (1/2) of the effective setpoint, without a
  *   break for STALL_SAT_HOLD_MS -> the same response as a stall (latch,
  *   stall_count + 1, counts towards the 3-in-60 s FAULT), except that the
  *   cause is MOTOR_LATCH_STALL_SATURATION and saturation_stall_count + 1:
  *   the Pi reacts differently (an obstacle vs an unreachable setpoint).
  * Why: a wheel that is held but slips slowly (bench 2026-10-15: a gloved
  * hold at v 20 crept at 3-9 units) never stays below the first rule's
  * 2 units, so its 500 ms never complete, while the PI winds the duty up to
  * the cap and keeps it there. Any of these breaks the condition and starts
  * the 1000 ms again: the duty leaving the cap, the speed reaching half the
  * setpoint, an invalid estimate, "v 0" or a setpoint below the minimum (no
  * effective setpoint), any "m" (open loop), and a sign change (the duty
  * can't be at the cap of the new sign without ramping through 0 first). A
  * wheel driven backwards while at the cap counts as below half (the speed is
  * taken in the setpoint's direction), so a push the wrong way latches too.
  * The 1000 ms is twice the first rule's 500: a normal step reaches the cap
  * only for a moment (the speed passes half the setpoint long before).
  *
  * BY DESIGN, on the floor: a setpoint the wheel can't reach at the 30 %
  * cap (for example v 100 on a slope, or against an obstacle the wheel
  * slips on) leaves the duty at the cap below half the setpoint, and latches
  * after 1 s like a stall. On the lift the cap gives ~93 units, so any v up
  * to 100 stays above half of it. The Pi must command reachable speeds and
  * re-arm ("v 0") on purpose after a latch; three of these within 60 s are a
  * reset-only FAULT (mfault 7), like three ordinary stalls.
  *
  * Pure C; host-tested by firmware/tests/test_stall_detect.c (the rules
  * placed exactly) and firmware/tests/test_saturation_stall.c (the closed
  * loop with the real controller, estimator and a plant model). No globals.
  ******************************************************************************
  */
#ifndef STALL_DETECT_H
#define STALL_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control.h"
#include "speed_control.h"
#include "speed_estimator.h"

#define STALL_MIN_DUTY_TICKS     128     /* 8 % */
#define STALL_MAX_SPEED_UNITS    2       /* |estimate| below this: not turning */
#define STALL_HOLD_MS            500u
#define STALL_REPEAT_COUNT       3u
#define STALL_REPEAT_WINDOW_MS   60000u

/* Saturation stall ("v" only): at the cap, below 1/2 of the setpoint, for 1 s. */
#define STALL_SAT_HOLD_MS            1000u
#define STALL_SAT_SPEED_NUMERATOR    1
#define STALL_SAT_SPEED_DENOMINATOR  2

typedef struct
{
  bool     holding;                          /* the condition holds, since since_ms */
  uint32_t since_ms;
  bool     sat_holding;                      /* the saturation condition holds, since sat_since_ms */
  uint32_t sat_since_ms;
  uint32_t stall_count;                      /* since boot, both rules (telemetry: stall_n) */
  uint32_t locked_stall_count;               /* since boot, rule 1 only (Pi link STATUS) */
  uint32_t saturation_stall_count;           /* since boot, rule 2 only (Pi link STATUS) */
  /* The previous STALL_REPEAT_COUNT - 1 stalls, the oldest at next_slot:
   * with the one happening now, the last STALL_REPEAT_COUNT. */
  uint32_t stall_times_ms[STALL_REPEAT_COUNT - 1u];
  uint32_t next_slot;
} StallDetector;

void StallDetect_Init(StallDetector *detector);

/* Every ms, before MotorController_Update, on the estimator's latest result
 * (main.c's order: the previous ms's read). speed: read only (mode and
 * effective setpoint, for the saturation rule). */
void StallDetect_Update(StallDetector *detector, MotorController *motor, const SpeedController *speed,
                        const SpeedEstimator *estimate, uint32_t now_ms);

#endif /* STALL_DETECT_H */
