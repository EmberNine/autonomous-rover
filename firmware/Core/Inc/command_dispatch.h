/**
  ******************************************************************************
  * @file    command_dispatch.h
  * @brief   Routes each finished command line to the one controller it is
  *          for. The only place that decides what a letter means:
  *
  *   "m <percent>"  -> MotorController_AcceptCommand   (feeds the motor watchdog)
  *                     + SpeedControl_SelectOpenLoop    (closed loop off)
  *   "v <units>"    -> MotorController_AcceptSpeedCommand (feeds the motor watchdog)
  *                     + SpeedControl_SetSetpoint        (closed loop on)
  *                     only with a speed controller; otherwise rejected
  *   "s <percent>"  -> SteeringController_AcceptCommand (feeds the steering timeout)
  *   "h <n>"        -> loop stall of n x 10 ms, ONLY in an IWDG_HANG_TEST build
  *   anything else  -> rejected, counted in cmd_bad (MotorController_CountRejected)
  *
  * "m" and "v" feed the motor watchdog; "s" feeds the steering's timeout;
  * nothing else feeds either. In particular a steady stream of "s" lines can
  * never keep the motor running: only "m" and "v" reach the two functions
  * that write the motor watchdog's reference time (AcceptCommand,
  * AcceptSpeedCommand), and the speed controller's own output
  * (SetClosedLoopTarget) doesn't (tests/test_command_dispatch.c checks this
  * end to end). The Pi only ever uses "v"; "m" is a bench/diagnostic command.
  *
  * Known limitation (as for "s" -> "m"): "s" (0x73) and "v" (0x76) differ in
  * 2 bits, so a byte corrupted in a way the UART doesn't flag could turn an
  * "s" line into a "v" line; bounded by the same cap, ramp and watchdog
  * (README, open items; the planned checksummed protocol removes it).
  *
  * Pure C with no HAL or MCU dependencies. No globals.
  ******************************************************************************
  */
#ifndef COMMAND_DISPATCH_H
#define COMMAND_DISPATCH_H

#include <stdbool.h>
#include <stdint.h>

#include "command_line.h"
#include "motor_control.h"
#include "speed_control.h"
#include "steering_control.h"

/* Deliberate-hang test for the hardware watchdog (IWDG). 0 = normal build:
 * the "h" command doesn't exist and is rejected like any unknown letter.
 * 1 = test build: "h <n>" (n = 1..100) stalls the main loop for n x 10 ms,
 * to check that a 200 ms stall does NOT reset the MCU and a 300 ms one
 * does. Never put a test build on the truck. Must be committed as 0:
 * tests/test_command_dispatch.c fails otherwise. */
#define IWDG_HANG_TEST  0

#if IWDG_HANG_TEST
#warning "IWDG_HANG_TEST build: 'h <n>' stalls the main loop. Not for the truck."
#endif

/* One "h" unit. */
#define HANG_TEST_MS_PER_UNIT  10u

typedef enum
{
  COMMAND_TARGET_NONE = 0,  /* incomplete or empty line: nothing happened */
  COMMAND_TARGET_MOTOR,
  COMMAND_TARGET_STEERING,
  COMMAND_TARGET_HANG,      /* the caller stalls the loop for hang_ms */
  COMMAND_TARGET_REJECTED,  /* counted in the motor controller's rejected count */
  COMMAND_TARGET_SPEED      /* "v": motor watchdog fed, speed controller's setpoint set */
} CommandTarget;

typedef struct
{
  CommandTarget target;
  uint32_t      hang_ms;  /* only for COMMAND_TARGET_HANG */
} CommandDispatchResult;

/* Acts on one CommandLine_FeedByte result. command is read only when
 * line_result is COMMAND_LINE_VALID. hang_test_enabled: pass
 * (IWDG_HANG_TEST != 0); a parameter rather than the macro itself so the
 * host test can check both builds. */
CommandDispatchResult CommandDispatch_Line(CommandLineResult line_result, const Command *command,
                                           MotorController *motor, SteeringController *steering,
                                           bool hang_test_enabled, uint32_t now_ms);

/* The same, with the closed-loop speed controller: "v" goes to it (and to
 * the motor watchdog), "m" switches it off. speed may be NULL: then "v" is
 * rejected like any unknown letter, exactly as CommandDispatch_Line does
 * (it calls this with NULL). */
CommandDispatchResult CommandDispatch_LineWithSpeed(CommandLineResult line_result, const Command *command,
                                                    MotorController *motor, SteeringController *steering,
                                                    SpeedController *speed, bool hang_test_enabled, uint32_t now_ms);

#endif /* COMMAND_DISPATCH_H */
