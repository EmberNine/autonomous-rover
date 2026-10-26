/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   The motor state machine, with no hardware access: the caller
  *          passes "now" in ms, and gets back what the H-bridge pins should
  *          be (MotorOutput). firmware/Core/Src/tb6612.c writes that to the
  *          pins.
  *
  * Commands: "m <percent>" lines (grammar in command_line.h), routed here by
  * command_dispatch.c.
  *
  *   - the value is a signed duty in percent, -100..100 ("m -0" is 0)
  *   - lines that don't parse, or have another letter, are rejected and
  *     counted here (MotorController_CountRejected), and never acted on
  *   - a value within +-100 but beyond the cap is clamped to the cap and
  *     counted; it is still a valid command
  *
  * Behavior:
  *   - LATCHED at boot and after a watchdog trip: the driver is disabled and
  *     non-zero commands are ignored until an "m 0" arrives (like an RC ESC
  *     that only arms at zero throttle). This stops a board reset, or a
  *     command link that comes back, from restarting the motor by itself.
  *   - Duty changes are ramped at MOTOR_RAMP_TICKS_PER_MS.
  *   - Direction never changes while the duty is non-zero: a reversal ramps
  *     down to 0, holds a short brake for MOTOR_REVERSE_DWELL_MS, switches the
  *     direction pins with duty 0, then ramps up.
  *   - Stopped always means the driver is disabled (STBY low, IN1 = IN2 = low,
  *     duty 0: the motor coasts). The only time the bridge is enabled with
  *     duty 0 is the reverse dwell (short brake), and the single ms when the
  *     driver is enabled before the first non-zero duty.
  *   - Watchdog: while the driver is enabled, no valid command for
  *     MOTOR_COMMAND_TIMEOUT_MS disables it at once (no ramp) and latches.
  *     Only valid "m" commands (including clamped ones) reset the watchdog;
  *     steering commands never do (command_dispatch.c).
  *   - FAULT (MotorController_LockOut): the driver is disabled for good and
  *     no command changes that, "m 0" included; only a reset clears it.
  *     main.c enters it when the PWM timer period doesn't match
  *     MOTOR_PWM_PERIOD_TICKS at boot, or when one of the output-stage
  *     guards in tb6612_plan.c fires (which only happens if this state
  *     machine is broken), when battery_monitor locks the motor out
  *     (cutoff, battery ADC fault, pack removed while driving), and when
  *     stall_detect sees 3 stalls within 60 s. Commands are still counted.
  *   - Closed loop ("v", speed_control.c): see the section at the end. "m"
  *     and "v" both feed the watchdog; "s" never does.
  *
  * Units: duty in timer ticks, MOTOR_PWM_PERIOD_TICKS = 100 %. Signed values
  * are positive for "forward", which tb6612.c maps to the IN pins.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_motor_control.c. No globals: state lives in structs
  * the caller owns.
  ******************************************************************************
  */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* TIM1 ARR (1599) + 1: one PWM period at 20 kHz. Must match MX_TIM1_Init:
 * main.c checks it at boot and keeps the motor in FAULT if it doesn't, and
 * tb6612_plan.c refuses to enable the driver with any other period. */
#define MOTOR_PWM_PERIOD_TICKS        1600
/* 1600 / 100 = 16 ticks per percent, exactly. */
#define MOTOR_TICKS_PER_PERCENT       (MOTOR_PWM_PERIOD_TICKS / 100)
/* Highest duty the firmware will ever apply, whatever is commanded. The stall
 * current of the motor is unknown, and in slow-decay PWM a stalled motor
 * draws roughly duty x (battery voltage / winding resistance), so the cap
 * also caps the stall current. Lower this one number to be more careful. */
#define MOTOR_DUTY_CAP_PERCENT        30
#define MOTOR_DUTY_CAP_TICKS          (MOTOR_DUTY_CAP_PERCENT * MOTOR_TICKS_PER_PERCENT)  /* 480 */
/* Ramp: 2 ticks per ms = 0 to the 30 % cap in 240 ms, 0 to 100 % in 800 ms.
 * Avoids step changes in current and in torque on the drivetrain. */
#define MOTOR_RAMP_TICKS_PER_MS       2
/* After a loop stall, ramp as if at most this many ms had passed, so the
 * duty never jumps by more than 20 ticks (1.25 %) in one step. */
#define MOTOR_RAMP_MAX_STEP_MS        10
/* Short brake at zero duty before the direction pins switch, so the motor
 * has slowed down before reverse voltage is applied. */
#define MOTOR_REVERSE_DWELL_MS        100
/* No valid command for this long while the driver is enabled = stop. The PC
 * console resends the current command every 100 ms, so this tolerates four
 * lost or late keepalives. */
#define MOTOR_COMMAND_TIMEOUT_MS      500

/* ---- Motor state machine ------------------------------------------------ */

typedef enum
{
  MOTOR_STATE_OFF = 0,      /* driver disabled, waiting for a non-zero command */
  MOTOR_STATE_RUNNING = 1,  /* driver enabled: ramping, holding, reverse dwell, or ramping down */
  MOTOR_STATE_LATCHED = 2,  /* driver disabled; non-zero commands ignored until "m 0" */
  MOTOR_STATE_FAULT = 3     /* driver disabled until reset, whatever is commanded */
} MotorState;

/* Why the controller is in FAULT (the first reason; later ones are not
 * recorded). Reported as "mfault" (telemetry v5; 0 unless mstate=3), so the
 * values are part of the telemetry format (tools/motor_console.py decodes
 * them, tools/test_firmware_sources.py pins the order): append only. */
typedef enum
{
  MOTOR_FAULT_NONE = 0,
  MOTOR_FAULT_PERIOD_MISMATCH,  /* TIM1 ARR + 1 != MOTOR_PWM_PERIOD_TICKS */
  MOTOR_FAULT_CAP_GUARD,        /* a duty above MOTOR_DUTY_CAP_TICKS reached the output stage */
  MOTOR_FAULT_DIRECTION_GUARD,  /* a direction change reached the output stage with a duty still set */
  /* From battery_monitor (bstate in the telemetry shows which, too). */
  MOTOR_FAULT_LOW_BATTERY,      /* battery cutoff (bstate=3) */
  MOTOR_FAULT_BATTERY_ADC,      /* battery ADC fault: the voltage can't be trusted (bstate=4) */
  MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING, /* pack removed with a non-zero duty (bstate=5) */
  /* From stall_detect. */
  MOTOR_FAULT_REPEATED_STALL    /* 3 stalls within 60 s: stop retrying until someone looks (reset) */
} MotorFaultReason;

/* Why the controller is LATCHED right now: MOTOR_LATCH_NONE exactly when the
 * state is not LATCHED (re-arming and FAULT both clear it). Not in the text
 * telemetry (frozen at v5); reported by the Pi link's STATUS frame, so the
 * values are part of that format: append only. */
typedef enum
{
  MOTOR_LATCH_NONE = 0,
  MOTOR_LATCH_BOOT,              /* Init: latched until the first re-arm */
  MOTOR_LATCH_WATCHDOG,          /* no valid motor command for MOTOR_COMMAND_TIMEOUT_MS */
  MOTOR_LATCH_STALL_LOCKED,      /* stall_detect rule 1: duty on, wheel not turning */
  MOTOR_LATCH_STALL_SATURATION,  /* stall_detect rule 2: at the cap below half the setpoint */
  MOTOR_LATCH_ESTIMATE_INVALID,  /* speed_control: the estimate stayed invalid under "v" */
  MOTOR_LATCH_ESTOP,             /* Pi link: ESTOP frame */
  MOTOR_LATCH_LINK_HELLO         /* Pi link: HELLO (entering binary mode, or a bridge restart) */
} MotorLatchReason;

typedef enum
{
  MOTOR_DIRECTION_COAST = 0,  /* IN1 = IN2 = low */
  MOTOR_DIRECTION_FORWARD,
  MOTOR_DIRECTION_REVERSE
} MotorDirection;

/* What the H-bridge should do right now. When driver_enabled is false,
 * direction is COAST and duty_ticks is 0. */
typedef struct
{
  bool           driver_enabled;  /* STBY high */
  MotorDirection direction;
  uint32_t       duty_ticks;      /* 0..MOTOR_DUTY_CAP_TICKS */
} MotorOutput;

typedef struct
{
  MotorState     state;
  int32_t        requested_percent;  /* last valid command as received, -100..100 */
  int32_t        target_ticks;       /* what the ramp heads for, after the cap; signed */
  int32_t        applied_ticks;      /* duty applied now; signed, same sign as bridge_direction */
  MotorDirection bridge_direction;   /* how the IN pins are set while RUNNING */
  bool           dwell_active;       /* holding a short brake before a reversal */
  uint32_t       dwell_start_ms;
  uint32_t       last_command_ms;    /* last valid command: the watchdog's reference */
  uint32_t       last_update_ms;
  MotorFaultReason fault_reason;     /* MOTOR_FAULT_NONE unless state is FAULT */
  MotorLatchReason latch_reason;     /* MOTOR_LATCH_NONE unless state is LATCHED */

  /* Totals since boot (never reset) */
  uint32_t       valid_command_count;
  uint32_t       rejected_command_count;
  uint32_t       clamped_command_count;
  uint32_t       watchdog_trip_count;
  uint32_t       latch_count;        /* MotorController_Latch calls that latched (stall, encoder) */
} MotorController;

/* Starts LATCHED with the driver disabled. */
void MotorController_Init(MotorController *controller, uint32_t now_ms);

/* A valid command (percent within +-100) arrived. Resets the watchdog. */
void MotorController_AcceptCommand(MotorController *controller, int32_t percent, uint32_t now_ms);

/* A rejected line arrived (any letter: the link's rejected-line count lives
 * here because the motor was its first user). Only counted; the watchdog
 * is NOT reset. */
void MotorController_CountRejected(MotorController *controller);

/* Advances the ramp, dwell and watchdog to now_ms and returns the pin states.
 * Call once per loop iteration (every 1 ms). */
MotorOutput MotorController_Update(MotorController *controller, uint32_t now_ms);

/* Enters FAULT from any state: the next Update returns "disabled" (no ramp),
 * and so does every one after it. Keeps the first reason if called again. */
void MotorController_LockOut(MotorController *controller, MotorFaultReason reason);

/* ---- Closed-loop speed control (speed_control.c) ---------------------------
 * The speed controller drives the motor THROUGH this state machine, never
 * around it: its output becomes target_ticks, so the cap, the ramp, the
 * reverse dwell, the watchdog, the latch, FAULT and the output-stage guards
 * all apply exactly as for "m". Two halves of what AcceptCommand does for
 * "m", kept apart on purpose: */

/* A valid "v" command arrived: counted (valid_command_count), FEEDS the
 * watchdog, and "v 0" re-arms a latch like "m 0". Sets no target: that's
 * the speed controller's job. Ignored (but counted) in FAULT. */
void MotorController_AcceptSpeedCommand(MotorController *controller, int32_t setpoint_units, uint32_t now_ms);

/* The Pi link's DRIVE frame (link_rx.c): the same as AcceptSpeedCommand,
 * except that re-arming is explicit: counted, FEEDS the watchdog, and
 * re-arms a latch ONLY when arm is true. The caller accepts arm only
 * together with a zero setpoint (link_protocol.c rejects anything else), so
 * a DRIVE 0 without arm keeps the motor latched, and so does any non-zero
 * DRIVE. Sets no target. Ignored (but counted) in FAULT. */
void MotorController_AcceptDriveCommand(MotorController *controller, bool arm, uint32_t now_ms);

/* The speed controller's output: target_ticks, clamped to +-MOTOR_DUTY_CAP_TICKS.
 * Ignored unless OFF or RUNNING (so never while LATCHED or in FAULT), and
 * does NOT feed the watchdog: if the "v" commands stop, the motor still
 * trips MOTOR_COMMAND_TIMEOUT_MS after the last one. */
void MotorController_SetClosedLoopTarget(MotorController *controller, int32_t target_ticks);

/* Stop at once (driver disabled, no ramp) and latch until "m 0" / "v 0",
 * like a watchdog trip but without counting one (latch_count instead):
 * used for a stall and for a speed estimate that stays invalid. No effect
 * in FAULT (stays FAULT) or when already LATCHED (the first reason stays). */
void MotorController_Latch(MotorController *controller, MotorLatchReason reason);

#endif /* MOTOR_CONTROL_H */
