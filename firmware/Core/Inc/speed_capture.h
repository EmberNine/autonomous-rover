/**
  ******************************************************************************
  * @file    speed_capture.h
  * @brief   TEMPORARY bench instrumentation for identifying the drive motor
  *          (gain, time constant, dead time, dead zone) before the speed
  *          controller is tuned: records the motor's response to one "m"
  *          step into RAM, one sample per period, and then hands it out as
  *          text lines for tools/fit_motor_step.py. No hardware access: the
  *          caller passes the motor and encoder values every ms and sends
  *          the lines.
  *
  * Only in a capture build (SPEED_CAPTURE 1). In a normal build main.c
  * compiles none of it: the "c" command doesn't exist (rejected like any
  * unknown letter) and nothing is recorded or sent.
  *
  * Use (capture build, motor stopped, console running):
  *   c <p>   arm, one sample every p ms (1..SPEED_CAPTURE_MAX_PERIOD_MS):
  *           c 1 = 3 s at 1 kHz (step response), c 4 = 12 s (enough
  *           wheel revolutions at m 10 for the once-per-revolution ripple)
  *   m 20    the next "m" that changes the motor's target starts recording
  *   m 0     (within the capture) records the ramp down and the coast too
  *   c 0     cancels (recording or sending)
  * After SPEED_CAPTURE_SAMPLES samples the capture is sent as one header
  * line and then one data line per telemetry window, each after that
  * window's telemetry line (main.c):
  *   caph id=1 per=1 n=3000 t0=123456 count0=-1234 raw0=2048 vbat=7902 prev=0 target=320
  *   cap id=1 i=0 d=0,1,1,x,2,... u=2,4,6,8,...
  * d = encoder counts since the previous good sample ("x" = the angle read
  * failed at that sample: missing, NOT zero motion; the next good d covers
  * the gap, so positions stay exact), u = applied duty in signed timer ticks
  * (1600 = 100 %). Each data line has SPEED_CAPTURE_SAMPLES_PER_LINE
  * samples starting at sample i.
  *
  * What starts a recording is any change of the motor's target while armed:
  * normally the "m" step, but a watchdog trip or a lockout (target back to
  * 0) does too. Repeated keepalives of the same "m" don't.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_speed_capture.c. No globals: the caller owns the
  * sample buffer.
  ******************************************************************************
  */
#ifndef SPEED_CAPTURE_H
#define SPEED_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "telemetry_format.h"

/* 0 = normal build. 1 = capture build (bench identification only): +12 KB
 * RAM for the samples, the "c" command, and "caph"/"cap" lines that
 * serial_logger.py counts as bad lines. Must be committed as 0:
 * tools/test_firmware_sources.py fails otherwise. */
#define SPEED_CAPTURE  0

#if SPEED_CAPTURE
#warning "SPEED_CAPTURE build: 'c' command and capture lines. Bench identification only."
#endif

#define SPEED_CAPTURE_SAMPLES            3000u  /* 12 KB of samples */
#define SPEED_CAPTURE_MAX_PERIOD_MS      10u
#define SPEED_CAPTURE_SAMPLES_PER_LINE   20u    /* ~150 bytes per data line */
#define SPEED_CAPTURE_MISSING            INT16_MIN  /* d of a sample whose angle read failed */

typedef struct
{
  int16_t delta_counts;  /* since the previous good sample, or SPEED_CAPTURE_MISSING */
  int16_t duty_ticks;    /* applied duty, signed */
} SpeedCaptureSample;

typedef enum
{
  SPEED_CAPTURE_IDLE = 0,
  SPEED_CAPTURE_ARMED,      /* waiting for the motor's target to change */
  SPEED_CAPTURE_RECORDING,
  SPEED_CAPTURE_SENDING     /* header, then the data lines */
} SpeedCaptureState;

/* What the caller hands over every ms, right after the encoder read. */
typedef struct
{
  uint32_t now_ms;
  int32_t  target_ticks;   /* motor_controller.target_ticks */
  int32_t  applied_ticks;  /* motor_controller.applied_ticks: the duty applied this ms */
  int32_t  count;          /* wheel_encoder.continuous_count (stale when read_ok is false) */
  uint16_t raw_angle;      /* latest good raw angle, 0..4095 */
  bool     read_ok;        /* this ms's angle read succeeded */
  uint32_t vbat_mv;        /* filtered pack voltage, for the header */
} SpeedCaptureInput;

typedef struct
{
  SpeedCaptureState state;
  SpeedCaptureSample *samples;
  uint32_t capacity;
  uint32_t period_ms;
  uint32_t ms_in_period;
  uint32_t sample_count;
  /* Seen every ms, armed or not, so a trigger compares with the real
   * previous target. */
  int32_t  last_target_ticks;
  bool     have_last_target;
  int32_t  last_good_count;       /* count at the last good sample */
  /* Header */
  uint32_t capture_id;            /* 1, 2, ... since boot */
  uint32_t t0_ms;
  int32_t  count0;
  uint32_t raw0;
  uint32_t vbat_mv;
  int32_t  previous_target_ticks;
  int32_t  trigger_target_ticks;
  /* Sending: line 0 is the header, line n >= 1 holds samples from
   * (n - 1) * SPEED_CAPTURE_SAMPLES_PER_LINE on. */
  uint32_t line_index;
  uint32_t item_index;            /* next piece of the line being built */
} SpeedCapture;

void SpeedCapture_Init(SpeedCapture *capture, SpeedCaptureSample *samples, uint32_t capacity);

/* A valid "c <value>" line. 0 cancels whatever is going on (true). 1..max
 * arms with that sample period if nothing is being recorded or sent
 * (true); anything else is refused (false: the caller counts it as a
 * rejected line). */
bool SpeedCapture_Command(SpeedCapture *capture, int32_t value);

/* Every ms, after the encoder read. Starts, records and ends a capture. */
void SpeedCapture_Tick(SpeedCapture *capture, const SpeedCaptureInput *input);

/* True while there are lines to send. */
bool SpeedCapture_HasLine(const SpeedCapture *capture);

/* Builds the current line into writer, at most max_items pieces per call
 * (a piece is one number or one field), so no single loop iteration does
 * much formatting. The caller starts the writer empty (TelemetryWriter_Init)
 * before the first call of each line. Returns true once the line is
 * complete, "\r\n" included. */
bool SpeedCapture_BuildLineStep(SpeedCapture *capture, TelemetryWriter *writer, uint32_t max_items);

/* The completed line was handed to the UART: go to the next one. After the
 * last line the capture is IDLE again (and can be re-armed). */
void SpeedCapture_LineSent(SpeedCapture *capture);

#endif /* SPEED_CAPTURE_H */
