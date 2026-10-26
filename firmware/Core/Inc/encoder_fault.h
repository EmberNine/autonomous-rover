/**
  ******************************************************************************
  * @file    encoder_fault.h
  * @brief   TEST BUILD ONLY (ENCODER_FAULT_TEST): injects encoder faults on
  *          command, to prove on the bench that the closed loop and the stall
  *          check react to them as designed. No hardware access.
  *
  * "e <n>" (only in an ENCODER_FAULT_TEST build; refused otherwise, like any
  * unknown letter) starts one fault, applied to the angle read of each ms
  * from the ms the command arrives in:
  *   e 1  the angle read fails for 20 ms   (estimate invalid ~41 ms: under
  *        "v" the controller holds; no latch, enc_bad unchanged)
  *   e 2  the angle read fails for 100 ms  (under "v": latch at the 52nd
  *        invalid ms, enc_bad + 1; under "m": nothing latches)
  *   e 3  the angle freezes for 700 ms: the reads succeed but return the
  *        angle of the first frozen ms (a valid estimate of 0: with duty
  *        >= 128 ticks the stall check latches after 500 ms, stall_n + 1;
  *        three within 60 s give FAULT, mfault 7)
  * Any other value, or an "e" while a fault is still running, is refused and
  * counted as a rejected line. "e" feeds no watchdog and moves nothing by
  * itself. An injected failed read is not counted in i2c_err (that is the
  * bus's own count); a real failed read during a freeze stays a failed read.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_encoder_fault.c. No globals.
  ******************************************************************************
  */
#ifndef ENCODER_FAULT_H
#define ENCODER_FAULT_H

#include <stdbool.h>
#include <stdint.h>

/* 0 = normal build. 1 = encoder fault test build (bench only): the "e"
 * command. Must be committed as 0: tools/test_firmware_sources.py fails
 * otherwise. Not together with SPEED_CAPTURE (main.c: #error). */
#define ENCODER_FAULT_TEST  0

#if ENCODER_FAULT_TEST
#warning "ENCODER_FAULT_TEST build: 'e' command injects encoder faults. Bench test only."
#endif

#define ENCODER_FAULT_SHORT_OUTAGE_MS   20u    /* e 1: held, no latch */
#define ENCODER_FAULT_LONG_OUTAGE_MS    100u   /* e 2: longer than the 50 ms hold: latch */
#define ENCODER_FAULT_FROZEN_ANGLE_MS   700u   /* e 3: longer than the 500 ms stall hold */

typedef enum
{
  ENCODER_FAULT_NONE = 0,
  ENCODER_FAULT_SHORT_OUTAGE = 1,   /* e 1 */
  ENCODER_FAULT_LONG_OUTAGE = 2,    /* e 2 */
  ENCODER_FAULT_FROZEN_ANGLE = 3    /* e 3 */
} EncoderFaultKind;

typedef struct
{
  EncoderFaultKind kind;           /* the fault running now, or NONE */
  uint32_t         start_ms;
  uint32_t         duration_ms;
  bool             frozen_angle_set;
  uint16_t         frozen_angle;   /* e 3: the angle every read returns */
  uint32_t         started_count;  /* faults started since boot */
} EncoderFault;

void EncoderFault_Init(EncoderFault *fault);

/* An "e <value>" line. Returns false (refused: count it as a rejected line)
 * for a value other than 1, 2 or 3, or while a fault is still running. */
bool EncoderFault_Command(EncoderFault *fault, int32_t value, uint32_t now_ms);

/* Every ms, right after the real angle read (read_ok: its result; raw_angle:
 * the angle it returned, if read_ok). Turns the read into a failed one, or
 * replaces the angle with the frozen one, while a fault runs. */
void EncoderFault_Apply(EncoderFault *fault, bool *read_ok, uint16_t *raw_angle, uint32_t now_ms);

#endif /* ENCODER_FAULT_H */
