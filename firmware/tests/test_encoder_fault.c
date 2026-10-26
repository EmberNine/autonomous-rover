/*
 * Host-side test for Core/Src/encoder_fault.c (ENCODER_FAULT_TEST builds
 * only): the injection itself, then each "e" fault through the real chain in
 * main.c's order (speed control, stall check, motor, a wheel model, the angle
 * read with the fault applied, the corrected unwrapper, the estimator), with
 * the reaction the bench plan predicts. Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -Itests/stubs -ICore/Inc tests/test_encoder_fault.c Core/Src/encoder_fault.c Core/Src/as5600.c Core/Src/angle_correction.c Core/Src/speed_estimator.c Core/Src/speed_control.c Core/Src/stall_detect.c Core/Src/motor_control.c -lm -o /tmp/test_encoder_fault && /tmp/test_encoder_fault
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "angle_correction.h"
#include "as5600.h"
#include "encoder_fault.h"
#include "speed_control.h"
#include "speed_estimator.h"
#include "stall_detect.h"

/* as5600.c's register reads are never called here (only its unwrapper); this
 * satisfies the linker. */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize; (void)pData; (void)Size; (void)Timeout;
  return HAL_ERROR;
}

static int failure_count = 0;
static int check_count = 0;

#define CHECK(condition, ...)                                   \
  do                                                            \
  {                                                             \
    check_count++;                                              \
    if (!(condition))                                           \
    {                                                           \
      failure_count++;                                          \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);               \
      printf(__VA_ARGS__);                                      \
      printf("\n");                                             \
    }                                                           \
  } while (0)

/* ---- The injection on its own ----------------------------------------------------- */

static void test_pinned_durations_and_codes(void)
{
  CHECK(ENCODER_FAULT_TEST == 0, "the switch is committed as 0");
  CHECK(ENCODER_FAULT_SHORT_OUTAGE_MS == 20u && ENCODER_FAULT_LONG_OUTAGE_MS == 100u &&
        ENCODER_FAULT_FROZEN_ANGLE_MS == 700u, "e 1 20 ms, e 2 100 ms, e 3 700 ms");
  /* The point of each: e 1 is held (shorter than the hold, even with the
   * estimator's 21 ms refill), e 2 latches, e 3 outlasts the stall hold. */
  CHECK(ENCODER_FAULT_SHORT_OUTAGE_MS + SPEED_RING_SIZE < SPEED_INVALID_HOLD_MS, "e 1 + refill < the 50 ms hold");
  CHECK(ENCODER_FAULT_LONG_OUTAGE_MS > SPEED_INVALID_HOLD_MS, "e 2 > the 50 ms hold");
  CHECK(ENCODER_FAULT_FROZEN_ANGLE_MS > STALL_HOLD_MS + SPEED_RING_SIZE + 50u, "e 3 > window + stall hold, with margin");
}

static void test_command_refusals(void)
{
  EncoderFault fault;
  EncoderFault_Init(&fault);
  CHECK(!EncoderFault_Command(&fault, 0, 10u) && !EncoderFault_Command(&fault, 4, 10u) &&
        !EncoderFault_Command(&fault, -1, 10u) && !EncoderFault_Command(&fault, 100, 10u),
        "values other than 1-3 are refused");
  CHECK(fault.kind == ENCODER_FAULT_NONE && fault.started_count == 0u, "and start nothing");
  CHECK(EncoderFault_Command(&fault, 1, 10u), "e 1 accepted");
  CHECK(!EncoderFault_Command(&fault, 2, 11u) && fault.kind == ENCODER_FAULT_SHORT_OUTAGE,
        "a second e while one runs is refused, the first one carries on");
  bool ok = true;
  uint16_t raw = 5u;
  EncoderFault_Apply(&fault, &ok, &raw, 30u);   /* 20 ms after the start: over */
  CHECK(ok && fault.kind == ENCODER_FAULT_NONE, "over after its duration");
  CHECK(EncoderFault_Command(&fault, 2, 31u) && fault.started_count == 2u, "then a new one is accepted");
}

/* Counts the ms a fault changes, applying it every ms from the command's ms. */
static void test_outage_lengths_are_exact(void)
{
  static const struct { int32_t value; uint32_t expected; uint32_t start; } cases[] = {
    { 1, 20u, 1000u }, { 2, 100u, 1000u }, { 1, 20u, UINT32_MAX - 7u }, { 2, 100u, UINT32_MAX - 50u },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    EncoderFault fault;
    EncoderFault_Init(&fault);
    CHECK(EncoderFault_Command(&fault, cases[i].value, cases[i].start), "e %d accepted", (int)cases[i].value);
    uint32_t failed = 0u;
    uint32_t last_failed_offset = 0u;
    for (uint32_t offset = 0u; offset < 300u; offset++)
    {
      bool ok = true;
      uint16_t raw = (uint16_t)(offset % 4096u);
      EncoderFault_Apply(&fault, &ok, &raw, cases[i].start + offset);
      if (!ok) { failed++; last_failed_offset = offset; }
      if (ok) { CHECK(raw == (uint16_t)(offset % 4096u), "a read let through keeps its angle"); }
    }
    CHECK(failed == cases[i].expected && last_failed_offset == cases[i].expected - 1u,
          "e %d from t=%u: %u failed reads, ms 0..%u (expected %u)", (int)cases[i].value, (unsigned)cases[i].start,
          (unsigned)failed, (unsigned)last_failed_offset, (unsigned)cases[i].expected);
  }
}

static void test_frozen_angle(void)
{
  EncoderFault fault;
  EncoderFault_Init(&fault);
  CHECK(EncoderFault_Command(&fault, 3, 500u), "e 3 accepted");
  uint32_t frozen = 0u;
  uint32_t failed = 0u;
  for (uint32_t offset = 0u; offset < 1000u; offset++)
  {
    bool ok = (offset != 300u);   /* one real failure in the middle */
    uint16_t raw = (uint16_t)((1234u + 3u * offset) % 4096u);
    EncoderFault_Apply(&fault, &ok, &raw, 500u + offset);
    if (!ok) { failed++; continue; }
    if (raw == 1234u) { frozen++; }
    if (offset >= 700u) { CHECK(raw == (uint16_t)((1234u + 3u * offset) % 4096u), "real angle again after 700 ms"); }
  }
  CHECK(frozen == 699u, "the first ms's angle is returned for the whole 700 ms (%u, one ms was a real failure)",
        (unsigned)frozen);
  CHECK(failed == 1u, "a real failed read during the freeze stays a failed read");
  CHECK(fault.kind == ENCODER_FAULT_NONE, "over");

  /* If the first frozen ms is a failed read, the freeze starts at the next good one. */
  EncoderFault_Init(&fault);
  CHECK(EncoderFault_Command(&fault, 3, 0u), "e 3 again");
  bool ok = false;
  uint16_t raw = 77u;
  EncoderFault_Apply(&fault, &ok, &raw, 0u);
  ok = true;
  raw = 88u;
  EncoderFault_Apply(&fault, &ok, &raw, 1u);
  ok = true;
  raw = 99u;
  EncoderFault_Apply(&fault, &ok, &raw, 2u);
  CHECK(raw == 88u, "frozen at the first good read (%u)", (unsigned)raw);
}

/* ---- Through the whole chain, in main.c's order --------------------------------------- */

typedef struct
{
  MotorController  motor;
  SpeedController  speed;
  StallDetector    stall;
  SpeedEstimator   estimator;
  EncoderFault     fault;
  AS5600_Unwrapper corrected;
  uint32_t         now_ms;
  double           wheel_speed;     /* counts/ms */
  double           position;        /* counts */
  bool             keepalive_is_v;
  int32_t          keepalive_value;
  int32_t          max_applied;
} Chain;

static void chain_send_v(Chain *chain, int32_t units)
{
  MotorController_AcceptSpeedCommand(&chain->motor, units, chain->now_ms);
  SpeedControl_SetSetpoint(&chain->speed, units, chain->motor.applied_ticks);
  chain->keepalive_is_v = true;
  chain->keepalive_value = units;
}

static void chain_send_m(Chain *chain, int32_t percent)
{
  MotorController_AcceptCommand(&chain->motor, percent, chain->now_ms);
  SpeedControl_SelectOpenLoop(&chain->speed);
  chain->keepalive_is_v = false;
  chain->keepalive_value = percent;
}

static void chain_init(Chain *chain)
{
  memset(chain, 0, sizeof(*chain));
  chain->now_ms = 5000u;
  MotorController_Init(&chain->motor, chain->now_ms);
  SpeedControl_Init(&chain->speed);
  StallDetect_Init(&chain->stall);
  SpeedEstimator_Init(&chain->estimator);
  EncoderFault_Init(&chain->fault);
  AS5600_UnwrapperInit(&chain->corrected);
  chain->position = 1000.3;
}

/* One ms. e_value != 0: an "e" line arrives in this ms (before the read, as in main.c). */
static void chain_step(Chain *chain, int32_t e_value)
{
  chain->now_ms++;
  if (chain->now_ms % 100u == 0u)   /* the console's keepalive */
  {
    if (chain->keepalive_is_v) { chain_send_v(chain, chain->keepalive_value); }
    else { chain_send_m(chain, chain->keepalive_value); }
  }
  if (e_value != 0) { CHECK(EncoderFault_Command(&chain->fault, e_value, chain->now_ms), "e %d accepted", (int)e_value); }
  SpeedControl_Update(&chain->speed, &chain->motor, &chain->estimator, chain->now_ms);
  StallDetect_Update(&chain->stall, &chain->motor, &chain->speed, &chain->estimator, chain->now_ms);
  MotorOutput output = MotorController_Update(&chain->motor, chain->now_ms);
  if (abs(chain->motor.applied_ticks) > chain->max_applied) { chain->max_applied = abs(chain->motor.applied_ticks); }

  /* Wheel on the lift: 0.0123 counts/ms per tick above a 104-tick dead zone, tau 30 ms, coast tau 400 ms. */
  double duty = output.driver_enabled
                    ? ((output.direction == MOTOR_DIRECTION_REVERSE) ? -(double)output.duty_ticks : (double)output.duty_ticks)
                    : 0.0;
  double target = copysign(fmax(fabs(duty) - 104.0, 0.0), duty) * 0.0123;
  if (!output.driver_enabled) { chain->wheel_speed *= exp(-1.0 / 400.0); }
  else { chain->wheel_speed += (target - chain->wheel_speed) * (1.0 - exp(-1.0 / 30.0)); }
  chain->position += chain->wheel_speed;

  /* The sensor reads the wheel angle with its 4-per-revolution error, the
   * one AngleCorrection removes: raw - eps(raw) = true angle, solved for raw
   * by fixed-point iteration (eps' <= 0.27, so it converges fast). */
  long true_angle = (long)floor(chain->position);
  long raw_estimate = true_angle;
  for (int i = 0; i < 6; i++)
  {
    raw_estimate = true_angle + AngleCorrection_ErrorCounts((uint16_t)(raw_estimate & 4095L));
  }
  /* The read, the fault, the corrected unwrapper and the estimate, as in main.c. */
  uint16_t raw = (uint16_t)(raw_estimate & 4095L);
  bool read_ok = true;
  EncoderFault_Apply(&chain->fault, &read_ok, &raw, chain->now_ms);
  if (read_ok) { AS5600_UnwrapperUpdate(&chain->corrected, AngleCorrection_Apply(raw)); }
  SpeedEstimatorInput input = { .now_ms = chain->now_ms, .count = chain->corrected.continuous_count,
                                .read_ok = read_ok, .magnet_detected = true };
  SpeedEstimator_Update(&chain->estimator, &input);
}

static void chain_run(Chain *chain, uint32_t ms)
{
  for (uint32_t i = 0u; i < ms; i++) { chain_step(chain, 0); }
}

/* Runs an "e" from steady driving; returns the ms after the command at which
 * the motor left RUNNING (0 = it never did within duration_ms). */
static uint32_t chain_fault(Chain *chain, int32_t e_value, uint32_t duration_ms)
{
  uint32_t left_running = 0u;
  for (uint32_t t = 0u; t < duration_ms; t++)
  {
    chain_step(chain, (t == 0u) ? e_value : 0);
    if (left_running == 0u && chain->motor.state != MOTOR_STATE_RUNNING) { left_running = t + 1u; }
  }
  return left_running;
}

static void test_e1_under_v_is_held(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_v(&chain, 0);
  chain_run(&chain, 100u);
  chain_send_v(&chain, 50);
  chain_run(&chain, 2000u);
  CHECK(chain.motor.state == MOTOR_STATE_RUNNING && chain.estimator.valid, "v 50 running before the fault");
  CHECK(abs(chain.estimator.speed_units - 50) <= 3, "and settled: the correction cancels the sensor's error (%d)",
        (int)chain.estimator.speed_units);
  uint32_t invalid_max = 0u;
  uint32_t left = 0u;
  for (uint32_t t = 0u; t < 500u; t++)
  {
    chain_step(&chain, (t == 0u) ? 1 : 0);
    if (chain.estimator.invalid_ms > invalid_max) { invalid_max = chain.estimator.invalid_ms; }
    if (left == 0u && chain.motor.state != MOTOR_STATE_RUNNING) { left = t + 1u; }
  }
  CHECK(left == 0u && chain.speed.enc_bad_count == 0u && chain.motor.latch_count == 0u,
        "e 1 under v 50: held, no latch, enc_bad 0");
  CHECK(invalid_max >= 39u && invalid_max <= 41u, "estimate invalid for ~41 ms (invalid_ms reached %u)",
        (unsigned)invalid_max);
  CHECK(abs(chain.estimator.speed_units - 50) <= 6, "still at v 50 afterwards (%d)", (int)chain.estimator.speed_units);
}

static void test_e2_under_v_latches_and_counts_enc_bad(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_v(&chain, 0);
  chain_run(&chain, 100u);
  chain_send_v(&chain, 50);
  chain_run(&chain, 2000u);
  uint32_t left = chain_fault(&chain, 2, 300u);
  /* Invalid from the command's ms (invalid_ms 0) to invalid_ms 50 at the
   * 51st; the controller acts on that one ms later: the 52nd. */
  CHECK(left == 52u, "e 2 under v 50: latched at the 52nd failing ms (%u)", (unsigned)left);
  CHECK(chain.motor.state == MOTOR_STATE_LATCHED && chain.speed.enc_bad_count == 1u && chain.stall.stall_count == 0u,
        "LATCHED, enc_bad 1, stall_n 0");
  CHECK(chain.motor.applied_ticks == 0, "coasting (duty 0)");
  chain_send_v(&chain, 0);
  chain_step(&chain, 0);
  CHECK(chain.motor.state == MOTOR_STATE_OFF, "v 0 re-arms");
}

static void test_e2_under_m_does_not_latch(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_m(&chain, 0);
  chain_run(&chain, 100u);
  chain_send_m(&chain, 30);
  chain_run(&chain, 2000u);
  uint32_t left = chain_fault(&chain, 2, 1000u);
  CHECK(left == 0u && chain.motor.applied_ticks == 480, "e 2 under m 30: still RUNNING at 480 ticks");
  CHECK(chain.speed.enc_bad_count == 0u && chain.stall.stall_count == 0u && chain.motor.latch_count == 0u,
        "no enc_bad, no stall (suspended while the estimate is invalid)");
}

static void test_e3_under_m_is_a_stall(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_m(&chain, 0);
  chain_run(&chain, 100u);
  chain_send_m(&chain, 20);
  chain_run(&chain, 2000u);
  uint32_t left = chain_fault(&chain, 3, 1000u);
  /* The estimate falls to 0 once the 20 ms window holds only frozen counts;
   * the stall then needs 500 ms more (plus the 1 ms pipeline). */
  CHECK(left >= STALL_HOLD_MS + 15u && left <= STALL_HOLD_MS + SPEED_RING_SIZE + 2u,
        "e 3 under m 20: stall latch %u ms after the command", (unsigned)left);
  CHECK(chain.motor.state == MOTOR_STATE_LATCHED && chain.stall.stall_count == 1u && chain.speed.enc_bad_count == 0u,
        "LATCHED, stall_n 1, enc_bad 0");
}

static void test_e3_under_v_speeds_up_then_latches(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_v(&chain, 0);
  chain_run(&chain, 100u);
  chain_send_v(&chain, 50);
  chain_run(&chain, 2000u);
  int32_t duty_before = abs(chain.motor.applied_ticks);
  double speed_before = chain.wheel_speed;
  chain.max_applied = 0;
  double max_speed = 0.0;
  uint32_t left = 0u;
  for (uint32_t t = 0u; t < 1000u; t++)
  {
    chain_step(&chain, (t == 0u) ? 3 : 0);
    if (chain.wheel_speed > max_speed) { max_speed = chain.wheel_speed; }
    if (left == 0u && chain.motor.state != MOTOR_STATE_RUNNING) { left = t + 1u; }
  }
  printf("  e 3 under v 50: duty %d -> max %d ticks, wheel %.0f -> max %.0f units, latched after %u ms\n",
         (int)duty_before, (int)chain.max_applied, speed_before * 20.0, max_speed * 20.0, (unsigned)left);
  CHECK(chain.max_applied > duty_before + 100, "the PI sees 0 and raises the duty");
  CHECK(max_speed * 20.0 <= 96.0, "the wheel speeds up, but no further than the 30 %% cap allows (%.0f units)",
        max_speed * 20.0);
  CHECK(left >= STALL_HOLD_MS && left <= STALL_HOLD_MS + SPEED_RING_SIZE + 2u &&
        chain.stall.stall_count == 1u && chain.speed.enc_bad_count == 0u,
        "then the stall check latches (%u ms), stall_n 1", (unsigned)left);
}

static void test_three_e3_within_60_s_give_fault_7(void)
{
  Chain chain;
  chain_init(&chain);
  chain_send_m(&chain, 0);
  chain_run(&chain, 100u);
  for (int n = 1; n <= 3; n++)
  {
    chain_send_m(&chain, 0);   /* re-arm */
    chain_step(&chain, 0);
    chain_send_m(&chain, 20);
    chain_run(&chain, 2000u);
    (void)chain_fault(&chain, 3, 1000u);
    CHECK(chain.stall.stall_count == (uint32_t)n, "stall %d counted", n);
  }
  CHECK(chain.motor.state == MOTOR_STATE_FAULT && chain.motor.fault_reason == MOTOR_FAULT_REPEATED_STALL &&
        (int)chain.motor.fault_reason == 7, "3 stalls within 60 s: FAULT, mfault 7");
  chain_send_m(&chain, 0);
  chain_send_v(&chain, 0);
  chain_step(&chain, 0);
  CHECK(chain.motor.state == MOTOR_STATE_FAULT, "m 0 / v 0 don't clear it (reset only)");
}

int main(void)
{
  test_pinned_durations_and_codes();
  test_command_refusals();
  test_outage_lengths_are_exact();
  test_frozen_angle();
  test_e1_under_v_is_held();
  test_e2_under_v_latches_and_counts_enc_bad();
  test_e2_under_m_does_not_latch();
  test_e3_under_m_is_a_stall();
  test_e3_under_v_speeds_up_then_latches();
  test_three_e3_within_60_s_give_fault_7();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
