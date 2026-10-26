/*
 * Host-side test for Core/Src/speed_estimator.c. Not part of the firmware
 * build or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_speed_estimator.c Core/Src/speed_estimator.c -lm -o /tmp/test_speed_estimator && /tmp/test_speed_estimator
 * (-fno-sanitize-recover=all: undefined behaviour, e.g. a signed overflow in
 * the count difference, stops the test instead of only printing a warning.)
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "speed_estimator.h"

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

/* Deterministic pseudo-random numbers (xorshift32), so a failure reproduces. */
static uint32_t random_state = 0x0BADF00Du;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

/* A wheel turning at a constant speed, read every ms like main.c does: the
 * count is the whole counts so far, and stays put on a failed read. */
typedef struct
{
  SpeedEstimator estimator;
  uint32_t now_ms;
  double   position;        /* true position, counts */
  double   counts_per_ms;
  int32_t  count;           /* what the unwrapper holds */
  int32_t  count_base;
} Wheel;

static void wheel_init(Wheel *wheel, double counts_per_ms, uint32_t start_ms, int32_t count_base, double phase)
{
  SpeedEstimator_Init(&wheel->estimator);
  wheel->now_ms = start_ms;
  wheel->position = phase;
  wheel->counts_per_ms = counts_per_ms;
  wheel->count_base = count_base;
  wheel->count = count_base;
}

/* One ms: the wheel moves, the encoder is read (or not), the estimator updates. */
static void wheel_step(Wheel *wheel, bool read_ok, bool magnet, uint32_t step_ms)
{
  wheel->now_ms += step_ms;
  wheel->position += wheel->counts_per_ms * (double)step_ms;
  if (read_ok)
  {
    wheel->count = (int32_t)((uint32_t)wheel->count_base + (uint32_t)(int32_t)floor(wheel->position));
  }
  SpeedEstimatorInput input = { .now_ms = wheel->now_ms, .count = wheel->count, .read_ok = read_ok,
                                .magnet_detected = magnet };
  SpeedEstimator_Update(&wheel->estimator, &input);
}

static void wheel_run(Wheel *wheel, uint32_t ms)
{
  for (uint32_t i = 0u; i < ms; i++) { wheel_step(wheel, true, true, 1u); }
}

/* ---- Start-up ------------------------------------------------------------------- */

static void test_start_up_needs_a_full_window(void)
{
  Wheel wheel;
  wheel_init(&wheel, 1.0, 1000u, 0, 0.5);
  CHECK(!wheel.estimator.valid && wheel.estimator.reason == SPEED_ESTIMATE_STARTING, "invalid before any update");
  int wrong = 0;
  for (uint32_t update = 1u; update <= SPEED_RING_SIZE; update++)
  {
    wheel_step(&wheel, true, true, 1u);
    bool should_be_valid = (update == SPEED_RING_SIZE);
    if (wheel.estimator.valid != should_be_valid) { wrong++; }
    if (!should_be_valid && (wheel.estimator.reason != SPEED_ESTIMATE_STARTING ||
                             wheel.estimator.invalid_ms != update - 1u || wheel.estimator.speed_units != 0))
    {
      wrong++;
    }
  }
  CHECK(wrong == 0, "invalid (STARTING, invalid_ms counting from 0, speed 0) for 20 updates, valid at the 21st: %d wrong", wrong);
  CHECK(wheel.estimator.valid && wheel.estimator.speed_units == 20 && wheel.estimator.invalid_ms == 0u &&
        wheel.estimator.reason == SPEED_ESTIMATE_VALID, "21st update: valid, 20 units at 1 count/ms (got %d)",
        (int)wheel.estimator.speed_units);
}

/* ---- Resolution ------------------------------------------------------------------- */

static void test_resolution(void)
{
  /* 1 count/ms (~10 % duty on the lift): exactly 20 units, every ms, any phase. */
  int wrong = 0;
  for (int phase = 0; phase < 10; phase++)
  {
    Wheel wheel;
    wheel_init(&wheel, 1.0, 0u, 0, phase / 10.0);
    wheel_run(&wheel, 1000u);
    for (uint32_t i = 0u; i < 500u; i++)
    {
      wheel_step(&wheel, true, true, 1u);
      if (!wheel.estimator.valid || wheel.estimator.speed_units != 20) { wrong++; }
    }
  }
  CHECK(wrong == 0, "1 count/ms: %d estimates not exactly 20", wrong);

  /* 2.75 counts/ms (~20 %): 55 counts per 20 ms exactly. */
  wrong = 0;
  for (int phase = 0; phase < 10; phase++)
  {
    Wheel wheel;
    wheel_init(&wheel, 2.75, 0u, 0, phase / 10.0);
    wheel_run(&wheel, 100u);
    for (uint32_t i = 0u; i < 500u; i++)
    {
      wheel_step(&wheel, true, true, 1u);
      if (wheel.estimator.speed_units != 55) { wrong++; }
    }
  }
  CHECK(wrong == 0, "2.75 counts/ms: %d estimates not exactly 55", wrong);

  /* A speed that isn't a whole number of counts per window: each estimate is
   * one of the two neighbouring units (resolution 1 unit = 50 counts/s), and
   * the mean is the true speed. */
  Wheel wheel;
  wheel_init(&wheel, 2.71, 0u, 0, 0.3);
  wheel_run(&wheel, 100u);
  int64_t sum = 0;
  int outside = 0;
  for (uint32_t i = 0u; i < 10000u; i++)
  {
    wheel_step(&wheel, true, true, 1u);
    if (wheel.estimator.speed_units != 54 && wheel.estimator.speed_units != 55) { outside++; }
    sum += wheel.estimator.speed_units;
  }
  CHECK(outside == 0, "2.71 counts/ms: %d estimates not 54 or 55", outside);
  CHECK(fabs((double)sum / 10000.0 - 54.2) < 0.01, "2.71 counts/ms: mean %.3f, expected 54.2", (double)sum / 10000.0);

  /* Reverse: negative units. */
  wheel_init(&wheel, -2.75, 0u, 0, 0.0);
  wheel_run(&wheel, 100u);
  CHECK(wheel.estimator.valid && wheel.estimator.speed_units == -55, "reverse: -55 (got %d)", (int)wheel.estimator.speed_units);

  /* Stopped: 0, valid (a stopped wheel is a good reading, not a fault). */
  wheel_init(&wheel, 0.0, 0u, 1234, 0.0);
  wheel_run(&wheel, 100u);
  CHECK(wheel.estimator.valid && wheel.estimator.speed_units == 0, "stopped: 0 and valid");
}

/* ---- Window exactness ------------------------------------------------------------- */

static void test_window_is_exactly_count_now_minus_count_20_ms_ago(void)
{
  SpeedEstimator estimator;
  SpeedEstimator_Init(&estimator);
  enum { STEPS = 20000 };
  static int32_t history[STEPS];
  int32_t count = 0;
  int wrong = 0;
  for (uint32_t n = 0u; n < STEPS; n++)
  {
    count += (int32_t)(next_random() % 11u) - 5;   /* -5..+5 per ms: varying, never a jump */
    history[n] = count;
    SpeedEstimatorInput input = { .now_ms = 7u + n, .count = count, .read_ok = true, .magnet_detected = true };
    SpeedEstimator_Update(&estimator, &input);
    if (n >= SPEED_WINDOW_MS && (!estimator.valid || estimator.speed_units != count - history[n - SPEED_WINDOW_MS]))
    {
      wrong++;
    }
  }
  CHECK(wrong == 0, "%d estimates differ from count[n] - count[n-20]", wrong);
}

/* ---- 32-bit wrap ---------------------------------------------------------------------- */

static void test_count_and_time_wrap(void)
{
  /* Forward through INT32_MAX -> INT32_MIN, time through UINT32_MAX -> 0. */
  Wheel wheel;
  wheel_init(&wheel, 2.0, UINT32_MAX - 50u, INT32_MAX - 100, 0.0);
  int wrong = 0;
  for (uint32_t i = 0u; i < 200u; i++)
  {
    wheel_step(&wheel, true, true, 1u);
    if (i >= SPEED_WINDOW_MS && (!wheel.estimator.valid || wheel.estimator.speed_units != 40)) { wrong++; }
  }
  CHECK(wrong == 0 && wheel.count < 0, "forward across the count and tick wraps: %d estimates not 40 (count now %d)",
        wrong, (int)wheel.count);

  /* Backward through INT32_MIN -> INT32_MAX. */
  wheel_init(&wheel, -2.0, 0u, INT32_MIN + 50, 0.0);
  wrong = 0;
  for (uint32_t i = 0u; i < 200u; i++)
  {
    wheel_step(&wheel, true, true, 1u);
    if (i >= SPEED_WINDOW_MS && (!wheel.estimator.valid || wheel.estimator.speed_units != -40)) { wrong++; }
  }
  CHECK(wrong == 0 && wheel.count > 0, "backward across the count wrap: %d estimates not -40", wrong);
}

/* ---- Invalid rules: entering and leaving ------------------------------------------------ */

/* After a valid run, one bad update at time t0 (made by bad_step); checks
 * invalid with the reason from t0 through t0 + 20 updates (the bad one and
 * the next 20 good ones), invalid_ms counting, and valid again at the 21st. */
typedef void (*BadStep)(Wheel *wheel);

static void check_one_bad_update(const char *what, BadStep bad_step, SpeedEstimateReason reason)
{
  Wheel wheel;
  wheel_init(&wheel, 1.5, 500u, 0, 0.2);
  wheel_run(&wheel, 200u);
  CHECK(wheel.estimator.valid, "%s: setup valid", what);
  uint32_t bad_before = wheel.estimator.bad_update_count;

  bad_step(&wheel);
  uint32_t t0 = wheel.now_ms;
  CHECK(!wheel.estimator.valid && wheel.estimator.reason == reason && wheel.estimator.invalid_ms == 0u &&
        wheel.estimator.bad_update_count == bad_before + 1u,
        "%s: invalid at once with reason %d (got valid %d reason %d invalid_ms %u)", what, (int)reason,
        (int)wheel.estimator.valid, (int)wheel.estimator.reason, (unsigned)wheel.estimator.invalid_ms);

  int wrong = 0;
  for (uint32_t k = 1u; k <= SPEED_WINDOW_MS; k++)
  {
    wheel_step(&wheel, true, true, 1u);
    if (wheel.estimator.valid || wheel.estimator.reason != reason || wheel.estimator.invalid_ms != wheel.now_ms - t0)
    {
      wrong++;
    }
  }
  CHECK(wrong == 0, "%s: stays invalid with the same reason for the 20 updates after it (%d wrong)", what, wrong);
  wheel_step(&wheel, true, true, 1u);
  CHECK(wheel.estimator.valid && wheel.estimator.reason == SPEED_ESTIMATE_VALID && wheel.estimator.invalid_ms == 0u,
        "%s: valid again 21 updates after the bad one (valid %d, reason %d)", what, (int)wheel.estimator.valid,
        (int)wheel.estimator.reason);
}

static void bad_read(Wheel *wheel)        { wheel_step(wheel, false, true, 1u); }
static void bad_magnet(Wheel *wheel)      { wheel_step(wheel, true, false, 1u); }
static void bad_skip_one_ms(Wheel *wheel) { wheel_step(wheel, true, true, 2u); }
static void bad_resync(Wheel *wheel)      { wheel_step(wheel, true, true, 7u); }
static void bad_same_ms(Wheel *wheel)     { wheel_step(wheel, true, true, 0u); }
static void bad_jump_up(Wheel *wheel)     { wheel->count_base += 21; wheel_step(wheel, true, true, 1u); }
static void bad_jump_down(Wheel *wheel)   { wheel->count_base -= 23; wheel_step(wheel, true, true, 1u); }

static void test_each_rule_enters_and_leaves(void)
{
  check_one_bad_update("failed read", bad_read, SPEED_ESTIMATE_READ_FAILED);
  check_one_bad_update("MD = 0", bad_magnet, SPEED_ESTIMATE_NO_MAGNET);
  check_one_bad_update("2 ms step", bad_skip_one_ms, SPEED_ESTIMATE_TIME_STEP);
  check_one_bad_update("resync (7 ms)", bad_resync, SPEED_ESTIMATE_TIME_STEP);
  check_one_bad_update("0 ms step (catch-up)", bad_same_ms, SPEED_ESTIMATE_TIME_STEP);
  check_one_bad_update("jump up", bad_jump_up, SPEED_ESTIMATE_JUMP);
  check_one_bad_update("jump down", bad_jump_down, SPEED_ESTIMATE_JUMP);
}

static void test_jump_threshold_is_exclusive(void)
{
  /* Stopped wheel, then the count moves by exactly +-20 in one ms: allowed;
   * by 21: a jump. */
  static const struct { int32_t delta; bool jump; } cases[] = {
    { 20, false }, { -20, false }, { 21, true }, { -21, true }, { 2000, true },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    Wheel wheel;
    wheel_init(&wheel, 0.0, 0u, 100, 0.0);
    wheel_run(&wheel, 50u);
    wheel.count_base += cases[i].delta;
    wheel_step(&wheel, true, true, 1u);
    bool jumped = (!wheel.estimator.valid && wheel.estimator.reason == SPEED_ESTIMATE_JUMP);
    CHECK(jumped == cases[i].jump, "delta %d in 1 ms: jump %d, expected %d", (int)cases[i].delta, (int)jumped,
          (int)cases[i].jump);
  }
}

static void test_magnet_lost_for_a_while(void)
{
  Wheel wheel;
  wheel_init(&wheel, 1.0, UINT32_MAX - 100u, 0, 0.0);   /* invalid_ms across the tick wrap too */
  wheel_run(&wheel, 100u);
  uint32_t t0 = wheel.now_ms + 1u;
  int wrong = 0;
  for (uint32_t k = 0u; k < 300u; k++)
  {
    wheel_step(&wheel, true, false, 1u);
    if (wheel.estimator.valid || wheel.estimator.reason != SPEED_ESTIMATE_NO_MAGNET ||
        wheel.estimator.invalid_ms != wheel.now_ms - t0)
    {
      wrong++;
    }
  }
  CHECK(wrong == 0 && wheel.estimator.invalid_ms == 299u, "MD = 0 for 300 ms: invalid throughout, invalid_ms 0..299 (%d wrong)",
        wrong);
  /* Back: 20 more good updates still invalid, valid at the 21st. */
  wheel_run(&wheel, SPEED_WINDOW_MS);
  CHECK(!wheel.estimator.valid && wheel.estimator.invalid_ms == 319u, "20 ms after MD returns: still invalid");
  wheel_run(&wheel, 1u);
  CHECK(wheel.estimator.valid, "21 ms after: valid");
}

static void test_reason_priority_and_bad_during_start_up(void)
{
  Wheel wheel;
  wheel_init(&wheel, 1.0, 0u, 0, 0.0);
  wheel_run(&wheel, 100u);
  /* Everything wrong at once: the time step wins, then a failed read, then the magnet. */
  wheel.now_ms += 1u;   /* so that the next step is a 2 ms step */
  wheel_step(&wheel, false, false, 1u);
  CHECK(wheel.estimator.reason == SPEED_ESTIMATE_TIME_STEP, "time step first (got %d)", (int)wheel.estimator.reason);
  wheel_step(&wheel, false, false, 1u);
  CHECK(wheel.estimator.reason == SPEED_ESTIMATE_READ_FAILED, "then a failed read (got %d)", (int)wheel.estimator.reason);
  wheel_step(&wheel, true, false, 1u);
  CHECK(wheel.estimator.reason == SPEED_ESTIMATE_NO_MAGNET, "then no magnet (got %d)", (int)wheel.estimator.reason);

  /* A failed read during start-up: reported as such, not as STARTING, and
   * the window must still be clean before it is valid. */
  wheel_init(&wheel, 1.0, 0u, 0, 0.0);
  wheel_run(&wheel, 5u);
  wheel_step(&wheel, false, true, 1u);
  CHECK(!wheel.estimator.valid && wheel.estimator.reason == SPEED_ESTIMATE_READ_FAILED, "bad during start-up: its reason");
  wheel_run(&wheel, SPEED_WINDOW_MS);
  CHECK(!wheel.estimator.valid, "21 updates in total but the failed read is still in the window");
  wheel_run(&wheel, 1u);
  CHECK(wheel.estimator.valid, "valid once the window is clean");
}

static void test_speed_still_shown_while_invalid(void)
{
  Wheel wheel;
  wheel_init(&wheel, 2.0, 0u, 0, 0.0);
  wheel_run(&wheel, 100u);
  wheel.count_base += 500;
  wheel_step(&wheel, true, true, 1u);
  CHECK(!wheel.estimator.valid && wheel.estimator.speed_units == 540,
        "a jump: invalid, and the raw window difference stays visible for diagnostics (got %d)",
        (int)wheel.estimator.speed_units);
}

/* ---- A random hour against a reference model -------------------------------------------- */

static void test_random_hour_against_a_model(void)
{
  /* Reference: keeps every update and decides validity from scratch. */
  enum { HOUR = 3600000 };
  SpeedEstimator estimator;
  SpeedEstimator_Init(&estimator);
  static int32_t counts[SPEED_RING_SIZE];
  static bool good[SPEED_RING_SIZE];
  uint32_t n = 0u, now = 123u;
  int32_t count = 0;
  int wrong_valid = 0, wrong_speed = 0;
  uint32_t previous_now = 0u;
  int32_t previous_count = 0;
  for (uint32_t i = 0u; i < HOUR; i++)
  {
    uint32_t dice = next_random() % 10000u;
    uint32_t step = (dice < 3u) ? (dice + 1u) * 2u % 7u : 1u;          /* rare 0/2/4/6 ms steps */
    now += step;
    bool read_ok = (next_random() % 5000u) != 0u;
    bool magnet = (next_random() % 20000u) != 0u;
    if (read_ok)
    {
      count += ((next_random() % 3000u) == 0u) ? 40 : (int32_t)(next_random() % 9u) - 4;   /* rare glitch */
    }
    SpeedEstimatorInput input = { .now_ms = now, .count = count, .read_ok = read_ok, .magnet_detected = magnet };
    SpeedEstimator_Update(&estimator, &input);

    int32_t delta = count - previous_count;
    bool this_good = read_ok && magnet && (n == 0u || (now - previous_now == 1u && delta <= 20 && delta >= -20));
    counts[n % SPEED_RING_SIZE] = count;
    good[n % SPEED_RING_SIZE] = this_good;
    n++;
    bool model_valid = (n >= SPEED_RING_SIZE);
    for (uint32_t k = 0u; k < SPEED_RING_SIZE && model_valid; k++) { model_valid = good[k]; }
    if (estimator.valid != model_valid) { wrong_valid++; }
    if (n >= SPEED_RING_SIZE && estimator.speed_units != count - counts[n % SPEED_RING_SIZE]) { wrong_speed++; }
    previous_now = now;
    previous_count = count;
  }
  CHECK(wrong_valid == 0, "validity differed from the model %d times", wrong_valid);
  CHECK(wrong_speed == 0, "speed differed from the model %d times", wrong_speed);
  CHECK(estimator.bad_update_count > 1000u, "the hour had plenty of bad updates (%u)", (unsigned)estimator.bad_update_count);
}

static void test_constants(void)
{
  CHECK(SPEED_WINDOW_MS == 20u && SPEED_RING_SIZE == 21u, "20 ms window: 1 unit = 1 count per 20 ms = 50 counts/s");
  CHECK(SPEED_MAX_DELTA_COUNTS == 20, "a per-ms delta above 20 counts is a glitch");
}

int main(void)
{
  test_constants();
  test_start_up_needs_a_full_window();
  test_resolution();
  test_window_is_exactly_count_now_minus_count_20_ms_ago();
  test_count_and_time_wrap();
  test_each_rule_enters_and_leaves();
  test_jump_threshold_is_exclusive();
  test_magnet_lost_for_a_while();
  test_reason_priority_and_bad_during_start_up();
  test_speed_still_shown_while_invalid();
  test_random_hour_against_a_model();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
