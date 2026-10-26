/*
 * Host-side test for Core/Src/loop_schedule.c. Not part of the firmware
 * build or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_loop_schedule.c Core/Src/loop_schedule.c -o /tmp/test_loop_schedule && /tmp/test_loop_schedule
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loop_schedule.h"

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

/* The loop's schedules (main.c): report every 100 ms at phase 0, magnet
 * health every 100 ms at phase 50, heartbeat ticks every 100 ms. */
#define REPORT_PERIOD_MS   100u
#define HEALTH_PHASE_MS    50u

/* ---- Normal 1 ms steps: exactly the old "t % period == phase" ---------------- */

static void test_one_ms_steps_fire_exactly_on_the_marks(void)
{
  const uint32_t starts[] = { 1u, 37u, 99u, 100u, 101u, 12345u, 4000000000u };
  const uint32_t phases[] = { 0u, HEALTH_PHASE_MS, 99u };
  for (size_t s = 0; s < sizeof starts / sizeof starts[0]; s++)
  {
    for (size_t p = 0; p < sizeof phases / sizeof phases[0]; p++)
    {
      PeriodicSchedule schedule;
      PeriodicSchedule_Init(&schedule, REPORT_PERIOD_MS, phases[p], starts[s]);
      int mismatches = 0;
      for (uint32_t t = starts[s]; t != starts[s] + 10000u; t++)
      {
        bool due = PeriodicSchedule_IsDue(&schedule, t);
        bool old_rule = (t % REPORT_PERIOD_MS) == phases[p];
        if (due != old_rule) { mismatches++; }
      }
      CHECK(mismatches == 0, "start %" PRIu32 " phase %" PRIu32 ": %d ms differ from t %% 100 == phase",
            starts[s], phases[p], mismatches);
    }
  }
}

static void test_heartbeat_period_500_still_lands_on_500_ms_marks(void)
{
  PeriodicSchedule schedule;
  PeriodicSchedule_Init(&schedule, 500u, 0u, 3u);
  uint32_t first = 0;
  int count = 0;
  for (uint32_t t = 3u; t < 10003u; t++)
  {
    if (PeriodicSchedule_IsDue(&schedule, t))
    {
      CHECK(t % 500u == 0u, "fired at %" PRIu32, t);
      if (count == 0) { first = t; }
      count++;
    }
  }
  CHECK(first == 500u && count == 20, "first %" PRIu32 ", count %d", first, count);
}

static void test_init_on_a_mark_fires_at_once(void)
{
  PeriodicSchedule schedule;
  PeriodicSchedule_Init(&schedule, REPORT_PERIOD_MS, 0u, 200u);
  CHECK(PeriodicSchedule_IsDue(&schedule, 200u), "due at the start mark");
  CHECK(!PeriodicSchedule_IsDue(&schedule, 200u), "only once per mark");
  CHECK(!PeriodicSchedule_IsDue(&schedule, 299u), "not before the next mark");
  CHECK(PeriodicSchedule_IsDue(&schedule, 300u), "due at the next mark");
}

/* ---- Long steps: the stuck-bus case -------------------------------------- */

/* Steps of step_ms from start: counts firings, the largest and smallest
 * interval between them, and checks it never fires twice at one time. */
static void run_steps(uint32_t period, uint32_t phase, uint32_t start, uint32_t step_ms, uint32_t span_ms,
                      int *count, uint32_t *largest_interval, uint32_t *smallest_interval)
{
  PeriodicSchedule schedule;
  PeriodicSchedule_Init(&schedule, period, phase, start);
  *count = 0;
  *largest_interval = 0;
  *smallest_interval = UINT32_MAX;
  uint32_t previous = 0;
  for (uint32_t elapsed = 0; elapsed < span_ms; elapsed += step_ms)
  {
    uint32_t t = start + elapsed;
    if (PeriodicSchedule_IsDue(&schedule, t))
    {
      if (*count > 0)
      {
        uint32_t interval = t - previous;
        if (interval > *largest_interval) { *largest_interval = interval; }
        if (interval < *smallest_interval) { *smallest_interval = interval; }
      }
      previous = t;
      (*count)++;
      CHECK(!PeriodicSchedule_IsDue(&schedule, t), "fired twice at %" PRIu32, t);
    }
  }
}

static void test_52_ms_steps_still_fire_about_every_100_ms(void)
{
  /* Every starting residue: with "%" three in four of them never fired. */
  for (uint32_t start = 1000u; start < 1100u; start++)
  {
    int count;
    uint32_t largest, smallest;
    run_steps(REPORT_PERIOD_MS, 0u, start, 52u, 60000u, &count, &largest, &smallest);
    /* 60 s: one per 100 ms mark passed, so 600 (+-1 at the ends). */
    CHECK(count >= 599 && count <= 601, "start %" PRIu32 ": %d reports in 60 s", start, count);
    /* Two 52 ms steps = 104 ms between reports; now and then one step, 52 ms,
     * when the 4 ms a pair overshoots by has added up to a whole step. */
    CHECK(largest == 104u && smallest == 52u, "start %" PRIu32 ": intervals %" PRIu32 "..%" PRIu32,
          start, smallest, largest);

    run_steps(REPORT_PERIOD_MS, HEALTH_PHASE_MS, start, 52u, 60000u, &count, &largest, &smallest);
    CHECK(count >= 599 && count <= 601 && largest == 104u, "health, start %" PRIu32 ": %d, max %" PRIu32,
          start, count, largest);
  }
}

static void test_any_step_size_fires_once_per_mark_passed(void)
{
  for (uint32_t step = 1u; step <= 250u; step++)
  {
    int count;
    uint32_t largest, smallest;
    run_steps(REPORT_PERIOD_MS, 0u, 777u, step, 100000u, &count, &largest, &smallest);
    /* Never late by more than one step, never early. */
    uint32_t bound = (step <= REPORT_PERIOD_MS) ? REPORT_PERIOD_MS + step - 1u : step;
    CHECK(largest <= bound, "step %" PRIu32 ": interval %" PRIu32 " > %" PRIu32, step, largest, bound);
    if (step <= REPORT_PERIOD_MS)
    {
      CHECK(count >= 999 && count <= 1001, "step %" PRIu32 ": %d firings in 100 s", step, count);
    }
    else
    {
      /* Longer steps than the period: once per step, no catch-up. */
      int steps = (int)((100000u + step - 1u) / step);
      CHECK(count == steps || count == steps - 1, "step %" PRIu32 ": %d firings, %d steps", step, count, steps);
    }
  }
}

static void test_a_big_jump_fires_once_not_a_burst(void)
{
  PeriodicSchedule schedule;
  PeriodicSchedule_Init(&schedule, REPORT_PERIOD_MS, 0u, 1000u);
  CHECK(PeriodicSchedule_IsDue(&schedule, 1000u), "due at 1000");
  /* Resync: the loop was stuck for 10 s and restarts from 11057. */
  CHECK(PeriodicSchedule_IsDue(&schedule, 11057u), "due once after the jump");
  int burst = 0;
  for (uint32_t t = 11057u; t < 11100u; t++)
  {
    if (PeriodicSchedule_IsDue(&schedule, t)) { burst++; }
  }
  CHECK(burst == 0, "%d catch-up firings after the jump", burst);
  CHECK(PeriodicSchedule_IsDue(&schedule, 11100u), "back on the marks at 11100");
}

/* ---- uint32_t wrap of the ms tick (after 49.7 days) ------------------------ */

static void test_wraparound(void)
{
  const uint32_t start = UINT32_MAX - 1000u;
  for (uint32_t step = 1u; step <= 52u; step += 51u)   /* 1 ms and 52 ms steps */
  {
    PeriodicSchedule schedule;
    PeriodicSchedule_Init(&schedule, REPORT_PERIOD_MS, 0u, start);
    uint32_t previous = 0;
    int count = 0;
    uint32_t largest = 0;
    int short_intervals = 0;
    for (uint32_t elapsed = 0; elapsed < 5000u; elapsed += step)
    {
      uint32_t t = start + elapsed;   /* wraps past 0 */
      if (PeriodicSchedule_IsDue(&schedule, t))
      {
        if (count > 0)
        {
          uint32_t interval = t - previous;   /* unsigned difference: correct across the wrap */
          if (interval > largest) { largest = interval; }
          if (interval < REPORT_PERIOD_MS) { short_intervals++; }
        }
        previous = t;
        count++;
      }
    }
    CHECK(count >= 49 && count <= 51, "step %" PRIu32 ": %d firings across the wrap", step, count);
    CHECK(largest <= REPORT_PERIOD_MS + step - 1u, "step %" PRIu32 ": interval %" PRIu32, step, largest);
    if (step == 1u)
    {
      /* 2^32 is not a multiple of 100: one 96 ms interval at the wrap, as with "%". */
      CHECK(short_intervals == 1, "%d short intervals at the wrap", short_intervals);
    }
  }
}

/* ---- Heartbeat --------------------------------------------------------------- */

static void test_heartbeat_mode_precedence(void)
{
  CHECK(Heartbeat_Mode(false, true) == HEARTBEAT_NORMAL, "normal");
  CHECK(Heartbeat_Mode(false, false) == HEARTBEAT_IMU_MISSING, "IMU missing");
  CHECK(Heartbeat_Mode(true, true) == HEARTBEAT_MOTOR_FAULT, "FAULT");
  CHECK(Heartbeat_Mode(true, false) == HEARTBEAT_MOTOR_FAULT, "FAULT wins over IMU missing");
}

static void pattern_of(HeartbeatMode mode, uint32_t first_tick, char *out)
{
  for (uint32_t i = 0; i < 10u; i++)
  {
    out[i] = Heartbeat_LedOn(mode, first_tick + i) ? '#' : '.';
  }
  out[10] = '\0';
}

static void test_heartbeat_patterns(void)
{
  char normal[11], imu[11], fault[11];
  pattern_of(HEARTBEAT_NORMAL, 0u, normal);
  pattern_of(HEARTBEAT_IMU_MISSING, 0u, imu);
  pattern_of(HEARTBEAT_MOTOR_FAULT, 0u, fault);
  CHECK(strcmp(normal, "#####.....") == 0, "normal %s", normal);   /* 500 on, 500 off: as the old toggle */
  CHECK(strcmp(imu, "#.#.......") == 0, "IMU missing %s", imu);
  CHECK(strcmp(fault, "#.#.#.#.#.") == 0, "FAULT %s", fault);     /* 100 on, 100 off: as the old toggle */

  /* Every pattern repeats every 10 ticks (1 s). (The tick counter wraps
   * after 13.6 years of 100 ms ticks; not worth handling.) */
  for (int mode = HEARTBEAT_NORMAL; mode <= HEARTBEAT_MOTOR_FAULT; mode++)
  {
    char first[11], later[11];
    pattern_of((HeartbeatMode)mode, 0u, first);
    pattern_of((HeartbeatMode)mode, 1230u, later);
    CHECK(strcmp(first, later) == 0, "mode %d repeats every 10 ticks", mode);
  }
}

int main(void)
{
  test_one_ms_steps_fire_exactly_on_the_marks();
  test_heartbeat_period_500_still_lands_on_500_ms_marks();
  test_init_on_a_mark_fires_at_once();
  test_52_ms_steps_still_fire_about_every_100_ms();
  test_any_step_size_fires_once_per_mark_passed();
  test_a_big_jump_fires_once_not_a_burst();
  test_wraparound();
  test_heartbeat_mode_precedence();
  test_heartbeat_patterns();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
