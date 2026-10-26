/*
 * Host-side test for Core/Src/angle_correction.c, including the whole path
 * it will feed: corrected angle -> a second AS5600_Unwrapper (the real one,
 * Core/Src/as5600.c, compiled against tests/stubs/stm32l4xx_hal.h) -> the
 * speed estimator. Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -Itests/stubs -ICore/Inc tests/test_angle_correction.c Core/Src/angle_correction.c Core/Src/as5600.c Core/Src/speed_estimator.c -lm -o /tmp/test_angle_correction && /tmp/test_angle_correction
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "angle_correction.h"
#include "as5600.h"
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

/* as5600.c's register reads need this; the unwrapper never calls it. */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize; (void)pData; (void)Size; (void)Timeout;
  return HAL_ERROR;
}

static const double PI = 3.14159265358979323846;
#define A_COUNTS   (ANGLE_H4_AMPLITUDE_Q4 / 16.0)

/* The model the firmware approximates, in double precision. */
static double eps_exact(double raw)
{
  return A_COUNTS * sin(2.0 * PI * (raw + ANGLE_H4_PHASE_COUNTS) / 1024.0);
}

static int32_t modulo(int32_t value, int32_t m)
{
  int32_t r = value % m;
  return (r < 0) ? r + m : r;
}

/* ---- The constants ------------------------------------------------------------------ */

static void test_pinned_constants(void)
{
  /* Pinned on purpose: fitted from cap_m30_c2_1.log (2026-10-15) for the
   * board's current seating; change only with a re-fit (see the header). */
  CHECK(ANGLE_H4_AMPLITUDE_Q4 == 703 && ANGLE_H4_PHASE_COUNTS == -9, "A = 703/16 counts, r0 = -9 counts");
  CHECK(ANGLE_COUNTS_PER_REV == 4096u && ANGLE_H4_PERIOD == 1024u, "4096 counts per turn, 4 periods");
}

/* ---- Sine ---------------------------------------------------------------------------- */

static void test_sine_matches_the_math_over_a_whole_period(void)
{
  int bad = 0;
  for (uint32_t k = 0u; k < 1024u; k++)
  {
    double exact = 32767.0 * sin(2.0 * PI * k / 1024.0);
    if (fabs((double)AngleCorrection_SineQ15(k) - exact) > 0.5 + 1e-9) { bad++; }
  }
  CHECK(bad == 0, "%d of 1024 sine values more than 0.5 LSB from 32767 sin", bad);
  CHECK(AngleCorrection_SineQ15(0u) == 0 && AngleCorrection_SineQ15(256u) == 32767 &&
        AngleCorrection_SineQ15(512u) == 0 && AngleCorrection_SineQ15(768u) == -32767, "0, 90, 180, 270 degrees");
  CHECK(AngleCorrection_SineQ15(1024u + 100u) == AngleCorrection_SineQ15(100u), "wraps every 1024");
}

/* ---- The correction, all 4096 raw values -------------------------------------------------- */

static void test_every_raw_value(void)
{
  int far = 0, inconsistent = 0, out_of_range = 0, off_model = 0, ties = 0;
  for (uint32_t raw = 0u; raw < 4096u; raw++)
  {
    double exact = eps_exact((double)raw);
    int32_t eps = AngleCorrection_ErrorCounts((uint16_t)raw);
    /* Within half a count of the model: rounded correctly, except where the
     * model is within a hair of .5 and the table's +-0.5 LSB decides. */
    if (fabs((double)eps - exact) > 0.51) { far++; }
    if (eps != (int32_t)lround(exact)) { ties++; }
    uint16_t corrected = AngleCorrection_Apply((uint16_t)raw);
    if (corrected != (uint16_t)modulo((int32_t)raw - eps, 4096)) { inconsistent++; }
    double model = (double)raw - exact;                     /* corrected angle, exact */
    double distance = fmod(fabs((double)corrected - model) + 2048.0, 4096.0) - 2048.0;
    if (fabs(distance) > 0.51) { off_model++; }
    if (corrected > 4095u) { out_of_range++; }
  }
  CHECK(far == 0, "%d raw values with eps more than half a count from the model", far);
  CHECK(ties <= 32, "rounding differs from the model only at near-ties (%d of 4096)", ties);
  CHECK(inconsistent == 0, "%d corrected angles differ from (raw - eps) mod 4096", inconsistent);
  CHECK(off_model == 0, "%d corrected angles more than half a count from raw - eps_exact (mod 4096)", off_model);
  CHECK(out_of_range == 0, "corrected angle always 0..4095");
}

static void test_monotonic_zero_mean_periodic_and_peak(void)
{
  int backwards = 0, too_big = 0;
  for (uint32_t raw = 0u; raw < 4096u; raw++)
  {
    int32_t step = modulo((int32_t)AngleCorrection_Apply((uint16_t)((raw + 1u) % 4096u)) -
                          (int32_t)AngleCorrection_Apply((uint16_t)raw), 4096);
    if (step > 2048) { backwards++; }
    else if (step > 2) { too_big++; }
  }
  CHECK(backwards == 0 && too_big == 0, "each raw step is a corrected step of 0..2 counts (%d backwards, %d too big)",
        backwards, too_big);

  int64_t sum = 0;
  int not_periodic = 0, not_odd = 0;
  for (uint32_t raw = 0u; raw < 4096u; raw++)
  {
    int32_t eps = AngleCorrection_ErrorCounts((uint16_t)raw);
    sum += eps;
    if (eps != AngleCorrection_ErrorCounts((uint16_t)((raw + 1024u) % 4096u))) { not_periodic++; }
    if (eps != -AngleCorrection_ErrorCounts((uint16_t)((raw + 512u) % 4096u))) { not_odd++; }
  }
  CHECK(sum == 0, "eps sums to exactly 0 over a revolution (got %" PRId64 "): full-turn distances unchanged", sum);
  CHECK(not_periodic == 0, "period 1024: %d differ", not_periodic);
  CHECK(not_odd == 0, "half a period later it is exactly the opposite: %d differ", not_odd);

  /* sin peaks where raw - 9 = 256, i.e. raw 265: the rounded maximum (44)
   * is a plateau centred there. */
  int32_t best = -1000;
  uint32_t first = 0u, last = 0u;
  for (uint32_t raw = 0u; raw < 1024u; raw++)
  {
    int32_t eps = AngleCorrection_ErrorCounts((uint16_t)raw);
    if (eps > best) { best = eps; first = raw; last = raw; }
    else if (eps == best) { last = raw; }
  }
  CHECK(best == 44 && abs((int)(first + last) - 2 * 265) <= 2, "maximum 44 counts, plateau %u..%u centred on raw 265",
        (unsigned)first, (unsigned)last);
}

/* ---- End to end: sensor with the error -> correction -> unwrapper -> estimator ------------ */

typedef struct
{
  double mean;
  double std_percent;
  int    valid_samples;
} SpeedStats;

static void stats_add(double *sum, double *sum_sq, int *n, double value)
{
  *sum += value;
  *sum_sq += value * value;
  (*n)++;
}

/* A wheel turning at counts_per_ms (true), read every ms by a sensor whose
 * angle carries eps: measured m solves m - eps(m) = true, the exact inverse
 * of the correction. Raw and corrected paths run side by side. */
static void run_wheel(double counts_per_ms, uint32_t ms, SpeedStats *raw_stats, SpeedStats *corrected_stats,
                      int32_t *max_count_gap, double *mean_difference)
{
  AS5600_Unwrapper raw_unwrapper, corrected_unwrapper;
  AS5600_UnwrapperInit(&raw_unwrapper);
  AS5600_UnwrapperInit(&corrected_unwrapper);
  SpeedEstimator raw_estimator, corrected_estimator;
  SpeedEstimator_Init(&raw_estimator);
  SpeedEstimator_Init(&corrected_estimator);

  double raw_sum = 0, raw_sq = 0, cor_sum = 0, cor_sq = 0;
  int raw_n = 0, cor_n = 0;
  int32_t gap_min = INT32_MAX, gap_max = INT32_MIN;
  double true_start = 1000.3;
  for (uint32_t t = 0u; t < ms; t++)
  {
    double truth = true_start + counts_per_ms * t;
    double m = truth;
    for (int i = 0; i < 60; i++) { m = truth + eps_exact(m); }
    uint16_t raw = (uint16_t)modulo((int32_t)floor(m), 4096);

    AS5600_UnwrapperUpdate(&raw_unwrapper, raw);
    AS5600_UnwrapperUpdate(&corrected_unwrapper, AngleCorrection_Apply(raw));
    SpeedEstimatorInput raw_input = { .now_ms = t, .count = raw_unwrapper.continuous_count, .read_ok = true,
                                      .magnet_detected = true };
    SpeedEstimatorInput cor_input = { .now_ms = t, .count = corrected_unwrapper.continuous_count, .read_ok = true,
                                      .magnet_detected = true };
    SpeedEstimator_Update(&raw_estimator, &raw_input);
    SpeedEstimator_Update(&corrected_estimator, &cor_input);
    if (raw_estimator.valid) { stats_add(&raw_sum, &raw_sq, &raw_n, raw_estimator.speed_units); }
    if (corrected_estimator.valid) { stats_add(&cor_sum, &cor_sq, &cor_n, corrected_estimator.speed_units); }
    int32_t gap = corrected_unwrapper.continuous_count - raw_unwrapper.continuous_count;
    if (gap < gap_min) { gap_min = gap; }
    if (gap > gap_max) { gap_max = gap; }
  }
  raw_stats->mean = raw_sum / raw_n;
  raw_stats->std_percent = 100.0 * sqrt(raw_sq / raw_n - raw_stats->mean * raw_stats->mean) / fabs(raw_stats->mean);
  raw_stats->valid_samples = raw_n;
  corrected_stats->mean = cor_sum / cor_n;
  corrected_stats->std_percent = 100.0 * sqrt(cor_sq / cor_n - corrected_stats->mean * corrected_stats->mean) /
                                 fabs(corrected_stats->mean);
  corrected_stats->valid_samples = cor_n;
  *max_count_gap = gap_max - gap_min;
  *mean_difference = corrected_stats->mean - raw_stats->mean;
}

static void test_end_to_end_through_a_second_unwrapper_and_the_estimator(void)
{
  static const struct { double speed; double raw_std_min; double corrected_std_max; } cases[] = {
    { 4.8, 15.0, 1.5 },    /* ~30 % duty on the lift: 96 units */
    { 2.75, 15.0, 2.5 },   /* ~20 %: 55 units */
    { 1.0, 15.0, 6.0 },    /* ~10 %: 20 units; +-1 unit quantisation is 5 % */
    { -2.75, 15.0, 2.5 },  /* reverse */
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    SpeedStats raw, corrected;
    int32_t gap;
    double mean_difference;
    /* 10 s, a whole number of revolutions or more at every speed. */
    run_wheel(cases[i].speed, 10000u, &raw, &corrected, &gap, &mean_difference);
    CHECK(raw.std_percent > cases[i].raw_std_min, "%.2f counts/ms: raw estimate ripples (std %.1f %%)",
          cases[i].speed, raw.std_percent);
    CHECK(corrected.std_percent < cases[i].corrected_std_max,
          "%.2f counts/ms: corrected estimate std %.1f %% (limit %.1f, raw %.1f)", cases[i].speed,
          corrected.std_percent, cases[i].corrected_std_max, raw.std_percent);
    CHECK(corrected.valid_samples == raw.valid_samples && corrected.valid_samples > 9900,
          "%.2f counts/ms: the corrected path is valid as often as the raw one", cases[i].speed);
    /* The two counts never drift apart: they differ by eps at each end only. */
    CHECK(gap <= 2 * 44 + 2, "%.2f counts/ms: corrected and raw counts stay within 2A (spread %d)", cases[i].speed,
          (int)gap);
    CHECK(fabs(mean_difference) < 0.002 * fabs(raw.mean) + 0.05, "%.2f counts/ms: same mean speed (%.3f vs %.3f units)",
          cases[i].speed, corrected.mean, raw.mean);
  }
}

int main(void)
{
  test_pinned_constants();
  test_sine_matches_the_math_over_a_whole_period();
  test_every_raw_value();
  test_monotonic_zero_mean_periodic_and_peak();
  test_end_to_end_through_a_second_unwrapper_and_the_estimator();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
