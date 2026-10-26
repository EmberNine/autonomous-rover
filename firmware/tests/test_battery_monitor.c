/*
 * Host-side test for Core/Src/battery_monitor.c. Not part of the firmware
 * build or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_battery_monitor.c Core/Src/battery_monitor.c -lm -o /tmp/test_battery_monitor && /tmp/test_battery_monitor
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery_monitor.h"

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
static uint32_t random_state = 0x2468ACE1u;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

#define GOOD_CAL  1655u   /* a typical VREFINT_CAL (1.212 V at 3.0 V) */
#define FAR_MS    60000u  /* "never", in these tests */

/* Feeds battery_mv once per ms over [from_ms, to_ms) (wrap-safe) and
 * returns the first ms at which the state became `wanted`, or to_ms if it
 * never did (callers pick ranges where to_ms is not a real answer). */
static uint32_t feed_until_state(BatteryMonitor *monitor, uint32_t battery_mv, uint32_t from_ms, uint32_t to_ms,
                                 BatteryState wanted)
{
  for (uint32_t now_ms = from_ms; now_ms != to_ms; now_ms++)
  {
    BatteryMonitor_UpdateMv(monitor, battery_mv, false, now_ms);
    if (monitor->state == wanted) { return now_ms; }
  }
  return to_ms;
}

/* Feeds battery_mv once per ms over [from_ms, to_ms). */
static void feed(BatteryMonitor *monitor, uint32_t battery_mv, uint32_t from_ms, uint32_t to_ms)
{
  for (uint32_t now_ms = from_ms; now_ms != to_ms; now_ms++) { BatteryMonitor_UpdateMv(monitor, battery_mv, false, now_ms); }
}

/* Feeds battery_mv once per ms from from_ms; returns the first ms at which
 * the FILTERED voltage satisfies below ? (< threshold) : (>= threshold). */
static uint32_t feed_until_filtered(BatteryMonitor *monitor, uint32_t battery_mv, uint32_t from_ms, bool below,
                                    uint32_t threshold_mv)
{
  for (uint32_t now_ms = from_ms; now_ms != from_ms + FAR_MS; now_ms++)
  {
    BatteryMonitor_UpdateMv(monitor, battery_mv, false, now_ms);
    uint32_t filtered_mv = BatteryMonitor_FilteredMv(monitor);
    if (below ? filtered_mv < threshold_mv : filtered_mv >= threshold_mv) { return now_ms; }
  }
  return from_ms + FAR_MS;
}

/* A fresh monitor (as after a reset): its first sample seeds the filter, so
 * a constant input is "already there" for the threshold tests. */
static void fresh(BatteryMonitor *monitor)
{
  BatteryMonitor_Init(monitor, GOOD_CAL);
}

/* ---- The decided values ---------------------------------------------------------- */

/* Pinned on purpose: the tests below are written in terms of the constants,
 * so they would pass with any thresholds. These are the decisions (and the
 * 2026-10-14 divider measurement); change them here only together with the
 * README. */
static void test_decided_values(void)
{
  CHECK(BATTERY_WARN_MV == 7000u && BATTERY_WARN_CLEAR_MV == 7200u && BATTERY_CUTOFF_MV == 6600u,
        "warning 7.0 V, cleared at 7.2 V, cutoff 6.6 V");
  CHECK(BATTERY_LEVEL_HOLD_MS == 3000u, "warning, clear and cutoff held 3 s");
  CHECK(BATTERY_REMOVED_MV == 5000u && BATTERY_PRESENT_MV == 5500u && BATTERY_PRESENCE_HOLD_MS == 500u,
        "removed below 5.0 V, armed at 5.5 V, each held 0.5 s");
  CHECK(BATTERY_DIVIDER_NUM == 3102u && BATTERY_DIVIDER_DEN == 10000u, "divider 3102 / 10000");
  CHECK(VREFINT_CAL_VREF_MV == 3000u && ADC_FULL_SCALE == 4095u, "VREFINT_CAL taken at 3.0 V, 12 bits");
  CHECK(VDDA_MIN_MV == 3000u && VDDA_MAX_MV == 3600u && VREFINT_CAL_MIN == 1600u && VREFINT_CAL_MAX == 1700u,
        "plausibility windows");
  CHECK(BATTERY_FILTER_SHIFT == 6u && BATTERY_FILTER_FRACTION_BITS == 8u, "filter 1/64, 8 fraction bits");
  CHECK(BATTERY_MAX_MISSED_SAMPLES == 3u, "3 missed samples in a row are an ADC fault");
  CHECK(BATTERY_CUTOFF_TEST == 0, "BATTERY_CUTOFF_TEST is 0 (the bench cutoff test build is never committed)");
}

/* ---- Conversion ------------------------------------------------------------------ */

static void test_conversion(void)
{
  /* The factory point: VREFINT reads its calibration value at 3.0 V. */
  CHECK(Battery_VddaMv(GOOD_CAL, GOOD_CAL) == 3000u, "cal == raw -> 3000 mV, got %u",
        (unsigned)Battery_VddaMv(GOOD_CAL, GOOD_CAL));
  CHECK(Battery_VddaMv(GOOD_CAL, 1505u) == 3299u, "1655/1505 -> 3299 mV, got %u", (unsigned)Battery_VddaMv(GOOD_CAL, 1505u));
  CHECK(Battery_VddaMv(GOOD_CAL, 0u) == 0u, "raw 0 -> 0, no division by zero");

  /* The 2026-10-14 divider measurement: 2.457 V at the pin is 7.92 V. With
   * VDDA 3300 mV, 2.457 V reads 3049 counts (2457.07 mV) -> 7920.9 mV. */
  CHECK(Battery_PackMv(3300u, 3049u) == 7921u, "3300 mV, 3049 counts -> 7921 mV, got %u",
        (unsigned)Battery_PackMv(3300u, 3049u));
  /* The largest inputs don't overflow: 3600 mV at full scale -> 11605 mV. */
  CHECK(Battery_PackMv(VDDA_MAX_MV, ADC_FULL_SCALE) == 11605u, "full scale -> 11605 mV, got %u",
        (unsigned)Battery_PackMv(VDDA_MAX_MV, ADC_FULL_SCALE));
  CHECK(Battery_PackMv(3300u, 0u) == 0u, "0 counts -> 0 mV");

  /* Against floating point over the whole plausible range. */
  int vdda_wrong = 0;
  int pack_wrong = 0;
  for (uint32_t cal = VREFINT_CAL_MIN; cal <= VREFINT_CAL_MAX; cal += 7u)
  {
    for (uint32_t raw = 1300u; raw <= 1750u; raw += 3u)
    {
      if (Battery_VddaMv(cal, raw) != (uint32_t)lround(3000.0 * cal / raw)) { vdda_wrong++; }
    }
  }
  for (uint32_t vdda_mv = VDDA_MIN_MV; vdda_mv <= VDDA_MAX_MV; vdda_mv += 37u)
  {
    for (uint32_t raw = 0u; raw <= ADC_FULL_SCALE; raw += 13u)
    {
      double exact_mv = (double)vdda_mv * raw / 4095.0 / 0.3102;
      if (fabs((double)Battery_PackMv(vdda_mv, raw) - exact_mv) > 1.0) { pack_wrong++; }
    }
  }
  CHECK(vdda_wrong == 0, "%d VDDA values differ from round(3000 x cal / raw)", vdda_wrong);
  CHECK(pack_wrong == 0, "%d pack voltages more than 1 mV from vdda x raw / 4095 / 0.3102", pack_wrong);
}

/* ---- ADC plausibility ------------------------------------------------------------ */

static void test_calibration_word_plausibility(void)
{
  static const struct { uint16_t cal; bool fault; } cases[] = {
    { 0u, true }, { VREFINT_CAL_MIN - 1u, true }, { VREFINT_CAL_MIN, false }, { GOOD_CAL, false },
    { VREFINT_CAL_MAX, false }, { VREFINT_CAL_MAX + 1u, true }, { 0xFFFFu, true },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    BatteryMonitor monitor;
    BatteryMonitor_Init(&monitor, cases[i].cal);
    bool faulted = monitor.state == BATTERY_STATE_ADC_FAULT;
    CHECK(faulted == cases[i].fault && BatteryMonitor_MotorLockout(&monitor) == cases[i].fault,
          "VREFINT_CAL %u: fault %d, expected %d", (unsigned)cases[i].cal, (int)faulted, (int)cases[i].fault);
  }
}

static void test_sample_plausibility(void)
{
  /* With cal 1650, VDDA = 4,950,000 / raw: 1650 -> 3000 mV, 1375 -> 3600 mV. */
  static const struct { bool ok; uint32_t vrefint_raw; uint32_t battery_raw; bool fault; const char *what; } cases[] = {
    { true,  1500u, 2000u, false, "a good sample" },
    { false, 1500u, 2000u, true,  "conversion error" },
    { true,  0u,    2000u, true,  "VREFINT reads 0" },
    { true,  1650u, 2000u, false, "VDDA exactly 3000 mV" },
    { true,  1651u, 2000u, true,  "VDDA 2998 mV" },
    { true,  1375u, 2000u, false, "VDDA exactly 3600 mV" },
    { true,  1374u, 2000u, true,  "VDDA 3603 mV" },
    { true,  4096u, 2000u, true,  "VREFINT above 12 bits" },
    { true,  1500u, 4095u, false, "battery at full scale" },
    { true,  1500u, 4096u, true,  "battery above 12 bits" },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    BatteryMonitor monitor;
    BatteryMonitor_Init(&monitor, 1650u);
    BatteryMonitor_Update(&monitor, cases[i].ok, cases[i].vrefint_raw, cases[i].battery_raw, false, 100u);
    bool faulted = monitor.state == BATTERY_STATE_ADC_FAULT;
    CHECK(faulted == cases[i].fault && BatteryMonitor_MotorLockout(&monitor) == cases[i].fault,
          "%s: fault %d, expected %d", cases[i].what, (int)faulted, (int)cases[i].fault);
  }
}

static void test_adc_fault_latches_and_wins(void)
{
  BatteryMonitor monitor;
  BatteryMonitor_Init(&monitor, GOOD_CAL);
  /* Armed at about 8 V through the raw path: VDDA 3299 mV, 3080 counts. */
  for (uint32_t now_ms = 0u; now_ms < 1000u; now_ms++) { BatteryMonitor_Update(&monitor, true, 1505u, 3080u, false, now_ms); }
  CHECK(monitor.state == BATTERY_STATE_OK && monitor.vdda_mv == 3299u &&
        monitor.battery_mv == Battery_PackMv(3299u, 3080u),
        "raw path arms (state %d, vdda %u, battery %u)", (int)monitor.state, (unsigned)monitor.vdda_mv,
        (unsigned)monitor.battery_mv);

  BatteryMonitor_Update(&monitor, true, 1000u, 3080u, false, 1000u);  /* VDDA 4965 mV: implausible */
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT && BatteryMonitor_MotorLockout(&monitor), "one bad VREFINT -> fault");
  for (uint32_t now_ms = 1001u; now_ms < 1001u + FAR_MS; now_ms += 10u)
  {
    BatteryMonitor_Update(&monitor, true, 1505u, 3080u, false, now_ms);
  }
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT && BatteryMonitor_MotorLockout(&monitor),
        "fault stays latched through a minute of good samples (state %d)", (int)monitor.state);
  BatteryMonitor_UpdateMv(&monitor, 8000u, false, 70000u);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT, "UpdateMv can't clear the fault either");

  /* Fault wins over a latched cutoff. */
  fresh(&monitor);
  feed(&monitor, 6000u, 0u, 4000u);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF, "setup: cutoff (state %d)", (int)monitor.state);
  BatteryMonitor_Update(&monitor, false, 1505u, 3080u, false, 4000u);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT, "fault over cutoff (state %d)", (int)monitor.state);

  BatteryMonitor_Init(&monitor, GOOD_CAL);
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor), "reset clears it");
}

/* ---- Filter ---------------------------------------------------------------------------- */

static void test_filter(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 0u, "0 before the first sample");
  BatteryMonitor_UpdateMv(&monitor, 8000u, false, 0u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 8000u, "seeded with the first sample, got %u",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));
  BatteryMonitor_UpdateMv(&monitor, 0u, false, 1u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 7875u, "one step moves 1/64 of the way: 8000 -> 7875, got %u",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));
  BatteryMonitor_UpdateMv(&monitor, 8000u, false, 2u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 7877u, "and back up: 7875 -> 7876.95 -> 7877, got %u",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));

  /* Settles exactly on a constant input, from above and from below. */
  feed(&monitor, 6600u, 3u, 1000u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 6600u, "settles down to 6600, got %u",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));
  feed(&monitor, 7200u, 1000u, 2000u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) == 7200u, "settles up to 7200, got %u",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));

  /* A single spike can't move it past a threshold from far away. */
  fresh(&monitor);
  BatteryMonitor_UpdateMv(&monitor, 8000u, false, 0u);
  BatteryMonitor_UpdateMv(&monitor, 0u, false, 1u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) > BATTERY_CUTOFF_MV, "one 0 V sample from 8 V stays above cutoff");
}

/* ---- Presence -------------------------------------------------------------------------- */

static void test_no_pack_never_arms_or_locks_out(void)
{
  static const uint32_t levels[] = { 0u, 3300u, BATTERY_REMOVED_MV, BATTERY_PRESENT_MV - 1u };
  for (size_t i = 0; i < sizeof levels / sizeof levels[0]; i++)
  {
    BatteryMonitor monitor;
    fresh(&monitor);
    uint32_t armed_ms = feed_until_state(&monitor, levels[i], 0u, FAR_MS, BATTERY_STATE_OK);
    CHECK(armed_ms == FAR_MS && monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor),
          "%u mV for a minute: not present, no lockout (state %d)", (unsigned)levels[i], (int)monitor.state);
  }
}

static void test_arming_takes_exactly_the_presence_hold(void)
{
  static const uint32_t starts[] = { 0u, 12345u, UINT32_MAX - 200u };
  for (size_t i = 0; i < sizeof starts / sizeof starts[0]; i++)
  {
    BatteryMonitor monitor;
    fresh(&monitor);
    uint32_t armed_ms = feed_until_state(&monitor, BATTERY_PRESENT_MV, starts[i], starts[i] + 2000u, BATTERY_STATE_OK);
    CHECK(armed_ms == starts[i] + BATTERY_PRESENCE_HOLD_MS, "start %" PRIu32 ": armed after %" PRIu32 " ms, expected %u",
          starts[i], armed_ms - starts[i], BATTERY_PRESENCE_HOLD_MS);
  }
}

static void test_removal(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  CHECK(monitor.state == BATTERY_STATE_OK, "setup: armed");

  /* Exactly at the threshold is not "below": never removed. */
  feed(&monitor, BATTERY_REMOVED_MV, 1000u, 1000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_OK || monitor.state == BATTERY_STATE_CUTOFF,
        "at %u mV: still armed (state %d)", BATTERY_REMOVED_MV, (int)monitor.state);

  /* Unplugged: 0 V. Removed exactly the hold after the filter goes below,
   * and no cutoff even though it passed through 6.6 V on the way. */
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  uint32_t below_ms = feed_until_filtered(&monitor, 0u, 1000u, true, BATTERY_REMOVED_MV);
  uint32_t removed_ms = feed_until_state(&monitor, 0u, below_ms + 1u, below_ms + 2000u, BATTERY_STATE_NOT_PRESENT);
  CHECK(removed_ms == below_ms + BATTERY_PRESENCE_HOLD_MS, "removed %" PRIu32 " ms after going below, expected %u",
        removed_ms - below_ms, BATTERY_PRESENCE_HOLD_MS);
  feed(&monitor, 0u, removed_ms + 1u, removed_ms + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !monitor.cutoff_latched && !BatteryMonitor_MotorLockout(&monitor),
        "unplugging disarms, no cutoff (state %d)", (int)monitor.state);

  /* Removing a LOW pack clears the warning; a fresh pack arms as OK. */
  fresh(&monitor);
  feed(&monitor, 6900u, 0u, 5000u);
  CHECK(monitor.state == BATTERY_STATE_LOW, "setup: low (state %d)", (int)monitor.state);
  feed(&monitor, 0u, 5000u, 7000u);
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !monitor.low, "removed: not present, warning cleared");
  uint32_t armed_ms = feed_until_state(&monitor, 8400u, 7000u, 7000u + FAR_MS, BATTERY_STATE_OK);
  CHECK(armed_ms < 7000u + 2000u, "a full pack arms as OK (after %" PRIu32 " ms)", armed_ms - 7000u);
  feed(&monitor, 8400u, armed_ms + 1u, armed_ms + 10000u);
  CHECK(monitor.state == BATTERY_STATE_OK, "and stays OK (state %d)", (int)monitor.state);
}

/* Unplugs a pack that was at 8 V (feeding 0 V until removed); returns the ms
 * after the removal. On the way out the filter passed below 7.0 and 6.6 V,
 * so the warning and cutoff holds were running. */
static uint32_t unplug(BatteryMonitor *monitor, uint32_t from_ms)
{
  feed(monitor, 8000u, from_ms, from_ms + 1000u);
  uint32_t removed_ms = feed_until_state(monitor, 0u, from_ms + 1000u, from_ms + FAR_MS, BATTERY_STATE_NOT_PRESENT);
  CHECK(removed_ms < from_ms + 3000u && !monitor->cutoff_latched, "setup: unplugged without a cutoff");
  return removed_ms + 1u;
}

static void test_rearm_after_removal_starts_every_hold_again(void)
{
  /* A flat pack plugged in after a removal: armed the full presence hold
   * after the filter reaches 5.5 V, then LOW / CUTOFF the full level hold
   * after arming, not at once from holds left over from the last pack. */
  static const struct { uint32_t pack_mv; BatteryState after_hold; } packs[] = {
    { 6800u, BATTERY_STATE_LOW }, { 6000u, BATTERY_STATE_CUTOFF },
  };
  for (size_t i = 0; i < sizeof packs / sizeof packs[0]; i++)
  {
    BatteryMonitor monitor;
    fresh(&monitor);
    uint32_t t = unplug(&monitor, 0u);
    uint32_t reached_ms = feed_until_filtered(&monitor, packs[i].pack_mv, t, false, BATTERY_PRESENT_MV);
    uint32_t armed_ms = feed_until_state(&monitor, packs[i].pack_mv, reached_ms + 1u, reached_ms + 2000u,
                                         BATTERY_STATE_OK);
    CHECK(armed_ms == reached_ms + BATTERY_PRESENCE_HOLD_MS, "%u mV pack: armed %" PRIu32 " ms after reaching %u, expected %u",
          (unsigned)packs[i].pack_mv, armed_ms - reached_ms, BATTERY_PRESENT_MV, BATTERY_PRESENCE_HOLD_MS);
    uint32_t judged_ms = feed_until_state(&monitor, packs[i].pack_mv, armed_ms + 1u, armed_ms + 10000u,
                                          packs[i].after_hold);
    CHECK(judged_ms == armed_ms + BATTERY_LEVEL_HOLD_MS, "%u mV pack: state %d %" PRIu32 " ms after arming, expected %u",
          (unsigned)packs[i].pack_mv, (int)packs[i].after_hold, judged_ms - armed_ms, BATTERY_LEVEL_HOLD_MS);
  }

  /* Removed while hovering just under 5.0 V: even a full-scale sample
   * (11.6 V, the largest the ADC can report) can't lift the filter to 5.5 V
   * in one step (MAX_STEP_MV in battery_monitor.c), so the presence hold
   * restarts by itself; arming takes the full hold once it gets there. */
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  uint32_t removed_ms = feed_until_state(&monitor, BATTERY_REMOVED_MV - 1u, 1000u, 1000u + FAR_MS,
                                         BATTERY_STATE_NOT_PRESENT);
  BatteryMonitor_UpdateMv(&monitor, Battery_PackMv(VDDA_MAX_MV, ADC_FULL_SCALE), false, removed_ms + 1u);
  CHECK(BatteryMonitor_FilteredMv(&monitor) < BATTERY_PRESENT_MV, "one full-scale sample stays below 5.5 V (%u)",
        (unsigned)BatteryMonitor_FilteredMv(&monitor));
  uint32_t reached_ms = feed_until_filtered(&monitor, 8000u, removed_ms + 2u, false, BATTERY_PRESENT_MV);
  uint32_t armed_ms = feed_until_state(&monitor, 8000u, reached_ms + 1u, reached_ms + 2000u, BATTERY_STATE_OK);
  CHECK(armed_ms == reached_ms + BATTERY_PRESENCE_HOLD_MS, "re-armed %" PRIu32 " ms after reaching %u, expected %u",
        armed_ms - reached_ms, BATTERY_PRESENT_MV, BATTERY_PRESENCE_HOLD_MS);

  /* And a second removal takes the full hold too. */
  uint32_t below_ms = feed_until_filtered(&monitor, 0u, armed_ms + 1u, true, BATTERY_REMOVED_MV);
  removed_ms = feed_until_state(&monitor, 0u, below_ms + 1u, below_ms + 2000u, BATTERY_STATE_NOT_PRESENT);
  CHECK(removed_ms == below_ms + BATTERY_PRESENCE_HOLD_MS, "second removal %" PRIu32 " ms after going below, expected %u",
        removed_ms - below_ms, BATTERY_PRESENCE_HOLD_MS);
}

/* ---- Warning ----------------------------------------------------------------------------- */

static void test_warning_threshold_and_timing(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, BATTERY_WARN_MV, 0u, FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_OK, "exactly %u mV: no warning (state %d)", BATTERY_WARN_MV, (int)monitor.state);

  /* 1 mV below: armed at 500 ms (the warning hold starts there), LOW at
   * 500 + 3000, not a ms earlier. */
  fresh(&monitor);
  uint32_t low_ms = feed_until_state(&monitor, BATTERY_WARN_MV - 1u, 0u, FAR_MS, BATTERY_STATE_LOW);
  CHECK(low_ms == BATTERY_PRESENCE_HOLD_MS + BATTERY_LEVEL_HOLD_MS, "LOW at %" PRIu32 " ms, expected %u", low_ms,
        BATTERY_PRESENCE_HOLD_MS + BATTERY_LEVEL_HOLD_MS);
  CHECK(!BatteryMonitor_MotorLockout(&monitor), "a warning doesn't lock the motor out");
}

static void test_warning_clears_with_hysteresis(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 6900u, 0u, 4000u);
  CHECK(monitor.state == BATTERY_STATE_LOW, "setup: low");

  /* Back between the two thresholds: stays LOW. */
  feed(&monitor, 7100u, 4000u, 4000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_LOW, "7100 mV for a minute: still LOW (state %d)", (int)monitor.state);
  feed(&monitor, BATTERY_WARN_CLEAR_MV - 1u, 64000u, 64000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_LOW, "%u mV for a minute: still LOW (state %d)", BATTERY_WARN_CLEAR_MV - 1u,
        (int)monitor.state);

  /* At the clear threshold: OK exactly the hold after the filter reaches it. */
  uint32_t reached_ms = feed_until_filtered(&monitor, BATTERY_WARN_CLEAR_MV, 124000u, false, BATTERY_WARN_CLEAR_MV);
  uint32_t ok_ms = feed_until_state(&monitor, BATTERY_WARN_CLEAR_MV, reached_ms + 1u, reached_ms + 10000u,
                                    BATTERY_STATE_OK);
  CHECK(ok_ms == reached_ms + BATTERY_LEVEL_HOLD_MS, "cleared %" PRIu32 " ms after reaching %u, expected %u",
        ok_ms - reached_ms, BATTERY_WARN_CLEAR_MV, BATTERY_LEVEL_HOLD_MS);

  /* A second cycle takes the full holds again (no hold left running from
   * the first one). */
  uint32_t below_ms = feed_until_filtered(&monitor, 6900u, ok_ms + 1u, true, BATTERY_WARN_MV);
  uint32_t low_ms = feed_until_state(&monitor, 6900u, below_ms + 1u, below_ms + 10000u, BATTERY_STATE_LOW);
  CHECK(low_ms == below_ms + BATTERY_LEVEL_HOLD_MS, "warns again %" PRIu32 " ms after going below, expected %u",
        low_ms - below_ms, BATTERY_LEVEL_HOLD_MS);
  reached_ms = feed_until_filtered(&monitor, 7500u, low_ms + 1u, false, BATTERY_WARN_CLEAR_MV);
  ok_ms = feed_until_state(&monitor, 7500u, reached_ms + 1u, reached_ms + 10000u, BATTERY_STATE_OK);
  CHECK(ok_ms == reached_ms + BATTERY_LEVEL_HOLD_MS, "clears again %" PRIu32 " ms after reaching %u, expected %u",
        ok_ms - reached_ms, BATTERY_WARN_CLEAR_MV, BATTERY_LEVEL_HOLD_MS);
}

/* ---- Cutoff ------------------------------------------------------------------------------ */

static void test_cutoff_threshold_and_timing(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, BATTERY_CUTOFF_MV, 0u, FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_LOW && !BatteryMonitor_MotorLockout(&monitor),
        "exactly %u mV: LOW, no cutoff (state %d)", BATTERY_CUTOFF_MV, (int)monitor.state);

  static const uint32_t starts[] = { 0u, UINT32_MAX - 1000u };
  for (size_t i = 0; i < sizeof starts / sizeof starts[0]; i++)
  {
    fresh(&monitor);
    uint32_t cutoff_ms = feed_until_state(&monitor, BATTERY_CUTOFF_MV - 1u, starts[i], starts[i] + FAR_MS,
                                          BATTERY_STATE_CUTOFF);
    CHECK(cutoff_ms - starts[i] == BATTERY_PRESENCE_HOLD_MS + BATTERY_LEVEL_HOLD_MS,
          "start %" PRIu32 ": CUTOFF after %" PRIu32 " ms, expected %u", starts[i], cutoff_ms - starts[i],
          BATTERY_PRESENCE_HOLD_MS + BATTERY_LEVEL_HOLD_MS);
    CHECK(BatteryMonitor_MotorLockout(&monitor), "cutoff locks the motor out");
  }
}

static void test_cutoff_needs_an_unbroken_hold(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 6000u, 0u, 3400u);  /* armed at 500, 2900 ms below cutoff */
  CHECK(monitor.state != BATTERY_STATE_CUTOFF, "setup: not yet");
  feed(&monitor, 8400u, 3400u, 3420u);  /* 20 ms of recovery breaks the hold */
  CHECK(BatteryMonitor_FilteredMv(&monitor) >= BATTERY_CUTOFF_MV, "setup: filter back above cutoff");
  uint32_t below_ms = feed_until_filtered(&monitor, 6000u, 3420u, true, BATTERY_CUTOFF_MV);
  uint32_t cutoff_ms = feed_until_state(&monitor, 6000u, below_ms + 1u, below_ms + 10000u, BATTERY_STATE_CUTOFF);
  CHECK(cutoff_ms == below_ms + BATTERY_LEVEL_HOLD_MS, "cutoff %" PRIu32 " ms after going below again, expected %u",
        cutoff_ms - below_ms, BATTERY_LEVEL_HOLD_MS);
}

static void test_cutoff_latches_until_reset(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 6000u, 0u, 4000u);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF, "setup: cutoff (state %d)", (int)monitor.state);

  feed(&monitor, 8400u, 4000u, 4000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF && BatteryMonitor_MotorLockout(&monitor),
        "a recovered (resting) pack: still CUTOFF (state %d)", (int)monitor.state);
  feed(&monitor, 0u, 64000u, 64000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF && BatteryMonitor_MotorLockout(&monitor),
        "pack removed: still CUTOFF (state %d)", (int)monitor.state);
  feed(&monitor, 8400u, 124000u, 124000u + FAR_MS);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF && BatteryMonitor_MotorLockout(&monitor),
        "a fresh pack: still CUTOFF (state %d)", (int)monitor.state);

  BatteryMonitor_Init(&monitor, GOOD_CAL);
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor), "only a reset clears it");
}

/* ---- Motor-PWM ripple ---------------------------------------------------------------------- */

/* A5 has no RC filter, so the 20 kHz motor PWM ripple on VM can show up in
 * the samples. Since the carrier build VM has a 470 uF bulk capacitor beside
 * the TB6612 and the pack connects through a keyed connector, which keep it
 * small (bench 2026-10-14: no visible offset at m 20). The test keeps the
 * worst case estimated before the capacitor: ~1 A current pulses through the
 * pack's ~0.1-0.15 ohm source resistance (cells plus leads) is 100-150 mV
 * peak to peak at VM; tested at +-200 mV for margin, alternating sample to
 * sample, at the real 10 ms sample period. */
#define RIPPLE_MV          200u
#define SAMPLE_PERIOD_MS   10u

/* Feeds mean_mv +- ripple_mv (alternating, + first) every SAMPLE_PERIOD_MS
 * from from_ms; returns the time the state became wanted, or from_ms + FAR_MS. */
static uint32_t feed_rippled_until_state(BatteryMonitor *monitor, uint32_t mean_mv, uint32_t ripple_mv,
                                         uint32_t from_ms, BatteryState wanted)
{
  bool high = true;
  for (uint32_t now_ms = from_ms; now_ms - from_ms < FAR_MS; now_ms += SAMPLE_PERIOD_MS)
  {
    BatteryMonitor_UpdateMv(monitor, high ? mean_mv + ripple_mv : mean_mv - ripple_mv, false, now_ms);
    high = !high;
    if (monitor->state == wanted) { return now_ms; }
  }
  return from_ms + FAR_MS;
}

static void test_ripple_does_not_hold_off_the_cutoff(void)
{
  /* A pack resting at 7.4 V (armed, OK) sags to 5 mV under the cutoff and
   * hovers there. Clean, it cuts off after the filter delay (from 7.4 V to
   * under 6.6 V, ~0.64 s x ln(805 / 5) = ~3.3 s) plus the 3 s hold.
   * With the ripple, the filtered value keeps a jitter of
   * +-200 x (1/64) / (2 - 1/64) = +-1.6 mV, so it stays under 6.6 V only
   * once it is 1.6 mV further down: ~0.64 s x ln(5 / 3.4) = ~0.25 s later.
   * It must cut off within 0.5 s of the clean run, not be held off for
   * good by the jitter restarting the hold (as with 1/8: +-13 mV jitter,
   * more than the 5 mV margin). */
  BatteryMonitor rested;
  fresh(&rested);
  uint32_t armed_ms = feed_rippled_until_state(&rested, 7400u, 0u, 0u, BATTERY_STATE_OK);
  CHECK(armed_ms == BATTERY_PRESENCE_HOLD_MS, "setup: armed at %" PRIu32, armed_ms);
  uint32_t start_ms = armed_ms + 10000u;
  for (uint32_t now_ms = armed_ms + SAMPLE_PERIOD_MS; now_ms < start_ms; now_ms += SAMPLE_PERIOD_MS)
  {
    BatteryMonitor_UpdateMv(&rested, 7400u, false, now_ms);
  }
  CHECK(rested.state == BATTERY_STATE_OK && BatteryMonitor_FilteredMv(&rested) == 7400u, "setup: rested at 7.4 V");

  BatteryMonitor clean = rested;
  BatteryMonitor rippled = rested;
  uint32_t clean_ms = feed_rippled_until_state(&clean, BATTERY_CUTOFF_MV - 5u, 0u, start_ms, BATTERY_STATE_CUTOFF);
  uint32_t rippled_ms = feed_rippled_until_state(&rippled, BATTERY_CUTOFF_MV - 5u, RIPPLE_MV, start_ms,
                                                 BATTERY_STATE_CUTOFF);
  CHECK(clean_ms - start_ms > BATTERY_LEVEL_HOLD_MS && clean_ms - start_ms < BATTERY_LEVEL_HOLD_MS + 4000u,
        "clean: cutoff %" PRIu32 " ms after the sag (3 s hold + ~3.3 s filter delay)", clean_ms - start_ms);
  CHECK(rippled_ms + 2u * SAMPLE_PERIOD_MS >= clean_ms && rippled_ms <= clean_ms + 500u,
        "+-%u mV ripple: cutoff %" PRIu32 " ms after the sag, clean %" PRIu32 " ms", RIPPLE_MV,
        rippled_ms - start_ms, clean_ms - start_ms);
  CHECK(BatteryMonitor_MotorLockout(&rippled), "rippled pack locked out");

  /* The same ripple on a pack resting 5 mV ABOVE the cutoff never cuts off. */
  rippled = rested;
  uint32_t false_ms = feed_rippled_until_state(&rippled, BATTERY_CUTOFF_MV + 5u, RIPPLE_MV, start_ms,
                                               BATTERY_STATE_CUTOFF);
  CHECK(false_ms == start_ms + FAR_MS, "5 mV above the cutoff with ripple: no cutoff in a minute");
}

/* ---- Removed while driving ----------------------------------------------------------------- */

/* Feeds 0 V (unplugged) once per ms from from_ms with the given driving
 * flag, until the removal; returns that ms. */
static uint32_t unplug_with(BatteryMonitor *monitor, uint32_t from_ms, bool driving)
{
  for (uint32_t now_ms = from_ms; now_ms != from_ms + FAR_MS; now_ms++)
  {
    bool was_present = monitor->present;
    BatteryMonitor_UpdateMv(monitor, 0u, driving, now_ms);
    if (was_present && !monitor->present) { return now_ms; }
  }
  return from_ms + FAR_MS;
}

static void test_removed_while_driving_latches(void)
{
  /* Normal unplugging: Esc first, so the duty is 0 at the removal. */
  BatteryMonitor monitor;
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  uint32_t removed_ms = unplug_with(&monitor, 1000u, false);
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor),
        "unplugged with the motor stopped: NOT_PRESENT, no lockout (state %d)", (int)monitor.state);
  feed(&monitor, 8000u, removed_ms + 1u, removed_ms + 2000u);
  CHECK(monitor.state == BATTERY_STATE_OK, "and a pack plugged back in arms as OK (state %d)", (int)monitor.state);

  /* Removed while driving (sag below 5 V under load, or a loose connector). */
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  removed_ms = unplug_with(&monitor, 1000u, true);
  CHECK(monitor.state == BATTERY_STATE_REMOVED_WHILE_DRIVING && BatteryMonitor_MotorLockout(&monitor),
        "removed while driving: latched, locked out (state %d)", (int)monitor.state);
  for (uint32_t now_ms = removed_ms + 1u; now_ms < removed_ms + FAR_MS; now_ms++)
  {
    BatteryMonitor_UpdateMv(&monitor, 8400u, false, now_ms);  /* pack back, motor stopped */
  }
  CHECK(monitor.state == BATTERY_STATE_REMOVED_WHILE_DRIVING && BatteryMonitor_MotorLockout(&monitor),
        "pack back and motor stopped for a minute: still latched (state %d)", (int)monitor.state);
  BatteryMonitor_Init(&monitor, GOOD_CAL);
  CHECK(!BatteryMonitor_MotorLockout(&monitor), "only a reset clears it");

  /* Driving until just before the removal sample, stopped at it: no latch
   * (the rule is about the removal itself). */
  fresh(&monitor);
  feed(&monitor, 8000u, 0u, 1000u);
  uint32_t below_ms = 0u;
  for (uint32_t now_ms = 1000u; ; now_ms++)
  {
    BatteryMonitor_UpdateMv(&monitor, 0u, true, now_ms);
    if (BatteryMonitor_FilteredMv(&monitor) < BATTERY_REMOVED_MV) { below_ms = now_ms; break; }
  }
  for (uint32_t now_ms = below_ms + 1u; now_ms < below_ms + BATTERY_PRESENCE_HOLD_MS; now_ms++)
  {
    BatteryMonitor_UpdateMv(&monitor, 0u, true, now_ms);  /* still driving */
  }
  CHECK(monitor.present, "setup: not removed yet");
  BatteryMonitor_UpdateMv(&monitor, 0u, false, below_ms + BATTERY_PRESENCE_HOLD_MS);  /* the removal sample */
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor),
        "stopped at the removal sample: no latch (state %d)", (int)monitor.state);

  /* Driving with no pack ever plugged in (USB-only bench, "m 20"): nothing
   * is removed, so nothing latches. */
  fresh(&monitor);
  for (uint32_t now_ms = 0u; now_ms < FAR_MS; now_ms++) { BatteryMonitor_UpdateMv(&monitor, 0u, true, now_ms); }
  CHECK(monitor.state == BATTERY_STATE_NOT_PRESENT && !BatteryMonitor_MotorLockout(&monitor),
        "USB only, motor driving for a minute: no latch (state %d)", (int)monitor.state);

  /* CUTOFF still wins over it, and ADC_FAULT over both. */
  fresh(&monitor);
  feed(&monitor, 6000u, 0u, 4000u);
  unplug_with(&monitor, 4000u, true);
  CHECK(monitor.state == BATTERY_STATE_CUTOFF && monitor.removed_while_driving, "cutoff, then removed while driving: CUTOFF");
  BatteryMonitor_Update(&monitor, false, 1505u, 3080u, false, 10000u);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT, "then an ADC fault: ADC_FAULT");
}

/* ---- Missed samples ------------------------------------------------------------------------- */

static void test_missed_samples(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  for (uint32_t i = 0u; i < 100u; i++)
  {
    for (uint32_t miss = 0u; miss < BATTERY_MAX_MISSED_SAMPLES - 1u; miss++) { BatteryMonitor_MissedSample(&monitor); }
    BatteryMonitor_Update(&monitor, true, 1505u, 3080u, false, i * 10u);  /* a good one in between */
  }
  CHECK(monitor.state != BATTERY_STATE_ADC_FAULT && monitor.missed_count == 100u * (BATTERY_MAX_MISSED_SAMPLES - 1u),
        "%u misses in a row, then a good sample, 100 times: no fault (state %d, %u missed)",
        BATTERY_MAX_MISSED_SAMPLES - 1u, (int)monitor.state, (unsigned)monitor.missed_count);
  for (uint32_t miss = 0u; miss < BATTERY_MAX_MISSED_SAMPLES - 1u; miss++) { BatteryMonitor_MissedSample(&monitor); }
  CHECK(monitor.state != BATTERY_STATE_ADC_FAULT, "one short of the limit: no fault yet");
  BatteryMonitor_MissedSample(&monitor);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT && BatteryMonitor_MotorLockout(&monitor),
        "%u in a row: ADC fault, locked out", BATTERY_MAX_MISSED_SAMPLES);
  BatteryMonitor_Update(&monitor, true, 1505u, 3080u, false, 5000u);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT, "latched: a good sample doesn't clear it");

  /* A fault latched for another reason (bad VREFINT_CAL at boot) isn't
   * cleared or touched by missed samples either. */
  BatteryMonitor_Init(&monitor, 0xFFFFu);
  BatteryMonitor_MissedSample(&monitor);
  BatteryMonitor_Update(&monitor, true, 1505u, 3080u, false, 10u);
  CHECK(monitor.state == BATTERY_STATE_ADC_FAULT && BatteryMonitor_MotorLockout(&monitor) && monitor.missed_count == 0u,
        "boot fault: a missed sample leaves it latched (state %d, %u missed)", (int)monitor.state,
        (unsigned)monitor.missed_count);
}

/* ---- Invariants over a random hour ------------------------------------------------------- */

static void test_random_hour_invariants(void)
{
  BatteryMonitor monitor;
  fresh(&monitor);
  uint32_t battery_mv = 0u;
  bool driving = false;
  bool was_latched = false;
  int bad_lockout = 0;
  int latch_cleared = 0;
  int low_without_pack = 0;
  int driving_removals = 0;
  for (uint32_t now_ms = 0u; now_ms < 3600u * 1000u; now_ms += 10u)
  {
    uint32_t dice = next_random() % 1000u;
    if (dice < 2u)       { battery_mv = 0u; }                           /* unplugged */
    else if (dice < 4u)  { battery_mv = 8400u; }                        /* fresh pack */
    else if (dice < 300u && battery_mv != 0u) { battery_mv = 5800u + next_random() % 2800u; } /* anywhere in use; unplugged stays unplugged */
    if (dice >= 990u)    { driving = !driving; }
    if (dice == 999u)    { BatteryMonitor_Init(&monitor, GOOD_CAL); was_latched = false; }  /* reset */

    bool was_present = monitor.present;
    BatteryMonitor_UpdateMv(&monitor, battery_mv, driving, now_ms);
    if (was_present && !monitor.present && driving) { driving_removals++; }
    bool latched = monitor.state == BATTERY_STATE_CUTOFF || monitor.state == BATTERY_STATE_ADC_FAULT ||
                   monitor.state == BATTERY_STATE_REMOVED_WHILE_DRIVING;
    if (BatteryMonitor_MotorLockout(&monitor) != latched) { bad_lockout++; }
    if (was_latched && !latched) { latch_cleared++; }
    if (!monitor.present && monitor.low) { low_without_pack++; }
    was_latched = latched;
  }
  CHECK(bad_lockout == 0, "lockout disagreed with the state %d times", bad_lockout);
  CHECK(latch_cleared == 0, "a latched state cleared without a reset %d times", latch_cleared);
  CHECK(low_without_pack == 0, "LOW flagged with no pack %d times", low_without_pack);
  CHECK(driving_removals > 0, "the hour included removals while driving (%d)", driving_removals);
}

int main(void)
{
  test_decided_values();
  test_conversion();
  test_calibration_word_plausibility();
  test_sample_plausibility();
  test_adc_fault_latches_and_wins();
  test_filter();
  test_no_pack_never_arms_or_locks_out();
  test_arming_takes_exactly_the_presence_hold();
  test_removal();
  test_rearm_after_removal_starts_every_hold_again();
  test_warning_threshold_and_timing();
  test_warning_clears_with_hysteresis();
  test_cutoff_threshold_and_timing();
  test_cutoff_needs_an_unbroken_hold();
  test_cutoff_latches_until_reset();
  test_ripple_does_not_hold_off_the_cutoff();
  test_removed_while_driving_latches();
  test_missed_samples();
  test_random_hour_invariants();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
