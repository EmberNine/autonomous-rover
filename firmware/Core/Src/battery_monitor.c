/**
  ******************************************************************************
  * @file    battery_monitor.c
  * @brief   2S LiPo voltage monitor: conversion, plausibility, filter and
  *          state machine (see the header).
  ******************************************************************************
  */
#include "battery_monitor.h"

#define FILTER_ONE  (1u << BATTERY_FILTER_FRACTION_BITS)

/* The most one sample can move the filter up from just below REMOVED: the
 * largest pack voltage the ADC can report (full scale at VDDA_MAX_MV,
 * 11605 mV) minus REMOVED, over 2^SHIFT. Less than the presence gap, so the
 * first sample after a removal always breaks present_hold. */
#define MAX_STEP_MV  (((VDDA_MAX_MV * BATTERY_DIVIDER_DEN / BATTERY_DIVIDER_NUM) - BATTERY_REMOVED_MV) >> BATTERY_FILTER_SHIFT)
#if !(MAX_STEP_MV < BATTERY_PRESENT_MV - BATTERY_REMOVED_MV)
#error "one sample could lift the filter from REMOVED to PRESENT: reset present_hold on removal"
#endif

/* True once condition has held, without a break, for hold_ms (measured
 * between sample times, so wrap-safe). A false sample restarts it. */
static bool BatteryHold_Update(BatteryHold *hold, bool condition, uint32_t now_ms, uint32_t hold_ms)
{
  if (!condition)
  {
    hold->holding = false;
    return false;
  }
  if (!hold->holding)
  {
    hold->holding = true;
    hold->since_ms = now_ms;
  }
  return (uint32_t)(now_ms - hold->since_ms) >= hold_ms;
}

static void BatteryHold_Reset(BatteryHold *hold)
{
  hold->holding = false;
}

/* Never called once the ADC fault is latched (it sets the state itself and
 * stops all updates), so ADC_FAULT wins without a test here. */
static BatteryState BatteryMonitor_DeriveState(const BatteryMonitor *monitor)
{
  if (monitor->cutoff_latched) { return BATTERY_STATE_CUTOFF; }
  if (monitor->removed_while_driving) { return BATTERY_STATE_REMOVED_WHILE_DRIVING; }
  if (!monitor->present)       { return BATTERY_STATE_NOT_PRESENT; }
  if (monitor->low)            { return BATTERY_STATE_LOW; }
  return BATTERY_STATE_OK;
}

static void BatteryMonitor_LatchAdcFault(BatteryMonitor *monitor)
{
  monitor->adc_fault = true;
  monitor->state = BATTERY_STATE_ADC_FAULT;
}

uint32_t Battery_VddaMv(uint32_t vrefint_cal, uint32_t vrefint_raw)
{
  if (vrefint_raw == 0u) { return 0u; }
  /* vrefint_cal <= 0xFFFF: 3000 x 65535 fits in 32 bits. */
  return (VREFINT_CAL_VREF_MV * vrefint_cal + vrefint_raw / 2u) / vrefint_raw;
}

uint32_t Battery_PackMv(uint32_t vdda_mv, uint32_t battery_raw)
{
  /* Pin voltage in 0.1 mV first (<= 3600 x 4095 x 10 = 147,420,000), then
   * divided by the ratio (<= 36000 x 10000 / 10 = 36,000,000). */
  uint32_t pin_tenth_mv = (vdda_mv * battery_raw * 10u + ADC_FULL_SCALE / 2u) / ADC_FULL_SCALE;
  uint32_t numerator = pin_tenth_mv * (BATTERY_DIVIDER_DEN / 10u);
  return (numerator + BATTERY_DIVIDER_NUM / 2u) / BATTERY_DIVIDER_NUM;
}

void BatteryMonitor_Init(BatteryMonitor *monitor, uint16_t vrefint_cal)
{
  *monitor = (BatteryMonitor){0};
  monitor->vrefint_cal = vrefint_cal;
  monitor->state = BATTERY_STATE_NOT_PRESENT;
  if (vrefint_cal < VREFINT_CAL_MIN || vrefint_cal > VREFINT_CAL_MAX)
  {
    BatteryMonitor_LatchAdcFault(monitor);
  }
}

void BatteryMonitor_MissedSample(BatteryMonitor *monitor)
{
  if (monitor->adc_fault) { return; }
  monitor->missed_count++;
  monitor->missed_in_a_row++;
  if (monitor->missed_in_a_row >= BATTERY_MAX_MISSED_SAMPLES)
  {
    BatteryMonitor_LatchAdcFault(monitor);
  }
}

void BatteryMonitor_Update(BatteryMonitor *monitor, bool conversion_ok, uint32_t vrefint_raw,
                           uint32_t battery_raw, bool motor_driving, uint32_t now_ms)
{
  if (monitor->adc_fault) { return; }
  monitor->missed_in_a_row = 0u;
  /* A VREFINT reading of 0 or above 12 bits needs no check of its own: it
   * gives a VDDA of 0 or far below VDDA_MIN_MV. */
  if (!conversion_ok || battery_raw > ADC_FULL_SCALE)
  {
    BatteryMonitor_LatchAdcFault(monitor);
    return;
  }
  monitor->vdda_mv = Battery_VddaMv(monitor->vrefint_cal, vrefint_raw);
  if (monitor->vdda_mv < VDDA_MIN_MV || monitor->vdda_mv > VDDA_MAX_MV)
  {
    BatteryMonitor_LatchAdcFault(monitor);
    return;
  }
  BatteryMonitor_UpdateMv(monitor, Battery_PackMv(monitor->vdda_mv, battery_raw), motor_driving, now_ms);
}

void BatteryMonitor_UpdateMv(BatteryMonitor *monitor, uint32_t battery_mv, bool motor_driving, uint32_t now_ms)
{
  if (monitor->adc_fault) { return; }
  monitor->battery_mv = battery_mv;

  uint32_t sample_q = battery_mv << BATTERY_FILTER_FRACTION_BITS;
  if (!monitor->seeded)
  {
    monitor->filtered_q = sample_q;
    monitor->seeded = true;
  }
  else if (sample_q >= monitor->filtered_q)
  {
    monitor->filtered_q += (sample_q - monitor->filtered_q) >> BATTERY_FILTER_SHIFT;
  }
  else
  {
    monitor->filtered_q -= (monitor->filtered_q - sample_q) >> BATTERY_FILTER_SHIFT;
  }
  uint32_t filtered_mv = BatteryMonitor_FilteredMv(monitor);

  /* A hold only runs in its own phase, so one left "holding" from an
   * earlier phase would fire early when its phase comes back, unless its
   * first sample there breaks it (which restarts it). On removal, two can
   * be left like that and are reset: warn_hold and cutoff_hold (the pack
   * passed below 7.0 / 6.6 V on its way out, and a flat pack re-armed is
   * below them from its first sample). The other three restart themselves:
   *   - present_hold: removal leaves the filter below REMOVED, and one
   *     sample can't lift it to PRESENT (MAX_STEP_MV below);
   *   - removed_hold: its next first sample is the arming one
   *     (>= PRESENT > REMOVED);
   *   - clear_hold: its next first sample is the one after a warning
   *     (< WARN < WARN_CLEAR). */
  if (!monitor->present)
  {
    if (BatteryHold_Update(&monitor->present_hold, filtered_mv >= BATTERY_PRESENT_MV, now_ms,
                           BATTERY_PRESENCE_HOLD_MS))
    {
      monitor->present = true;
    }
  }
  else if (BatteryHold_Update(&monitor->removed_hold, filtered_mv < BATTERY_REMOVED_MV, now_ms,
                              BATTERY_PRESENCE_HOLD_MS))
  {
    monitor->present = false;
    monitor->low = false;
    /* Judged at the removal sample itself: normal unplugging stops the
     * motor first (Esc), so its duty is 0 by now. */
    if (motor_driving) { monitor->removed_while_driving = true; }
    BatteryHold_Reset(&monitor->warn_hold);
    BatteryHold_Reset(&monitor->cutoff_hold);
  }

  /* Only while armed: with no pack (0 V on USB) nothing is low. Evaluated in
   * the sample that arms, too, but every hold starts there. */
  if (monitor->present && !monitor->cutoff_latched)
  {
    if (BatteryHold_Update(&monitor->cutoff_hold, filtered_mv < BATTERY_CUTOFF_MV, now_ms, BATTERY_LEVEL_HOLD_MS))
    {
      monitor->cutoff_latched = true;
    }
    if (!monitor->low)
    {
      if (BatteryHold_Update(&monitor->warn_hold, filtered_mv < BATTERY_WARN_MV, now_ms, BATTERY_LEVEL_HOLD_MS))
      {
        monitor->low = true;  /* clear_hold restarts on the next sample (< WARN_CLEAR) */
      }
    }
    else if (BatteryHold_Update(&monitor->clear_hold, filtered_mv >= BATTERY_WARN_CLEAR_MV, now_ms,
                                BATTERY_LEVEL_HOLD_MS))
    {
      monitor->low = false;  /* warn_hold restarts on the next sample (>= WARN) */
    }
  }

  monitor->state = BatteryMonitor_DeriveState(monitor);
}

bool BatteryMonitor_MotorLockout(const BatteryMonitor *monitor)
{
  return monitor->adc_fault || monitor->cutoff_latched || monitor->removed_while_driving;
}

uint32_t BatteryMonitor_FilteredMv(const BatteryMonitor *monitor)
{
  return (monitor->filtered_q + FILTER_ONE / 2u) >> BATTERY_FILTER_FRACTION_BITS;
}
