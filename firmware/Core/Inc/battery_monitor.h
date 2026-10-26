/**
  ******************************************************************************
  * @file    battery_monitor.h
  * @brief   2S LiPo voltage monitor, with no hardware access: the caller
  *          passes ADC1's two injected results (VREFINT, then the battery
  *          divider on PA6 / IN11), the factory VREFINT_CAL word and "now"
  *          in ms, and reads back the state and whether the motor must be
  *          locked out.
  *
  * Conversion (integer mV, no floats):
  *   - VDDA = VREFINT_CAL_VREF_MV x VREFINT_CAL / vrefint_raw. VREFINT_CAL is
  *     the VREFINT reading taken in production at VREF+ = 3.0 V
  *     (stm32l4xx_ll_adc.h, VREFINT_CAL_ADDR / VREFINT_CAL_VREF), so this
  *     corrects for the real VDDA instead of assuming 3.3 V.
  *   - pin = VDDA x battery_raw / 4095; battery = pin / divider ratio, the
  *     ratio being BATTERY_DIVIDER_NUM / BATTERY_DIVIDER_DEN (measured
  *     2.457 V at the pin for 7.92 V at the pack, 2026-10-14).
  *
  * Plausibility: a conversion error, VREFINT_CAL outside its datasheet range,
  * a zero VREFINT reading, a reading above 12 bits or a VDDA outside
  * [VDDA_MIN_MV, VDDA_MAX_MV] is an ADC fault, at once (one sample). An ADC fault is LATCHED until reset and locks the motor out:
  * with the reference wrong, every voltage is wrong, including the cutoff.
  *
  * The decisions run on an IIR-filtered voltage (seeded with the first
  * sample, so there is no ramp up from 0 at boot), each one only after its
  * condition has held without a break for its whole time:
  *   - NOT_PRESENT at boot (e.g. on USB only, no LiPo): no warning, no
  *     cutoff, no lockout, so the no-LiPo bench checks still work.
  *   - Armed (OK) once >= BATTERY_PRESENT_MV for BATTERY_PRESENCE_HOLD_MS.
  *   - Removed (back to NOT_PRESENT) once < BATTERY_REMOVED_MV for
  *     BATTERY_PRESENCE_HOLD_MS. Faster than the 3 s cutoff, so unplugging
  *     the pack disarms instead of latching a cutoff.
  *   - LOW (warning) once < BATTERY_WARN_MV for BATTERY_LEVEL_HOLD_MS;
  *     back to OK once >= BATTERY_WARN_CLEAR_MV for BATTERY_LEVEL_HOLD_MS.
  *   - CUTOFF once < BATTERY_CUTOFF_MV for BATTERY_LEVEL_HOLD_MS while
  *     armed. LATCHED until reset: not cleared by the voltage recovering
  *     (a resting LiPo recovers) nor by removing the pack. Locks the motor
  *     out.
  *   - REMOVED_WHILE_DRIVING: the removal above happened in a sample where
  *     the caller said the motor was driving (non-zero duty). Unplugging
  *     normally (motor stopped first) can't do that; a pack sagging below
  *     5.0 V under load, or a connector coming loose while driving, can.
  *     LATCHED until reset like CUTOFF, and locks the motor out.
  *   - Missed samples (the conversion wasn't finished when the next was
  *     due): BATTERY_MAX_MISSED_SAMPLES in a row is an ADC fault.
  * Precedence: ADC_FAULT > CUTOFF > REMOVED_WHILE_DRIVING > NOT_PRESENT >
  * LOW > OK.
  *
  * BATTERY_CUTOFF_TEST (bench test builds only, like IWDG_HANG_TEST): 1
  * raises the warning and cutoff thresholds above a full 2S pack (8.4 V), so
  * a real pack trips the cutoff on the bench. Presence and removal are
  * unchanged. Builds with a #warning; tools/test_firmware_sources.py fails
  * if it is ever committed as 1.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_battery_monitor.c. No globals.
  ******************************************************************************
  */
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Conversion -------------------------------------------------------------- */

/* Vref+ at which VREFINT_CAL was taken in production (VREFINT_CAL_VREF in
 * stm32l4xx_ll_adc.h, +-10 mV). */
#define VREFINT_CAL_VREF_MV      3000u
#define ADC_FULL_SCALE           4095u   /* 12-bit right-aligned */

/* Divider ratio pin / pack = 3102 / 10000 (2.457 V / 7.92 V, 2026-10-14). */
#define BATTERY_DIVIDER_NUM      3102u
#define BATTERY_DIVIDER_DEN      10000u

/* VREFINT 1.182-1.232 V (L432 datasheet, embedded reference) read at
 * 3.0 V -> 1613-1682 counts, with margin. Anything else is not a
 * calibration word (erased flash reads 0xFFFF). */
#define VREFINT_CAL_MIN          1600u
#define VREFINT_CAL_MAX          1700u

/* VDDA is the Nucleo's 3.3 V (measured ~3.28 V); 3.6 V is the MCU maximum. */
#define VDDA_MIN_MV              3000u
#define VDDA_MAX_MV              3600u

/* ---- Thresholds (filtered pack voltage) -------------------------------------- */

#define BATTERY_CUTOFF_TEST      0       /* 1 = bench cutoff test build only (see above) */

#define BATTERY_REMOVED_MV       5000u   /* below: removed */
#define BATTERY_PRESENT_MV       5500u   /* at or above: present (500 mV above removed) */
#if BATTERY_CUTOFF_TEST
#warning "BATTERY_CUTOFF_TEST=1: cutoff above a full pack, the motor locks out ~3 s after a LiPo arms. Bench test only."
#define BATTERY_CUTOFF_MV        8500u   /* above a full 2S pack (8.4 V) */
#define BATTERY_WARN_MV          8600u
#define BATTERY_WARN_CLEAR_MV    8800u
#else
#define BATTERY_CUTOFF_MV        6600u   /* below: cutoff, 3.3 V/cell */
#define BATTERY_WARN_MV          7000u   /* below: warning, 3.5 V/cell */
#define BATTERY_WARN_CLEAR_MV    7200u   /* at or above: warning cleared */
#endif

#define BATTERY_PRESENCE_HOLD_MS 500u
#define BATTERY_LEVEL_HOLD_MS    3000u

#if !(BATTERY_REMOVED_MV < BATTERY_PRESENT_MV && BATTERY_PRESENT_MV < BATTERY_CUTOFF_MV && \
      BATTERY_CUTOFF_MV < BATTERY_WARN_MV && BATTERY_WARN_MV < BATTERY_WARN_CLEAR_MV)
#error "battery thresholds must satisfy REMOVED < PRESENT < CUTOFF < WARN < WARN_CLEAR"
#endif
#if !(BATTERY_PRESENCE_HOLD_MS < BATTERY_LEVEL_HOLD_MS)
#error "removal must be detected before a cutoff can latch"
#endif

/* IIR: y += (x - y) / 2^BATTERY_FILTER_SHIFT, y kept with
 * BATTERY_FILTER_FRACTION_BITS extra bits so it settles on the exact mV
 * (it stops moving within 2^SHIFT - 1 fraction units, under 0.5 mV).
 * 1/64 per sample: at a 10 ms sample period the time constant is about
 * 0.64 s. A5 has no RC filter, so the motor's 20 kHz PWM ripple on VM
 * (reduced by the 470 uF bulk capacitor, not removed) reaches the samples;
 * with a short time constant
 * the filtered value near a threshold would jitter across it and keep
 * restarting the 3 s hold (test_battery_monitor.c, "ripple"). Max value:
 * 11605 mV << 8 fits easily in 32 bits. */
#define BATTERY_FILTER_SHIFT          6u
#define BATTERY_FILTER_FRACTION_BITS  8u

/* A conversion takes ~41 us and is due every 10 ms, so one that isn't
 * finished means the ADC stopped. Three in a row (30 ms) -> ADC fault:
 * a single miss tolerated, a stopped ADC caught well inside a 3 s hold. */
#define BATTERY_MAX_MISSED_SAMPLES    3u

/* ---- State --------------------------------------------------------------------- */

typedef enum
{
  BATTERY_STATE_NOT_PRESENT = 0,  /* boot, or removed: no warning, no cutoff */
  BATTERY_STATE_OK = 1,           /* armed, above the warning */
  BATTERY_STATE_LOW = 2,          /* armed, warning */
  BATTERY_STATE_CUTOFF = 3,       /* latched until reset; motor locked out */
  BATTERY_STATE_ADC_FAULT = 4,    /* latched until reset; motor locked out */
  BATTERY_STATE_REMOVED_WHILE_DRIVING = 5  /* latched until reset; motor locked out */
} BatteryState;

/* A condition that must hold without a break for some time. */
typedef struct
{
  bool holding;
  uint32_t since_ms;
} BatteryHold;

typedef struct
{
  BatteryState state;
  uint16_t vrefint_cal;
  bool adc_fault;            /* latched */
  bool cutoff_latched;       /* latched */
  bool removed_while_driving; /* latched */
  bool present;
  bool low;
  bool seeded;
  uint32_t filtered_q;       /* mV << BATTERY_FILTER_FRACTION_BITS */
  uint32_t vdda_mv;          /* last computed, also when implausible (0 before the first sample) */
  uint32_t battery_mv;       /* last unfiltered pack voltage */
  uint32_t missed_in_a_row;  /* missed samples since the last good one */
  uint32_t missed_count;     /* missed samples since boot (debugger only) */
  BatteryHold present_hold;
  BatteryHold removed_hold;
  BatteryHold warn_hold;
  BatteryHold clear_hold;
  BatteryHold cutoff_hold;
} BatteryMonitor;

/* vrefint_cal: the word at VREFINT_CAL_ADDR. Out of range -> ADC_FAULT at once. */
void BatteryMonitor_Init(BatteryMonitor *monitor, uint16_t vrefint_cal);

/* One sample: both injected results of one conversion sequence.
 * conversion_ok = false (HAL error or timeout) is an ADC fault.
 * motor_driving: the motor has a non-zero duty right now (for the
 * removed-while-driving rule). */
void BatteryMonitor_Update(BatteryMonitor *monitor, bool conversion_ok, uint32_t vrefint_raw,
                           uint32_t battery_raw, bool motor_driving, uint32_t now_ms);

/* The voltage-domain step of Update (filter, presence, warning, cutoff),
 * for a pack voltage already converted and checked. */
void BatteryMonitor_UpdateMv(BatteryMonitor *monitor, uint32_t battery_mv, bool motor_driving, uint32_t now_ms);

/* The sample due now wasn't ready (no conversion result). Counted; the
 * BATTERY_MAX_MISSED_SAMPLES-th in a row latches an ADC fault. */
void BatteryMonitor_MissedSample(BatteryMonitor *monitor);

/* True when the motor must not run: cutoff, ADC fault or removed while
 * driving (all latched). */
bool BatteryMonitor_MotorLockout(const BatteryMonitor *monitor);

/* The filtered pack voltage in mV (rounded); 0 before the first sample. */
uint32_t BatteryMonitor_FilteredMv(const BatteryMonitor *monitor);

/* VDDA in mV from a VREFINT reading (rounded); 0 if vrefint_raw is 0. */
uint32_t Battery_VddaMv(uint32_t vrefint_cal, uint32_t vrefint_raw);

/* Pack voltage in mV (rounded) for a divider reading. Needs
 * vdda_mv <= VDDA_MAX_MV and battery_raw <= ADC_FULL_SCALE (32-bit range). */
uint32_t Battery_PackMv(uint32_t vdda_mv, uint32_t battery_raw);

#endif /* BATTERY_MONITOR_H */
