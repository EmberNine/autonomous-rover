/**
  ******************************************************************************
  * @file    angle_correction.h
  * @brief   Removes the encoder's 4-per-revolution angle error from the raw
  *          AS5600 angle. No hardware access.
  *
  * The wheel angle as read carries an error that repeats 4 times per
  * revolution: about +-44 counts, which looks like +-24 % speed ripple. It is
  * the same at every speed and in both directions, with its maxima tied to
  * the sensor's own angle frame. The likely cause is a sensor/magnet
  * measurement error rather than the wheel; it is not proven (a rotation test
  * that would separate the two isn't possible on this mount). Model and
  * correction, in counts:
  *     eps(raw)  = A * sin(2 pi (raw + r0) / 1024)
  *     corrected = (raw - eps(raw)) mod 4096
  * Only this 4th harmonic is removed: the once-per-turn part is partly a real
  * mechanical tight spot (it shrinks with speed), and the 2nd and 3rd aren't
  * the same from capture to capture.
  *
  * Properties (tests/test_angle_correction.c checks each):
  *   - eps averages to zero over a revolution (it is periodic in 1024), so
  *     any whole number of revolutions gives the same count difference as
  *     the raw angle: distances over full turns, and meters_per_encoder_tick
  *     (calibrated on raw counts over ~6.2 turns, worst endpoint effect
  *     2A = 88 counts = 0.35 %), are unchanged;
  *   - its slope is at most 0.27 count per count, so the corrected angle
  *     never runs backwards, and |eps| <= 44 is far inside the unwrapper's
  *     half-turn rule: feed the corrected angle to its own AS5600_Unwrapper.
  *
  * Integer only (no float): one quarter-wave sine table (257 x int16, Q15),
  * a fold into four quadrants, one multiply, rounded half away from zero.
  * The same on the host and on the target, bit for bit.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_angle_correction.c. No globals except the constant table.
  ******************************************************************************
  */
#ifndef ANGLE_CORRECTION_H
#define ANGLE_CORRECTION_H

#include <stdint.h>

/* Fitted with tools/fit_angle_correction.py from cap_m30_c2_1.log on
 * 2026-10-15 (steady 30 % duty, 6.4 revolutions; the other captures of that
 * session agree within +-5 % amplitude, +-5 degrees phase). The encoder board
 * is seated on its pegs, NOT glued, so these numbers are tied to its current
 * seating: re-fit if ML or the magnitude at rest change, or if the
 * 4-per-revolution ripple comes back in vest. Pinned by
 * tests/test_angle_correction.c and tools/test_firmware_sources.py. */
#define ANGLE_H4_AMPLITUDE_Q4    703   /* A in 1/16 counts: 43.94 counts */
#define ANGLE_H4_PHASE_COUNTS    (-9)  /* r0, counts */

#define ANGLE_COUNTS_PER_REV     4096u
#define ANGLE_H4_PERIOD          1024u   /* counts: 4 periods per revolution */

#if (ANGLE_H4_PHASE_COUNTS < -1023) || (ANGLE_H4_PHASE_COUNTS > 1023)
#error "ANGLE_H4_PHASE_COUNTS must be within one H4 period"
#endif
#if (ANGLE_H4_AMPLITUDE_Q4 < 0) || (ANGLE_H4_AMPLITUDE_Q4 > 16 * 150)
#error "ANGLE_H4_AMPLITUDE_Q4: the correction must keep the corrected angle increasing (A < 1024 / (2 pi) = 163 counts) with margin"
#endif

/* eps(raw) in whole counts (raw 0..4095). */
int32_t AngleCorrection_ErrorCounts(uint16_t raw);

/* The corrected angle, 0..4095. */
uint16_t AngleCorrection_Apply(uint16_t raw);

/* sin(2 pi k / 1024) in Q15 (32767 = 1.0), k 0..1023, from the quarter-wave
 * table. Exposed for the host test. */
int32_t AngleCorrection_SineQ15(uint32_t k);

#endif /* ANGLE_CORRECTION_H */
