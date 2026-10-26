#pragma once

// Private helper shared by the core library's .cpp files (not part of the
// public include/rover/ API).

#include <cmath>

namespace rover {

inline constexpr double kPi = 3.14159265358979323846;

// Wraps an angle into (-pi, pi] so headings never grow without bound
// (turning left forever would otherwise reach 100 rad, 1000 rad, ...).
inline double wrapAngleToPlusMinusPi(double angleRadians) {
  // Already in range: return it untouched. The fmod path below would round
  // it (0.3 -> 0.29999999999999982), and the filter re-wraps after every
  // update, so that rounding would otherwise accumulate as heading noise.
  if (angleRadians > -kPi && angleRadians <= kPi) {
    return angleRadians;
  }
  double wrappedAngle = std::fmod(angleRadians + kPi, 2.0 * kPi);  // fmod = float modulo (%)
  if (wrappedAngle <= 0.0) {
    wrappedAngle += 2.0 * kPi;
  }
  return wrappedAngle - kPi;
}

}  // namespace rover
