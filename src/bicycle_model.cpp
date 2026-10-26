#include "rover/bicycle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "angle_math.hpp"

namespace rover {

namespace {

// sin(u) / u, but safe at u == 0 (where the true limit is 1). For tiny u we
// use the first terms of its Taylor series instead of dividing 0 by 0.
double sinOverX(double u) {
  if (std::abs(u) < 1e-8) {
    return 1.0 - u * u / 6.0;
  }
  return std::sin(u) / u;
}

}  // namespace

BicycleModel::BicycleModel(const VehicleConfig& vehicleConfig) : vehicleConfig_(vehicleConfig) {
  if (!(vehicleConfig_.wheelbaseMeters > 0.0) || !std::isfinite(vehicleConfig_.wheelbaseMeters)) {
    throw std::invalid_argument("wheelbaseMeters must be positive and finite");
  }
  if (!(vehicleConfig_.maxSteeringAngleRadians > 0.0) ||
      !(vehicleConfig_.maxSteeringAngleRadians < kPi / 2.0)) {
    throw std::invalid_argument("maxSteeringAngleRadians must be in (0, pi/2)");
  }
}

void BicycleModel::resetPose(const Pose2D& newPose) {
  currentPose_ = newPose;
  currentPose_.headingRadians = wrapAngleToPlusMinusPi(currentPose_.headingRadians);
}

void BicycleModel::step(double forwardSpeedMetersPerSecond, double steeringAngleRadians,
                        double timeStepSeconds) {
  // Validate everything up front so a bad call never half-updates the pose.
  if (!std::isfinite(forwardSpeedMetersPerSecond) || !std::isfinite(steeringAngleRadians) ||
      !std::isfinite(timeStepSeconds)) {
    throw std::invalid_argument("step() arguments must be finite numbers");
  }
  if (timeStepSeconds < 0.0) {
    throw std::invalid_argument("timeStepSeconds must not be negative");
  }
  if (timeStepSeconds == 0.0) {
    return;
  }

  const double clampedSteeringAngleRadians =
      std::clamp(steeringAngleRadians, -vehicleConfig_.maxSteeringAngleRadians,
                 vehicleConfig_.maxSteeringAngleRadians);

  // The bicycle-model equation: how fast the heading rotates (radians/second).
  const double yawRateRadiansPerSecond =
      forwardSpeedMetersPerSecond / vehicleConfig_.wheelbaseMeters *
      std::tan(clampedSteeringAngleRadians);

  const double headingChangeRadians = yawRateRadiansPerSecond * timeStepSeconds;
  const double distanceTraveledMeters = forwardSpeedMetersPerSecond * timeStepSeconds;

  // Integration: with constant speed and steering, the vehicle drives along a
  // circular arc. Rather than approximating with many tiny straight steps
  // (Euler integration, which drifts), we use the exact result: the straight
  // chord from the arc's start to its end has length
  //     distance * sin(dTheta/2) / (dTheta/2)
  // and points along the *average* heading (start heading + dTheta/2).
  // sinOverX() makes this also work for driving straight (dTheta == 0),
  // so there is no special case.
  const double halfHeadingChangeRadians = headingChangeRadians / 2.0;
  const double chordLengthMeters = distanceTraveledMeters * sinOverX(halfHeadingChangeRadians);
  const double averageHeadingRadians = currentPose_.headingRadians + halfHeadingChangeRadians;

  currentPose_.xMeters += chordLengthMeters * std::cos(averageHeadingRadians);
  currentPose_.yMeters += chordLengthMeters * std::sin(averageHeadingRadians);
  currentPose_.headingRadians =
      wrapAngleToPlusMinusPi(currentPose_.headingRadians + headingChangeRadians);
}

}  // namespace rover
