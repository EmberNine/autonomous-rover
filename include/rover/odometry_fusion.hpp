#pragma once

#include <optional>

#include "rover/extended_kalman_filter.hpp"

namespace rover {

// Everything OdometryFusion needs to know about the truck and its inputs.
// Lengths in meters, angles in radians, times in seconds.
struct OdometryFusionConfig {
  double wheelbaseMeters{0.0};
  double rearTrackWidthMeters{0.0};     // the encoder wheel is half of this left of the centre
  bool rearAxleLocked{false};           // vehicle.cfg rear_axle_locked
  ExtendedKalmanFilterNoise noise;

  double restSecondsForBias{2.0};       // at rest this long before the filter starts
  double restSpeedMetersPerSecond{0.005};  // |encoder speed| below this counts as "at rest"
  double maxPredictStepSeconds{0.05};   // a longer gap between gyro samples is not predicted across
  double encoderMaxAgeSeconds{0.03};    // an encoder sample older than this (vs the gyro's) is not used
  double steeringMaxAgeSeconds{0.3};    // the same for the steering angle (STATUS comes at 10 Hz)
};

// The input side of the EKF node (ros_ws/src/rover_ekf), ROS-free so it is
// tested here: it turns the three input streams into ExtendedKalmanFilter
// calls.
//
//   onGyro      yaw rate [rad/s] at a stamp (the IMU's z axis: X forward,
//               Y left, Z up). Each sample is one filter cycle: predict over
//               the time since the previous sample (capped at
//               maxPredictStepSeconds, so a gap holds the pose instead of
//               extrapolating it), the gyro update, then the newest encoder
//               sample (once, if not older than encoderMaxAgeSeconds) and
//               the steering model (if the angle is not older than
//               steeringMaxAgeSeconds, and not while a fresh encoder sample
//               reads at rest: below restSpeedMetersPerSecond its "not
//               turning" would turn a hand rotation into gyro bias and pull
//               the heading back, 2026-10-25 bench).
//   onEncoder   the rear LEFT wheel's speed [m/s], or nullopt when the
//               firmware's estimate is not valid. The filter's speed is the
//               rear-axle centre's. Locked rear axle (the truck): both
//               wheels turn together and scrub, so the encoder wheel reads
//               the centre's speed, v_centre = v_left. Open differential:
//               v_centre = v_left + yaw_rate * track / 2 (the left wheel is
//               on the inside of a left turn).
//   onSteering  the steering angle the servo has [rad] (bridge /rover/steering).
//
// Start: the filter starts after restSecondsForBias of gyro samples taken
// while the encoder reads valid and at rest; their mean seeds the gyro bias.
// Motion restarts the count; an invalid encoder sample pauses it. Until then
// started() is false and nothing should be published.
//
// Inputs that are not finite, and stamps that go backwards, are dropped and
// counted.
class OdometryFusion {
 public:
  // Throws std::invalid_argument if the wheelbase or the track is not positive
  // and finite (the noise is checked by ExtendedKalmanFilter on start).
  explicit OdometryFusion(const OdometryFusionConfig& config);

  void onEncoder(double stampSeconds, std::optional<double> leftWheelSpeedMetersPerSecond);
  void onSteering(double stampSeconds, double steeringAngleRadians);
  // Returns true when the filter advanced (it has started).
  bool onGyro(double stampSeconds, double yawRateRadiansPerSecond);

  // Pose back to (0, 0, 0) with a small uncertainty; speed, yaw rate and the
  // gyro bias are kept. No-op before the start.
  void resetPose();

  bool started() const { return filter_.has_value(); }
  const ExtendedKalmanFilter& filter() const { return *filter_; }  // only once started()
  double lastStampSeconds() const { return lastGyroStamp_.value_or(0.0); }

  struct Counts {
    long cycles{0};
    long encoderUpdates{0};
    long steeringUpdates{0};
    long gaps{0};            // gyro gaps longer than maxPredictStepSeconds
    long dropped{0};         // non-finite inputs and stamps going backwards
  };
  const Counts& counts() const { return counts_; }

 private:
  void collectRest(double stampSeconds, double yawRate);

  OdometryFusionConfig config_;
  std::optional<ExtendedKalmanFilter> filter_;
  std::optional<double> lastGyroStamp_;

  std::optional<double> encoderStamp_;
  std::optional<double> encoderSpeed_;     // nullopt: the last sample was not valid
  bool encoderUsed_{true};

  std::optional<double> steeringStamp_;
  double steeringAngle_{0.0};

  std::optional<double> restStart_;
  double restSum_{0.0};
  long restCount_{0};

  Counts counts_;
};

}  // namespace rover
