#pragma once

#include "rover/vehicle_config.hpp"

namespace rover {

// Where the vehicle is, in a fixed "world" frame.
//   xMeters, yMeters: position of the rear axle's center.
//   headingRadians:   direction the vehicle points, measured counter-clockwise
//                     from the +x axis. 0 = facing +x, +pi/2 = facing +y.
//                     Always kept in the range (-pi, pi].
struct Pose2D {
  double xMeters{0.0};
  double yMeters{0.0};
  double headingRadians{0.0};
};

// Dead-reckoning motion model for a car-like vehicle (the "bicycle model").
//
// The idea: pretend the two front wheels are one wheel in the middle of the
// front axle, and likewise the rear pair. That gives a "bicycle" whose rear
// wheel goes straight and whose front wheel is steered by angle `delta`.
// The standard equations, for forward speed v and wheelbase L, are:
//
//     x'       = v * cos(heading)
//     y'       = v * sin(heading)
//     heading' = v / L * tan(delta)          <- "yaw rate"
//
// (the primes mean "rate of change per second").
//
// Sign convention: positive steering angle = steer left = turn counter-
// clockwise; positive speed = forward, negative = reverse.
//
// This class does not depend on ROS. A ROS 2 node can later own one of these
// and call step() from its timer callback.
class BicycleModel {
 public:
  // Copies the config (so the caller's object can go away afterwards).
  // Throws std::invalid_argument if wheelbaseMeters <= 0 or
  // maxSteeringAngleRadians is not in (0, pi/2) - both would make the
  // equations meaningless (division by zero, or tan() blowing up).
  explicit BicycleModel(const VehicleConfig& vehicleConfig);

  // Moves the model to `newPose` (default: the origin, facing +x).
  void resetPose(const Pose2D& newPose = Pose2D{});

  const Pose2D& pose() const { return currentPose_; }

  // Advances the pose by `timeStepSeconds`, assuming speed and steering stay
  // constant for that duration.
  //   - steeringAngleRadians is clamped to +/- maxSteeringAngleRadians.
  //   - timeStepSeconds == 0 is a no-op (pose unchanged).
  //   - Throws std::invalid_argument if timeStepSeconds is negative, or any
  //     argument is NaN/infinite; the pose is left unchanged in that case.
  void step(double forwardSpeedMetersPerSecond, double steeringAngleRadians,
            double timeStepSeconds);

 private:
  // Trailing underscore = private data member (a common C++ convention, like
  // Python's leading underscore).
  VehicleConfig vehicleConfig_;
  Pose2D currentPose_;
};

}  // namespace rover
