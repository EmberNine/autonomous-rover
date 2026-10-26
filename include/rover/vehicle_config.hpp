#pragma once

#include <string>

namespace rover {

// Physical measurements of the vehicle. Think of this as a typed row of
// settings: a plain struct (like a Python dataclass) with no behavior other
// than being loadable from a text file.
//
// Lengths are in meters and angles in radians everywhere in this project.
// A value of 0.0 means "not set" - the file simply didn't mention that key.
struct VehicleConfig {
  std::string name{"unnamed_vehicle"};
  double wheelbaseMeters{0.0};             // front axle center to rear axle center
  double rearTrackWidthMeters{0.0};        // rear wheel centre to rear wheel centre
  double tireDiameterMeters{0.0};
  double metersPerEncoderTick{0.0};        // distance the vehicle travels per encoder tick
  double maxSteeringAngleRadians{0.0};     // steering limit at full lock, one side
  bool rearAxleLocked{false};              // both rear wheels on one shaft (no differential)

  // Reads a plain-text file of `key = value` lines.
  //   - '#' starts a comment (whole line or after a value); blank lines are skipped.
  //   - Unknown keys are ignored, so newer config files still load in older code.
  //   - Keys missing from the file keep the struct defaults above.
  //
  // Recognized keys: name, wheelbase_meters, rear_track_width_meters,
  // tire_diameter_meters, meters_per_encoder_tick, max_steering_angle_radians,
  // rear_axle_locked (true or false).
  //
  // Throws std::runtime_error if the file cannot be opened, a non-comment line
  // has no '=', a recognized numeric key has a value that is not a finite
  // number, or rear_axle_locked is neither true nor false. (Unlike Python, C++ has no "file not found" exception subclass in
  // the standard library, so std::runtime_error is the conventional choice.)
  static VehicleConfig loadFromFile(const std::string& filePath);
};

}  // namespace rover
