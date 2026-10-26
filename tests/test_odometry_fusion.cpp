#include "rover/odometry_fusion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

using rover::ExtendedKalmanFilter;
using rover::OdometryFusion;
using rover::OdometryFusionConfig;

namespace {

constexpr double kWheelbase = 0.133;   // config/vehicle.cfg
constexpr double kRearTrack = 0.081;   // config/vehicle.cfg
constexpr double kPi = 3.14159265358979323846;
constexpr double kStep = 0.01;         // the bridge's 100 Hz STATE

OdometryFusionConfig config() {
  OdometryFusionConfig fusionConfig;
  fusionConfig.wheelbaseMeters = kWheelbase;
  fusionConfig.rearTrackWidthMeters = kRearTrack;
  fusionConfig.rearAxleLocked = true;     // config/vehicle.cfg: the truck
  fusionConfig.noise.gyroYawRateVariance = 1e-6;
  fusionConfig.noise.processGyroBiasVariancePerSecond = 1e-9;
  return fusionConfig;
}

// One STATE frame: encoder and gyro with the same stamp, as the bridge sends them.
bool frame(OdometryFusion& fusion, double stamp, double leftSpeed, double gyro) {
  fusion.onEncoder(stamp, leftSpeed);
  return fusion.onGyro(stamp, gyro);
}

// At rest for `seconds` from `start`: returns the next stamp.
double rest(OdometryFusion& fusion, double start, double seconds, double gyro = 0.002) {
  double stamp = start;
  for (; stamp < start + seconds + 1e-9; stamp += kStep) {
    frame(fusion, stamp, 0.0, gyro);
  }
  return stamp;
}

double state(const OdometryFusion& fusion, int index) { return fusion.filter().state()(index); }

}  // namespace

TEST(OdometryFusion, RejectsAMissingWheelbaseOrTrack) {
  OdometryFusionConfig bad = config();
  bad.rearTrackWidthMeters = 0.0;
  EXPECT_THROW(OdometryFusion{bad}, std::invalid_argument);
  bad = config();
  bad.wheelbaseMeters = std::nan("");
  EXPECT_THROW(OdometryFusion{bad}, std::invalid_argument);
}

TEST(OdometryFusion, StartsAfterTwoSecondsAtRestWithTheMeanGyroAsBias) {
  OdometryFusion fusion(config());
  double stamp = 100.0;
  for (; stamp < 101.99; stamp += kStep) {
    EXPECT_FALSE(frame(fusion, stamp, 0.0, (std::lround(stamp * 100) % 2) ? 0.003 : 0.001));
  }
  EXPECT_FALSE(fusion.started());
  frame(fusion, 102.0, 0.0, 0.002);
  ASSERT_TRUE(fusion.started());
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kGyroBiasIndex), 0.002, 1e-4);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kXIndex), 0.0);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kHeadingIndex), 0.0);
  EXPECT_TRUE(frame(fusion, 102.01, 0.0, 0.002));
}

TEST(OdometryFusion, MotionRestartsTheRestCountAndAnInvalidEncoderPausesIt) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 1.5);
  frame(fusion, stamp, 0.05, 0.0);  // pushed: start again
  stamp = rest(fusion, stamp + kStep, 1.5);
  EXPECT_FALSE(fusion.started());
  for (int i = 0; i < 100; ++i, stamp += kStep) {  // 1 s of invalid estimates: paused, not counted
    fusion.onEncoder(stamp, std::nullopt);
    fusion.onGyro(stamp, 0.5);
  }
  EXPECT_FALSE(fusion.started());
  rest(fusion, stamp, 0.5);
  ASSERT_TRUE(fusion.started());
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kGyroBiasIndex), 0.002, 1e-9);  // the 0.5 rad/s never counted
}

TEST(OdometryFusion, NoEncoderSampleMeansNoStart) {
  OdometryFusion fusion(config());
  for (double stamp = 100.0; stamp < 105.0; stamp += kStep) {
    fusion.onGyro(stamp, 0.0);
  }
  EXPECT_FALSE(fusion.started());
}

TEST(OdometryFusion, DrivesStraightTwoMetres) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  for (int i = 0; i < 2000; ++i, stamp += kStep) {  // 20 s at 0.1 m/s
    frame(fusion, stamp, 0.1, 0.0);
  }
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kXIndex), 2.0, 0.02);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kYIndex), 0.0, 1e-3);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kSpeedIndex), 0.1, 1e-3);
  EXPECT_EQ(fusion.counts().encoderUpdates, fusion.counts().cycles);
}

// Open differential: in a left turn the rear-left wheel is on the inside, so it
// reads v - w * track / 2; the filter's speed must be the centre's v.
TEST(OdometryFusion, OpenDiffTheLeftWheelSpeedIsCorrectedToTheAxleCentreBothWays) {
  for (const double direction : {1.0, -1.0}) {
    OdometryFusionConfig openDiff = config();
    openDiff.rearAxleLocked = false;
    OdometryFusion fusion(openDiff);
    double stamp = rest(fusion, 100.0, 2.0, 0.0);
    const double centreSpeed = 0.1;
    const double yawRate = direction * 0.15;
    const double leftSpeed = centreSpeed - yawRate * kRearTrack / 2.0;
    const double steering = std::atan(kWheelbase * yawRate / centreSpeed);
    for (int i = 0; i < 1000; ++i, stamp += kStep) {
      fusion.onSteering(stamp, steering);
      frame(fusion, stamp, leftSpeed, yawRate);
    }
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kSpeedIndex), centreSpeed, 5e-4) << direction;
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kYawRateIndex), yawRate, 1e-3) << direction;
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kHeadingIndex), yawRate * 10.0, 0.01) << direction;
  }
}

// Locked rear axle: both wheels turn together (and scrub), so the encoder
// wheel reads the centre's v in a turn, and the filter takes it as is.
TEST(OdometryFusion, LockedAxleTheLeftWheelSpeedIsTheAxleCentresBothWays) {
  for (const double direction : {1.0, -1.0}) {
    OdometryFusion fusion(config());
    double stamp = rest(fusion, 100.0, 2.0, 0.0);
    const double centreSpeed = 0.1;
    const double yawRate = direction * 0.15;
    const double steering = std::atan(kWheelbase * yawRate / centreSpeed);
    for (int i = 0; i < 1000; ++i, stamp += kStep) {
      fusion.onSteering(stamp, steering);
      frame(fusion, stamp, centreSpeed, yawRate);
    }
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kSpeedIndex), centreSpeed, 5e-4) << direction;
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kYawRateIndex), yawRate, 1e-3) << direction;
    EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kHeadingIndex), yawRate * 10.0, 0.01) << direction;
  }
}

TEST(OdometryFusion, AHalfCircleAtFullLockEndsHeadingBack) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  const double lock = 0.204;
  const double centreSpeed = 0.05;
  const double yawRate = centreSpeed * std::tan(lock) / kWheelbase;
  const double seconds = kPi / yawRate;
  const int steps = static_cast<int>(std::lround(seconds / kStep));
  for (int i = 0; i < steps; ++i, stamp += kStep) {
    fusion.onSteering(stamp, lock);
    frame(fusion, stamp, centreSpeed, yawRate);           // locked axle: the centre's speed
  }
  const double radius = kWheelbase / std::tan(lock);
  EXPECT_NEAR(std::abs(state(fusion, ExtendedKalmanFilter::kHeadingIndex)), kPi, 0.02);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kXIndex), 0.0, 0.02);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kYIndex), 2.0 * radius, 0.02);
}

TEST(OdometryFusion, EachEncoderSampleIsUsedOnceAndOldOnesNotAtAll) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  const long before = fusion.counts().encoderUpdates;
  fusion.onEncoder(stamp, 0.1);
  fusion.onGyro(stamp, 0.0);
  fusion.onGyro(stamp + kStep, 0.0);                      // no new encoder sample
  EXPECT_EQ(fusion.counts().encoderUpdates, before + 1);
  fusion.onEncoder(stamp + kStep, 0.1);
  fusion.onGyro(stamp + 0.05, 0.0);                       // 40 ms newer than the encoder: too old
  EXPECT_EQ(fusion.counts().encoderUpdates, before + 1);
  fusion.onEncoder(stamp + 0.06, std::nullopt);           // invalid
  fusion.onGyro(stamp + 0.06, 0.0);
  EXPECT_EQ(fusion.counts().encoderUpdates, before + 1);
}

TEST(OdometryFusion, SteeringIsUsedOnlyWhileFresh) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  fusion.onSteering(stamp, 0.1);
  frame(fusion, stamp, 0.1, 0.0);
  frame(fusion, stamp + 0.3, 0.1, 0.0);                   // 0.3 s: still fresh
  EXPECT_EQ(fusion.counts().steeringUpdates, 2);
  frame(fusion, stamp + 0.31, 0.1, 0.0);                  // stale (or a steering FAULT: none sent)
  EXPECT_EQ(fusion.counts().steeringUpdates, 2);
}

TEST(OdometryFusion, SteeringIsNotUsedWhileTheEncoderReadsAtRest) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  fusion.onSteering(stamp, 0.0);
  frame(fusion, stamp, 0.004, 0.0);                       // below restSpeedMetersPerSecond
  EXPECT_EQ(fusion.counts().steeringUpdates, 0);
  frame(fusion, stamp + kStep, 0.006, 0.0);               // rolling
  EXPECT_EQ(fusion.counts().steeringUpdates, 1);
}

// The 2026-10-25 bench: the truck turned 90 deg CCW by hand at v = 0, steering
// centred (TIMED_OUT). With the model applied at rest the filter showed 0.75 of
// the turn and then decayed ~0.015 rad/s (the bias had absorbed the rotation).
TEST(OdometryFusion, AHandTurnAtRestKeepsItsHeadingAndTheBias) {
  constexpr double kBias = -0.0026;
  constexpr double kSeconds = 1.25;               // A sin^2(pi t / T): peak 2.51 rad/s, 90 deg in all
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 30.0, kBias);
  fusion.resetPose();
  stamp = rest(fusion, stamp, 12.0, kBias);
  for (double t = 0.0; t < kSeconds; t += kStep, stamp += kStep) {
    fusion.onSteering(stamp, 0.0);
    frame(fusion, stamp, 0.0, kPi / kSeconds * std::pow(std::sin(kPi * t / kSeconds), 2) + kBias);
  }
  const double afterTurn = state(fusion, ExtendedKalmanFilter::kHeadingIndex);
  EXPECT_NEAR(afterTurn, kPi / 2.0, 0.01);
  for (int i = 0; i < 1200; ++i, stamp += kStep) {  // 12 s at rest
    if (i % 10 == 0) {
      fusion.onSteering(stamp, 0.0);
    }
    frame(fusion, stamp, 0.0, kBias);
  }
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kHeadingIndex), afterTurn, 0.005);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kGyroBiasIndex), kBias, 1e-4);
}

TEST(OdometryFusion, AGyroGapHoldsThePoseInsteadOfExtrapolating) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  for (int i = 0; i < 300; ++i, stamp += kStep) {
    frame(fusion, stamp, 0.1, 0.0);
  }
  const double before = state(fusion, ExtendedKalmanFilter::kXIndex);
  frame(fusion, stamp + 1.0, 0.1, 0.0);                   // 1 s of lost frames
  EXPECT_EQ(fusion.counts().gaps, 1);
  EXPECT_NEAR(state(fusion, ExtendedKalmanFilter::kXIndex) - before, 0.1 * 0.05, 1e-3);   // one capped step
}

TEST(OdometryFusion, NonFiniteInputsAndBackwardStampsAreDropped) {
  OdometryFusion fusion(config());
  double stamp = rest(fusion, 100.0, 2.0, 0.0);
  const long cycles = fusion.counts().cycles;
  EXPECT_FALSE(fusion.onGyro(stamp, std::nan("")));
  EXPECT_FALSE(fusion.onGyro(stamp - 1.0, 0.0));          // backwards
  fusion.onEncoder(stamp, std::nan(""));
  fusion.onSteering(stamp, std::nan(""));
  fusion.onSteering(stamp, 1.6);                           // not a steering angle
  EXPECT_EQ(fusion.counts().dropped, 5);
  EXPECT_EQ(fusion.counts().cycles, cycles);
  EXPECT_TRUE(fusion.onGyro(stamp + kStep, 0.0));
}

TEST(OdometryFusion, ResetPoseKeepsSpeedAndBias) {
  OdometryFusion fusion(config());
  fusion.resetPose();                                      // before the start: nothing
  double stamp = rest(fusion, 100.0, 2.0, 0.003);
  for (int i = 0; i < 500; ++i, stamp += kStep) {
    fusion.onSteering(stamp, 0.1);
    frame(fusion, stamp, 0.1, 0.003 + 0.1 * std::tan(0.1) / kWheelbase);
  }
  const double speed = state(fusion, ExtendedKalmanFilter::kSpeedIndex);
  const double bias = state(fusion, ExtendedKalmanFilter::kGyroBiasIndex);
  ASSERT_GT(state(fusion, ExtendedKalmanFilter::kXIndex), 0.3);
  fusion.resetPose();
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kXIndex), 0.0);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kYIndex), 0.0);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kHeadingIndex), 0.0);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kSpeedIndex), speed);
  EXPECT_DOUBLE_EQ(state(fusion, ExtendedKalmanFilter::kGyroBiasIndex), bias);
  EXPECT_DOUBLE_EQ(fusion.filter().covariance()(ExtendedKalmanFilter::kXIndex, ExtendedKalmanFilter::kSpeedIndex),
                   0.0);
}
