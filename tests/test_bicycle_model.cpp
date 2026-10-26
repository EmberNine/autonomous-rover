#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "rover/bicycle_model.hpp"

using rover::BicycleModel;
using rover::Pose2D;
using rover::VehicleConfig;

namespace {

constexpr double kPi = 3.14159265358979323846;

VehicleConfig makeTestVehicleConfig(double wheelbaseMeters = 0.30,
                                    double maxSteeringAngleRadians = 0.5) {
  VehicleConfig testConfig;
  testConfig.wheelbaseMeters = wheelbaseMeters;
  testConfig.maxSteeringAngleRadians = maxSteeringAngleRadians;
  return testConfig;
}

}  // namespace

TEST(BicycleModelStraightLine, DrivesAlongPositiveXFromDefaultPose) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(1.0, 0.0, 2.0);
  EXPECT_NEAR(model.pose().xMeters, 2.0, 1e-12);
  EXPECT_NEAR(model.pose().yMeters, 0.0, 1e-12);
  EXPECT_NEAR(model.pose().headingRadians, 0.0, 1e-12);
}

TEST(BicycleModelStraightLine, FollowsCurrentHeading) {
  BicycleModel model(makeTestVehicleConfig());
  model.resetPose(Pose2D{1.0, 2.0, kPi / 2.0});  // facing +y
  model.step(0.5, 0.0, 4.0);
  EXPECT_NEAR(model.pose().xMeters, 1.0, 1e-12);
  EXPECT_NEAR(model.pose().yMeters, 4.0, 1e-12);
}

TEST(BicycleModelStraightLine, NegativeSpeedReverses) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(-1.0, 0.0, 1.0);
  EXPECT_NEAR(model.pose().xMeters, -1.0, 1e-12);
}

TEST(BicycleModelStraightLine, ManySmallStepsEqualOneBigStep) {
  BicycleModel oneBigStepModel(makeTestVehicleConfig());
  BicycleModel manySmallStepsModel(makeTestVehicleConfig());
  oneBigStepModel.step(1.0, 0.0, 3.0);
  for (int stepIndex = 0; stepIndex < 3000; ++stepIndex) {
    manySmallStepsModel.step(1.0, 0.0, 0.001);
  }
  EXPECT_NEAR(oneBigStepModel.pose().xMeters, manySmallStepsModel.pose().xMeters, 1e-9);
}

TEST(BicycleModelTurning, LeftSteeringTurnsCounterClockwise) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(1.0, 0.3, 0.5);
  EXPECT_GT(model.pose().yMeters, 0.0);
  EXPECT_GT(model.pose().headingRadians, 0.0);
}

TEST(BicycleModelTurning, RightSteeringTurnsClockwise) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(1.0, -0.3, 0.5);
  EXPECT_LT(model.pose().yMeters, 0.0);
  EXPECT_LT(model.pose().headingRadians, 0.0);
}

TEST(BicycleModelTurning, LeftAndRightAreMirrorImages) {
  BicycleModel leftTurnModel(makeTestVehicleConfig());
  BicycleModel rightTurnModel(makeTestVehicleConfig());
  leftTurnModel.step(1.0, 0.3, 0.5);
  rightTurnModel.step(1.0, -0.3, 0.5);
  EXPECT_NEAR(leftTurnModel.pose().xMeters, rightTurnModel.pose().xMeters, 1e-12);
  EXPECT_NEAR(leftTurnModel.pose().yMeters, -rightTurnModel.pose().yMeters, 1e-12);
  EXPECT_NEAR(leftTurnModel.pose().headingRadians, -rightTurnModel.pose().headingRadians, 1e-12);
}

TEST(BicycleModelTurning, MatchesAnalyticCircularArc) {
  const double wheelbaseMeters = 0.3;
  const double steeringAngleRadians = 0.3;
  const double speedMetersPerSecond = 1.0;
  const double durationSeconds = 1.5;
  BicycleModel model(makeTestVehicleConfig(wheelbaseMeters));
  model.step(speedMetersPerSecond, steeringAngleRadians, durationSeconds);

  const double turnRadiusMeters = wheelbaseMeters / std::tan(steeringAngleRadians);
  const double totalHeadingChange = speedMetersPerSecond / turnRadiusMeters * durationSeconds;
  EXPECT_NEAR(model.pose().xMeters, turnRadiusMeters * std::sin(totalHeadingChange), 1e-9);
  EXPECT_NEAR(model.pose().yMeters, turnRadiusMeters * (1.0 - std::cos(totalHeadingChange)), 1e-9);
  EXPECT_NEAR(model.pose().headingRadians, totalHeadingChange, 1e-9);
}

TEST(BicycleModelTurning, FullCircleReturnsToStart) {
  const double wheelbaseMeters = 0.3;
  const double steeringAngleRadians = 0.25;
  const double speedMetersPerSecond = 0.8;
  const double yawRate = speedMetersPerSecond / wheelbaseMeters * std::tan(steeringAngleRadians);
  const double fullCircleSeconds = 2.0 * kPi / yawRate;

  BicycleModel model(makeTestVehicleConfig(wheelbaseMeters));
  const int stepCount = 1000;
  for (int stepIndex = 0; stepIndex < stepCount; ++stepIndex) {
    model.step(speedMetersPerSecond, steeringAngleRadians, fullCircleSeconds / stepCount);
  }
  EXPECT_NEAR(model.pose().xMeters, 0.0, 1e-9);
  EXPECT_NEAR(model.pose().yMeters, 0.0, 1e-9);
}

TEST(BicycleModelTurning, TinySteeringIsContinuousWithStraightLine) {
  BicycleModel nearlyStraightModel(makeTestVehicleConfig());
  BicycleModel straightModel(makeTestVehicleConfig());
  nearlyStraightModel.step(1.0, 1e-9, 1.0);
  straightModel.step(1.0, 0.0, 1.0);
  // A 1e-9 rad steering angle genuinely curves the path by ~1.7e-9 m over
  // 1 s, so the tolerance must be a bit looser than that; the point of this
  // test is that nothing blows up (e.g. 0/0) as steering approaches zero.
  EXPECT_NEAR(nearlyStraightModel.pose().xMeters, straightModel.pose().xMeters, 1e-8);
  EXPECT_NEAR(nearlyStraightModel.pose().yMeters, straightModel.pose().yMeters, 1e-8);
}

TEST(BicycleModelTurning, HeadingStaysWithinPlusMinusPi) {
  BicycleModel model(makeTestVehicleConfig());
  for (int stepIndex = 0; stepIndex < 500; ++stepIndex) {
    model.step(1.0, 0.4, 0.1);
    ASSERT_GT(model.pose().headingRadians, -kPi - 1e-12);
    ASSERT_LE(model.pose().headingRadians, kPi + 1e-12);
  }
}

TEST(BicycleModelSteeringClamp, SteeringBeyondLimitBehavesLikeTheLimit) {
  BicycleModel overLimitModel(makeTestVehicleConfig(0.3, 0.4));
  BicycleModel atLimitModel(makeTestVehicleConfig(0.3, 0.4));
  overLimitModel.step(1.0, 1.2, 1.0);
  atLimitModel.step(1.0, 0.4, 1.0);
  EXPECT_NEAR(overLimitModel.pose().xMeters, atLimitModel.pose().xMeters, 1e-12);
  EXPECT_NEAR(overLimitModel.pose().yMeters, atLimitModel.pose().yMeters, 1e-12);
  EXPECT_NEAR(overLimitModel.pose().headingRadians, atLimitModel.pose().headingRadians, 1e-12);
}

TEST(BicycleModelSteeringClamp, NegativeSteeringBeyondLimitIsAlsoClamped) {
  BicycleModel overLimitModel(makeTestVehicleConfig(0.3, 0.4));
  BicycleModel atLimitModel(makeTestVehicleConfig(0.3, 0.4));
  overLimitModel.step(1.0, -3.0, 1.0);
  atLimitModel.step(1.0, -0.4, 1.0);
  EXPECT_NEAR(overLimitModel.pose().yMeters, atLimitModel.pose().yMeters, 1e-12);
  EXPECT_NEAR(overLimitModel.pose().headingRadians, atLimitModel.pose().headingRadians, 1e-12);
}

TEST(BicycleModelSteeringClamp, SteeringWithinLimitIsNotAltered) {
  BicycleModel gentleModel(makeTestVehicleConfig(0.3, 0.4));
  BicycleModel harderModel(makeTestVehicleConfig(0.3, 0.4));
  gentleModel.step(1.0, 0.1, 1.0);
  harderModel.step(1.0, 0.4, 1.0);
  EXPECT_LT(gentleModel.pose().headingRadians, harderModel.pose().headingRadians);
}

TEST(BicycleModelTimeStep, ZeroTimeStepLeavesPoseUnchanged) {
  BicycleModel model(makeTestVehicleConfig());
  model.resetPose(Pose2D{1.0, 1.0, 0.3});
  model.step(1.0, 0.3, 0.0);
  EXPECT_DOUBLE_EQ(model.pose().xMeters, 1.0);
  EXPECT_DOUBLE_EQ(model.pose().yMeters, 1.0);
  EXPECT_DOUBLE_EQ(model.pose().headingRadians, 0.3);
}

TEST(BicycleModelTimeStep, NegativeTimeStepThrowsAndLeavesPoseUnchanged) {
  BicycleModel model(makeTestVehicleConfig());
  model.resetPose(Pose2D{1.0, 1.0, 0.3});
  EXPECT_THROW(model.step(1.0, 0.3, -0.1), std::invalid_argument);
  EXPECT_DOUBLE_EQ(model.pose().xMeters, 1.0);
  EXPECT_DOUBLE_EQ(model.pose().yMeters, 1.0);
  EXPECT_DOUBLE_EQ(model.pose().headingRadians, 0.3);
}

TEST(BicycleModelTimeStep, NonFiniteArgumentsThrow) {
  BicycleModel model(makeTestVehicleConfig());
  const double notANumber = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(model.step(1.0, 0.0, notANumber), std::invalid_argument);
  EXPECT_THROW(model.step(1.0, 0.0, infinity), std::invalid_argument);
  EXPECT_THROW(model.step(notANumber, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(model.step(1.0, notANumber, 0.1), std::invalid_argument);
}

TEST(BicycleModelSetup, ResetPoseSetsPoseAndWrapsHeading) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(1.0, 0.2, 1.0);
  model.resetPose(Pose2D{3.0, -1.0, 2.0 * kPi + 0.5});
  EXPECT_NEAR(model.pose().xMeters, 3.0, 1e-12);
  EXPECT_NEAR(model.pose().yMeters, -1.0, 1e-12);
  EXPECT_NEAR(model.pose().headingRadians, 0.5, 1e-12);
}

TEST(BicycleModelSetup, ResetPoseWithNoArgumentReturnsToOrigin) {
  BicycleModel model(makeTestVehicleConfig());
  model.step(1.0, 0.2, 1.0);
  model.resetPose();
  EXPECT_DOUBLE_EQ(model.pose().xMeters, 0.0);
  EXPECT_DOUBLE_EQ(model.pose().yMeters, 0.0);
  EXPECT_DOUBLE_EQ(model.pose().headingRadians, 0.0);
}

TEST(BicycleModelSetup, RejectsInvalidVehicleConfig) {
  EXPECT_THROW(BicycleModel(makeTestVehicleConfig(0.0)), std::invalid_argument);
  EXPECT_THROW(BicycleModel(makeTestVehicleConfig(-0.3)), std::invalid_argument);
  EXPECT_THROW(BicycleModel(makeTestVehicleConfig(0.3, 0.0)), std::invalid_argument);
  EXPECT_THROW(BicycleModel(makeTestVehicleConfig(0.3, kPi / 2.0)), std::invalid_argument);
}
