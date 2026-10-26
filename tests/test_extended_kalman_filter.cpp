#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "rover/extended_kalman_filter.hpp"

using rover::ExtendedKalmanFilter;
using rover::ExtendedKalmanFilterNoise;
using StateVector = ExtendedKalmanFilter::StateVector;
using CovarianceMatrix = ExtendedKalmanFilter::CovarianceMatrix;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTestWheelbaseMeters = 0.30;

constexpr int kX = ExtendedKalmanFilter::kXIndex;
constexpr int kY = ExtendedKalmanFilter::kYIndex;
constexpr int kHeading = ExtendedKalmanFilter::kHeadingIndex;
constexpr int kSpeed = ExtendedKalmanFilter::kSpeedIndex;
constexpr int kYawRate = ExtendedKalmanFilter::kYawRateIndex;
constexpr int kGyroBias = ExtendedKalmanFilter::kGyroBiasIndex;

StateVector makeState(double xMeters, double yMeters, double headingRadians, double speed,
                      double yawRate, double gyroBias) {
  StateVector state;
  state << xMeters, yMeters, headingRadians, speed, yawRate, gyroBias;
  return state;
}

ExtendedKalmanFilterNoise makeZeroProcessNoise() {
  ExtendedKalmanFilterNoise noise;
  noise.processXVariancePerSecond = 0.0;
  noise.processYVariancePerSecond = 0.0;
  noise.processHeadingVariancePerSecond = 0.0;
  noise.processSpeedVariancePerSecond = 0.0;
  noise.processYawRateVariancePerSecond = 0.0;
  noise.processGyroBiasVariancePerSecond = 0.0;
  return noise;
}

// Test-side copy of the (-pi, pi] wrap, used to compare headings.
double wrapAngle(double angleRadians) {
  double wrappedAngle = std::fmod(angleRadians + kPi, 2.0 * kPi);
  if (wrappedAngle <= 0.0) {
    wrappedAngle += 2.0 * kPi;
  }
  return wrappedAngle - kPi;
}

double smallestEigenvalue(const CovarianceMatrix& covariance) {
  return Eigen::SelfAdjointEigenSolver<CovarianceMatrix>(covariance, Eigen::EigenvaluesOnly)
      .eigenvalues()
      .minCoeff();
}

}  // namespace

// ---------------------------------------------------------------------------
// Jacobian
// ---------------------------------------------------------------------------

TEST(ExtendedKalmanFilterJacobian, MatchesCentralFiniteDifferences) {
  const std::vector<StateVector> testStates = {
      makeState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
      makeState(1.5, -2.0, 0.7, 2.0, 1.9, 0.01),
      makeState(-3.0, 4.0, -1.2, -0.8, 0.5, -0.02),      // reversing
      makeState(0.2, 0.1, kPi - 1e-7, 1.2, -1.5, 0.0),   // heading just below +pi
      makeState(0.2, 0.1, -kPi + 1e-7, -1.0, 1.5, 0.0),  // heading just above -pi
      makeState(10.0, -10.0, kPi, 3.0, 1.9, 0.05),       // heading exactly +pi
      makeState(0.0, 0.0, 2.5, 0.0, -1.9, 0.0),          // turning in place (speed 0)
  };
  const std::vector<double> timeSteps = {0.001, 0.01, 0.1, 0.5};
  const double perturbation = 1e-6;

  for (const StateVector& state : testStates) {
    for (const double timeStepSeconds : timeSteps) {
      const CovarianceMatrix analyticalJacobian =
          ExtendedKalmanFilter::transitionJacobian(state, timeStepSeconds);

      for (int column = 0; column < ExtendedKalmanFilter::kStateSize; ++column) {
        StateVector statePlus = state;
        StateVector stateMinus = state;
        statePlus(column) += perturbation;
        stateMinus(column) -= perturbation;
        StateVector outputDifference =
            ExtendedKalmanFilter::predictedState(statePlus, timeStepSeconds) -
            ExtendedKalmanFilter::predictedState(stateMinus, timeStepSeconds);
        // Near +/-pi the two outputs can land on opposite sides of the wrap;
        // the true difference is the short way around.
        outputDifference(kHeading) = wrapAngle(outputDifference(kHeading));
        const StateVector numericalColumn = outputDifference / (2.0 * perturbation);

        for (int row = 0; row < ExtendedKalmanFilter::kStateSize; ++row) {
          EXPECT_NEAR(analyticalJacobian(row, column), numericalColumn(row), 1e-7)
              << "F(" << row << "," << column << ") at dt=" << timeStepSeconds
              << ", state=" << state.transpose();
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Predict
// ---------------------------------------------------------------------------

TEST(ExtendedKalmanFilterPredict, StraightLineAdvancesBySpeedTimesTime) {
  const double speed = 1.5;
  const double timeStepSeconds = 0.01;
  const int stepCount = 1000;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, makeZeroProcessNoise(),
                              makeState(0.0, 0.0, 0.0, speed, 0.0, 0.0), CovarianceMatrix::Zero());

  for (int stepIndex = 0; stepIndex < stepCount; ++stepIndex) {
    filter.predict(timeStepSeconds);
  }
  EXPECT_NEAR(filter.state()(kX), speed * timeStepSeconds * stepCount, 1e-9);
  EXPECT_NEAR(filter.state()(kY), 0.0, 1e-12);
  EXPECT_NEAR(filter.state()(kHeading), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(filter.state()(kSpeed), speed);
}

TEST(ExtendedKalmanFilterPredict, ConstantYawRateTracesExactCircle) {
  const double speed = 1.0;
  const double yawRate = 1.9;  // the design doc's worst-case turn rate
  const double timeStepSeconds = 0.01;
  const int stepCount = 500;   // 9.5 rad: more than a full circle, crosses +/-pi
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, makeZeroProcessNoise(),
                              makeState(0.0, 0.0, 0.0, speed, yawRate, 0.0),
                              CovarianceMatrix::Zero());

  for (int stepIndex = 0; stepIndex < stepCount; ++stepIndex) {
    filter.predict(timeStepSeconds);
  }

  // Closed form for a circle of radius speed/yawRate starting at the origin facing +x.
  const double elapsedSeconds = timeStepSeconds * stepCount;
  const double turnRadiusMeters = speed / yawRate;
  const double totalHeadingChange = yawRate * elapsedSeconds;
  const double exactX = turnRadiusMeters * std::sin(totalHeadingChange);
  const double exactY = turnRadiusMeters * (1.0 - std::cos(totalHeadingChange));

  // Tolerance from the doc's error analysis: each midpoint step overshoots the
  // arc's chord by at most distance * (yaw_rate*dt)^2 / 24, so after a path of
  // length L the position error is at most L * (yaw_rate*dt)^2 / 24 (~7e-5 m here).
  const double pathLengthMeters = speed * elapsedSeconds;
  const double headingChangePerStep = yawRate * timeStepSeconds;
  const double positionTolerance =
      pathLengthMeters * headingChangePerStep * headingChangePerStep / 24.0 + 1e-12;

  EXPECT_NEAR(filter.state()(kX), exactX, positionTolerance);
  EXPECT_NEAR(filter.state()(kY), exactY, positionTolerance);
  EXPECT_NEAR(filter.state()(kHeading), wrapAngle(totalHeadingChange), 1e-9);
}

TEST(ExtendedKalmanFilterPredict, HeadingStaysNormalizedAcrossPlusMinusPi) {
  for (const double yawRate : {1.0, -1.0}) {
    const double startHeading = yawRate > 0.0 ? 3.0 : -3.0;
    ExtendedKalmanFilter filter(kTestWheelbaseMeters, makeZeroProcessNoise(),
                                makeState(0.0, 0.0, startHeading, 1.0, yawRate, 0.0),
                                CovarianceMatrix::Zero());
    for (int stepIndex = 1; stepIndex <= 100; ++stepIndex) {
      filter.predict(0.01);
      const double heading = filter.state()(kHeading);
      ASSERT_GT(heading, -kPi);
      ASSERT_LE(heading, kPi);
      ASSERT_NEAR(heading, wrapAngle(startHeading + yawRate * 0.01 * stepIndex), 1e-9);
    }
  }
}

TEST(ExtendedKalmanFilterPredict, ConstructorAndResetWrapHeadingIntoRange) {
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{},
                              makeState(0.0, 0.0, 2.0 * kPi + 0.5, 0.0, 0.0, 0.0),
                              CovarianceMatrix::Zero());
  EXPECT_NEAR(filter.state()(kHeading), 0.5, 1e-12);
  filter.reset(makeState(0.0, 0.0, -kPi, 0.0, 0.0, 0.0), CovarianceMatrix::Zero());
  EXPECT_DOUBLE_EQ(filter.state()(kHeading), kPi);  // -pi is outside (-pi, pi]
}

TEST(ExtendedKalmanFilterPredict, ProcessNoiseScalesWithTimeStep) {
  // With speed = yaw_rate = 0, F only couples heading <- yaw_rate, so the
  // speed variance grows by exactly q_speed * dt regardless of how dt is split.
  ExtendedKalmanFilterNoise noise;
  ExtendedKalmanFilter oneStepFilter(kTestWheelbaseMeters, noise, StateVector::Zero(),
                                     CovarianceMatrix::Zero());
  ExtendedKalmanFilter tenStepFilter(kTestWheelbaseMeters, noise, StateVector::Zero(),
                                     CovarianceMatrix::Zero());
  oneStepFilter.predict(0.1);
  for (int stepIndex = 0; stepIndex < 10; ++stepIndex) {
    tenStepFilter.predict(0.01);
  }
  EXPECT_NEAR(oneStepFilter.covariance()(kSpeed, kSpeed),
              noise.processSpeedVariancePerSecond * 0.1, 1e-15);
  EXPECT_NEAR(tenStepFilter.covariance()(kSpeed, kSpeed),
              noise.processSpeedVariancePerSecond * 0.1, 1e-15);
  EXPECT_NEAR(oneStepFilter.covariance()(kGyroBias, kGyroBias),
              noise.processGyroBiasVariancePerSecond * 0.1, 1e-20);
}

// ---------------------------------------------------------------------------
// Updates
// ---------------------------------------------------------------------------

TEST(ExtendedKalmanFilterUpdate, PreciseEncoderPullsSpeedToMeasurement) {
  ExtendedKalmanFilterNoise noise;
  noise.encoderSpeedVariance = 1e-10;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, noise,
                              makeState(0.0, 0.0, 0.0, 0.5, 0.0, 0.0),
                              CovarianceMatrix::Identity());

  const ExtendedKalmanFilter::UpdateResult result = filter.updateEncoderSpeed(2.0);

  EXPECT_DOUBLE_EQ(result.residual, 1.5);
  EXPECT_DOUBLE_EQ(result.residualVariance, 1.0 + 1e-10);  // S = P_speed + R
  EXPECT_NEAR(filter.state()(kSpeed), 2.0, 1e-9);
  EXPECT_LT(filter.covariance()(kSpeed, kSpeed), 2e-10);
  // Uncorrelated states are untouched.
  EXPECT_DOUBLE_EQ(filter.state()(kYawRate), 0.0);
  EXPECT_DOUBLE_EQ(filter.covariance()(kYawRate, kYawRate), 1.0);
}

TEST(ExtendedKalmanFilterUpdate, PreciseGyroPinsYawRatePlusBias) {
  ExtendedKalmanFilterNoise noise;
  noise.gyroYawRateVariance = 1e-10;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, noise,
                              makeState(0.0, 0.0, 0.0, 0.0, 0.1, 0.0),
                              CovarianceMatrix::Identity());

  const ExtendedKalmanFilter::UpdateResult result = filter.updateGyroYawRate(0.5);

  EXPECT_DOUBLE_EQ(result.residual, 0.4);
  EXPECT_DOUBLE_EQ(result.residualVariance, 2.0 + 1e-10);  // S = P_yaw + P_bias + R
  // The gyro sees only the sum, so it cannot tell yaw_rate from bias: with
  // equal prior variances the correction is split evenly between them.
  const double measuredSum = filter.state()(kYawRate) + filter.state()(kGyroBias);
  EXPECT_NEAR(measuredSum, 0.5, 1e-9);
  EXPECT_NEAR(filter.state()(kYawRate), 0.3, 1e-9);
  EXPECT_NEAR(filter.state()(kGyroBias), 0.2, 1e-9);
  // The variance of the *sum* collapses; each part alone stays uncertain.
  const CovarianceMatrix& covariance = filter.covariance();
  const double sumVariance = covariance(kYawRate, kYawRate) + covariance(kGyroBias, kGyroBias) +
                             2.0 * covariance(kYawRate, kGyroBias);
  EXPECT_LT(sumVariance, 2e-10);
  EXPECT_NEAR(covariance(kYawRate, kYawRate), 0.5, 1e-9);
}

TEST(ExtendedKalmanFilterUpdate, SteeringModelResidualAndVarianceMatchDesign) {
  ExtendedKalmanFilterNoise noise;
  const double speed = 1.2;
  const double yawRate = 0.4;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, noise,
                              makeState(0.0, 0.0, 0.0, speed, yawRate, 0.0),
                              CovarianceMatrix::Identity());

  const double steeringAngleRadians = 0.25;
  const ExtendedKalmanFilter::UpdateResult result =
      filter.updateSteeringModel(steeringAngleRadians);

  const double curvature = std::tan(steeringAngleRadians) / kTestWheelbaseMeters;
  const double expectedResidual = 0.0 - (yawRate - curvature * speed);
  const double steeringSigma = noise.steeringSigmaMinRadiansPerSecond +
                               noise.steeringSigmaRelative * std::abs(curvature * speed);
  // H = [0 0 0 -k 1 0] with P = I gives H*P*H^T = k^2 + 1.
  const double expectedResidualVariance =
      curvature * curvature + 1.0 + steeringSigma * steeringSigma;
  EXPECT_NEAR(result.residual, expectedResidual, 1e-12);
  EXPECT_NEAR(result.residualVariance, expectedResidualVariance, 1e-12);
  // The update moves yaw_rate toward k*speed.
  EXPECT_GT(filter.state()(kYawRate), yawRate);
}

TEST(ExtendedKalmanFilterUpdate, CovarianceStaysSymmetricPositiveSemiDefinite) {
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{},
                              StateVector::Zero(), CovarianceMatrix::Identity() * 0.01);

  std::mt19937 randomGenerator(12345);
  std::normal_distribution<double> unitNoise(0.0, 1.0);
  const double timeStepSeconds = 0.01;

  for (int stepIndex = 0; stepIndex < 5000; ++stepIndex) {
    // Weaving drive with a speed ramp, so every Jacobian entry is exercised.
    const double elapsedSeconds = stepIndex * timeStepSeconds;
    const double steeringAngleRadians = 0.4 * std::sin(0.5 * elapsedSeconds);
    const double trueSpeed = 1.5 * std::sin(0.1 * elapsedSeconds);
    const double trueYawRate = trueSpeed * std::tan(steeringAngleRadians) / kTestWheelbaseMeters;

    filter.predict(timeStepSeconds);
    filter.updateEncoderSpeed(trueSpeed + 0.02 * unitNoise(randomGenerator));
    filter.updateGyroYawRate(trueYawRate + 0.03 + 0.01 * unitNoise(randomGenerator));
    filter.updateSteeringModel(steeringAngleRadians);

    const CovarianceMatrix& covariance = filter.covariance();
    const double largestMagnitude = covariance.cwiseAbs().maxCoeff();
    const double asymmetry = (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
    ASSERT_LE(asymmetry, 1e-12 * largestMagnitude) << "step " << stepIndex;
    ASSERT_GE(smallestEigenvalue(covariance), -1e-12 * largestMagnitude) << "step " << stepIndex;
  }
}

TEST(ExtendedKalmanFilterUpdate, StationaryTruckLearnsGyroBias) {
  ExtendedKalmanFilterNoise noise;  // placeholder defaults
  CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
  initialCovariance.diagonal() << 0.0, 0.0, 0.0, 0.1, 0.1, 0.1 * 0.1;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, noise, StateVector::Zero(), initialCovariance);

  const double trueGyroOffset = 0.03;  // rad/s (~1.7 deg/s), typical cheap-MEMS offset
  const double gyroNoiseSigma = std::sqrt(noise.gyroYawRateVariance);
  std::mt19937 randomGenerator(2024);
  std::normal_distribution<double> gyroNoise(0.0, gyroNoiseSigma);

  const int stepCount = 3000;  // 30 s at 100 Hz
  const int averagedStepCount = 1000;
  double yawRateSumOverLastSteps = 0.0;
  for (int stepIndex = 0; stepIndex < stepCount; ++stepIndex) {
    filter.predict(0.01);
    filter.updateEncoderSpeed(0.0);
    filter.updateGyroYawRate(trueGyroOffset + gyroNoise(randomGenerator));
    filter.updateSteeringModel(0.0);
    if (stepIndex >= stepCount - averagedStepCount) {
      yawRateSumOverLastSteps += filter.state()(kYawRate);
    }
  }

  const double gyroBiasSigma = std::sqrt(filter.covariance()(kGyroBias, kGyroBias));
  // The filter should be confident (sigma well under the offset it learned) ...
  EXPECT_LT(gyroBiasSigma, 0.05 * trueGyroOffset);
  // ... and right: within 5% of the offset, and within 4 of its own sigmas.
  EXPECT_NEAR(filter.state()(kGyroBias), trueGyroOffset, 0.05 * trueGyroOffset);
  EXPECT_NEAR(filter.state()(kGyroBias), trueGyroOffset, 4.0 * gyroBiasSigma);
  // yaw_rate was not blamed for the offset. A single yaw_rate estimate jitters
  // step to step (q_yaw_rate is large, so it re-opens every predict), so
  // check its average over the last 10 s rather than the final value.
  const double yawRateSigma = std::sqrt(filter.covariance()(kYawRate, kYawRate));
  EXPECT_NEAR(filter.state()(kYawRate), 0.0, 4.0 * yawRateSigma);
  EXPECT_NEAR(yawRateSumOverLastSteps / averagedStepCount, 0.0, 0.05 * trueGyroOffset);
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

TEST(ExtendedKalmanFilterValidation, ZeroTimeStepLeavesFilterUnchanged) {
  const StateVector initialState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  const CovarianceMatrix initialCovariance = CovarianceMatrix::Identity() * 0.1;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, initialState,
                              initialCovariance);
  filter.predict(0.0);
  EXPECT_EQ(filter.state(), initialState);
  EXPECT_EQ(filter.covariance(), initialCovariance);
}

TEST(ExtendedKalmanFilterValidation, BadTimeStepThrowsAndLeavesFilterUnchanged) {
  const StateVector initialState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  const CovarianceMatrix initialCovariance = CovarianceMatrix::Identity() * 0.1;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, initialState,
                              initialCovariance);

  EXPECT_THROW(filter.predict(-0.01), std::invalid_argument);
  EXPECT_THROW(filter.predict(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(filter.predict(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_EQ(filter.state(), initialState);
  EXPECT_EQ(filter.covariance(), initialCovariance);
}

TEST(ExtendedKalmanFilterValidation, NonFiniteMeasurementsThrowAndLeaveFilterUnchanged) {
  const StateVector initialState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  const CovarianceMatrix initialCovariance = CovarianceMatrix::Identity() * 0.1;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, initialState,
                              initialCovariance);

  const double notANumber = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(filter.updateEncoderSpeed(notANumber), std::invalid_argument);
  EXPECT_THROW(filter.updateEncoderSpeed(-infinity), std::invalid_argument);
  EXPECT_THROW(filter.updateGyroYawRate(notANumber), std::invalid_argument);
  EXPECT_THROW(filter.updateGyroYawRate(infinity), std::invalid_argument);
  EXPECT_THROW(filter.updateSteeringModel(notANumber), std::invalid_argument);
  EXPECT_THROW(filter.updateSteeringModel(infinity), std::invalid_argument);
  EXPECT_THROW(filter.updateSteeringModel(kPi / 2.0), std::invalid_argument);
  EXPECT_THROW(filter.updateSteeringModel(-kPi / 2.0), std::invalid_argument);
  EXPECT_EQ(filter.state(), initialState);
  EXPECT_EQ(filter.covariance(), initialCovariance);
}

TEST(ExtendedKalmanFilterValidation, RejectsInvalidConstructorArguments) {
  const double notANumber = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const ExtendedKalmanFilterNoise validNoise;
  const StateVector validState = StateVector::Zero();
  const CovarianceMatrix validCovariance = CovarianceMatrix::Identity();

  EXPECT_THROW(ExtendedKalmanFilter(0.0, validNoise, validState, validCovariance),
               std::invalid_argument);
  EXPECT_THROW(ExtendedKalmanFilter(-0.3, validNoise, validState, validCovariance),
               std::invalid_argument);
  EXPECT_THROW(ExtendedKalmanFilter(notANumber, validNoise, validState, validCovariance),
               std::invalid_argument);
  EXPECT_THROW(ExtendedKalmanFilter(infinity, validNoise, validState, validCovariance),
               std::invalid_argument);

  // Every field, set to a value that must be rejected.
  std::vector<double ExtendedKalmanFilterNoise::*> nonNegativeFields = {
      &ExtendedKalmanFilterNoise::processXVariancePerSecond,
      &ExtendedKalmanFilterNoise::processYVariancePerSecond,
      &ExtendedKalmanFilterNoise::processHeadingVariancePerSecond,
      &ExtendedKalmanFilterNoise::processSpeedVariancePerSecond,
      &ExtendedKalmanFilterNoise::processYawRateVariancePerSecond,
      &ExtendedKalmanFilterNoise::processGyroBiasVariancePerSecond,
      &ExtendedKalmanFilterNoise::steeringSigmaRelative,
  };
  for (const auto field : nonNegativeFields) {
    for (const double badValue : {-1e-6, notANumber, infinity}) {
      ExtendedKalmanFilterNoise badNoise = validNoise;
      badNoise.*field = badValue;
      EXPECT_THROW(
          ExtendedKalmanFilter(kTestWheelbaseMeters, badNoise, validState, validCovariance),
          std::invalid_argument);
    }
    ExtendedKalmanFilterNoise zeroNoise = validNoise;
    zeroNoise.*field = 0.0;
    EXPECT_NO_THROW(
        ExtendedKalmanFilter(kTestWheelbaseMeters, zeroNoise, validState, validCovariance));
  }

  std::vector<double ExtendedKalmanFilterNoise::*> strictlyPositiveFields = {
      &ExtendedKalmanFilterNoise::encoderSpeedVariance,
      &ExtendedKalmanFilterNoise::gyroYawRateVariance,
      &ExtendedKalmanFilterNoise::steeringSigmaMinRadiansPerSecond,
  };
  for (const auto field : strictlyPositiveFields) {
    for (const double badValue : {0.0, -1e-6, notANumber, infinity}) {
      ExtendedKalmanFilterNoise badNoise = validNoise;
      badNoise.*field = badValue;
      EXPECT_THROW(
          ExtendedKalmanFilter(kTestWheelbaseMeters, badNoise, validState, validCovariance),
          std::invalid_argument);
    }
  }
}

namespace {

// Invalid (state, covariance) pairs that both the constructor and reset() must reject.
std::vector<std::pair<StateVector, CovarianceMatrix>> makeInvalidInitialConditions() {
  const StateVector validState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  const CovarianceMatrix validCovariance = CovarianceMatrix::Identity();
  std::vector<std::pair<StateVector, CovarianceMatrix>> invalidConditions;

  StateVector nonFiniteState = validState;
  nonFiniteState(kSpeed) = std::numeric_limits<double>::quiet_NaN();
  invalidConditions.emplace_back(nonFiniteState, validCovariance);

  CovarianceMatrix nonFiniteCovariance = validCovariance;
  nonFiniteCovariance(kX, kX) = std::numeric_limits<double>::infinity();
  invalidConditions.emplace_back(validState, nonFiniteCovariance);

  CovarianceMatrix asymmetricCovariance = validCovariance;
  asymmetricCovariance(kX, kY) = 0.5;
  invalidConditions.emplace_back(validState, asymmetricCovariance);

  CovarianceMatrix negativeVarianceCovariance = validCovariance;
  negativeVarianceCovariance(kHeading, kHeading) = -0.1;
  invalidConditions.emplace_back(validState, negativeVarianceCovariance);

  // Symmetric with a positive diagonal, but not PSD (correlation > 1).
  CovarianceMatrix indefiniteCovariance = validCovariance;
  indefiniteCovariance(kSpeed, kYawRate) = 2.0;
  indefiniteCovariance(kYawRate, kSpeed) = 2.0;
  invalidConditions.emplace_back(validState, indefiniteCovariance);

  return invalidConditions;
}

}  // namespace

TEST(ExtendedKalmanFilterValidation, ConstructorRejectsInvalidInitialStateOrCovariance) {
  for (const auto& [badState, badCovariance] : makeInvalidInitialConditions()) {
    EXPECT_THROW(ExtendedKalmanFilter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, badState,
                                      badCovariance),
                 std::invalid_argument);
  }
}

TEST(ExtendedKalmanFilterValidation, ConstructorStoresInitialStateAndCovariance) {
  const StateVector initialState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  CovarianceMatrix initialCovariance = CovarianceMatrix::Identity() * 0.1;
  initialCovariance(kSpeed, kYawRate) = 0.02;
  initialCovariance(kYawRate, kSpeed) = 0.02;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, initialState,
                              initialCovariance);
  EXPECT_EQ(filter.state(), initialState);
  EXPECT_EQ(filter.covariance(), initialCovariance);
}

TEST(ExtendedKalmanFilterValidation, ResetReinitializesOrRejectsLeavingFilterUnchanged) {
  const StateVector initialState = makeState(1.0, 2.0, 0.3, 1.0, 0.5, 0.01);
  const CovarianceMatrix initialCovariance = CovarianceMatrix::Identity() * 0.1;
  ExtendedKalmanFilter filter(kTestWheelbaseMeters, ExtendedKalmanFilterNoise{}, initialState,
                              initialCovariance);

  for (const auto& [badState, badCovariance] : makeInvalidInitialConditions()) {
    EXPECT_THROW(filter.reset(badState, badCovariance), std::invalid_argument);
  }
  EXPECT_EQ(filter.state(), initialState);
  EXPECT_EQ(filter.covariance(), initialCovariance);

  // A valid reset after some filtering replaces both completely.
  filter.predict(0.1);
  filter.updateEncoderSpeed(0.7);
  const StateVector newState = makeState(-4.0, 3.0, -1.0, 0.0, 0.0, 0.02);
  const CovarianceMatrix newCovariance = CovarianceMatrix::Identity() * 0.5;
  filter.reset(newState, newCovariance);
  EXPECT_EQ(filter.state(), newState);
  EXPECT_EQ(filter.covariance(), newCovariance);
}
