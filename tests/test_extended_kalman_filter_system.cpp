// System-level tests: run ExtendedKalmanFilter over whole simulated drives.
//
// Group A (ExtendedKalmanFilterConsistency): truth generated from exactly the
//   model the filter assumes, so its reported covariance must match its real
//   error statistically (NEES / NIS chi-square tests).
// Group B (ExtendedKalmanFilterRealisticDrive): truth from BicycleModel on a
//   scripted course; checks the design is useful (beats dead reckoning,
//   learns gyro bias, is not overconfident, and survives tire scrub).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>

#include "rover/bicycle_model.hpp"
#include "rover/extended_kalman_filter.hpp"
#include "rover/vehicle_config.hpp"

using rover::BicycleModel;
using rover::ExtendedKalmanFilter;
using rover::ExtendedKalmanFilterNoise;
using rover::VehicleConfig;
using StateVector = ExtendedKalmanFilter::StateVector;
using CovarianceMatrix = ExtendedKalmanFilter::CovarianceMatrix;

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr int kX = ExtendedKalmanFilter::kXIndex;
constexpr int kY = ExtendedKalmanFilter::kYIndex;
constexpr int kHeading = ExtendedKalmanFilter::kHeadingIndex;
constexpr int kSpeed = ExtendedKalmanFilter::kSpeedIndex;
constexpr int kYawRate = ExtendedKalmanFilter::kYawRateIndex;
constexpr int kGyroBias = ExtendedKalmanFilter::kGyroBiasIndex;
constexpr int kStateSize = ExtendedKalmanFilter::kStateSize;

constexpr double kTimeStepSeconds = 0.01;  // 100 Hz

// Standard normal quantile for a two-sided 95% interval (97.5th percentile).
constexpr double kStandardNormalQuantile975 = 1.959963984540054;

// Test-side copy of the (-pi, pi] wrap.
double wrapAngle(double angleRadians) {
  if (angleRadians > -kPi && angleRadians <= kPi) {
    return angleRadians;
  }
  double wrappedAngle = std::fmod(angleRadians + kPi, 2.0 * kPi);
  if (wrappedAngle <= 0.0) {
    wrappedAngle += 2.0 * kPi;
  }
  return wrappedAngle - kPi;
}

// Quantile of the chi-square distribution with `degreesOfFreedom`, via the
// Wilson-Hilferty approximation: (X/k)^(1/3) is close to normal with mean
// 1 - 2/(9k) and variance 2/(9k), so
//     chi2_p(k) ~= k * (1 - 2/(9k) + z_p * sqrt(2/(9k)))^3.
// Relative error is well under 1% for k >= 20, far below the bound widths
// used here.
double chiSquareQuantile(double degreesOfFreedom, double standardNormalQuantile) {
  const double variance = 2.0 / (9.0 * degreesOfFreedom);
  const double cubeRoot = 1.0 - variance + standardNormalQuantile * std::sqrt(variance);
  return degreesOfFreedom * cubeRoot * cubeRoot * cubeRoot;
}

// Bounds for the *average* of `sampleCount` independent chi-square(dof)
// values: their sum is chi-square(dof * sampleCount), divided by sampleCount.
struct AverageChiSquareBounds {
  double lower;
  double upper;
};

AverageChiSquareBounds averageChiSquareBounds95(int degreesOfFreedom, int sampleCount) {
  const double totalDegreesOfFreedom = static_cast<double>(degreesOfFreedom) * sampleCount;
  return AverageChiSquareBounds{
      chiSquareQuantile(totalDegreesOfFreedom, -kStandardNormalQuantile975) / sampleCount,
      chiSquareQuantile(totalDegreesOfFreedom, kStandardNormalQuantile975) / sampleCount};
}

// NEES = e^T * P^-1 * e with e = truth - estimate (heading wrapped). Uses a
// Cholesky-type solve (LDLT) instead of forming P^-1, which is cheaper and
// more accurate.
double normalizedEstimationErrorSquared(const StateVector& trueState,
                                        const ExtendedKalmanFilter& filter) {
  StateVector estimationError = trueState - filter.state();
  estimationError(kHeading) = wrapAngle(estimationError(kHeading));
  return estimationError.dot(filter.covariance().ldlt().solve(estimationError));
}

double fractionInside(const std::vector<double>& values, double lower, double upper) {
  const auto insideCount = std::count_if(values.begin(), values.end(), [&](double value) {
    return value >= lower && value <= upper;
  });
  return static_cast<double>(insideCount) / static_cast<double>(values.size());
}

double mean(const std::vector<double>& values) {
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

}  // namespace

// ===========================================================================
// GROUP A - strict consistency: truth follows exactly the filter's model.
// ===========================================================================

namespace {

constexpr int kConsistencyRunCount = 50;
constexpr int kConsistencyStepCount = 3000;  // 30 s at 100 Hz

// Per-step values summed over all Monte Carlo runs (divide by run count).
struct ConsistencyResults {
  std::vector<double> neesSumPerStep = std::vector<double>(kConsistencyStepCount, 0.0);
  std::vector<double> encoderNisSumPerStep = std::vector<double>(kConsistencyStepCount, 0.0);
  std::vector<double> gyroNisSumPerStep = std::vector<double>(kConsistencyStepCount, 0.0);
};

// Runs once and is shared by the Group A tests (the Monte Carlo is the
// expensive part; the tests only look at different statistics of it).
const ConsistencyResults& consistencyResults() {
  static const ConsistencyResults results = [] {
    ConsistencyResults sums;
    const ExtendedKalmanFilterNoise noise;  // the placeholder defaults
    const double wheelbaseMeters = 0.133;   // unused: no steering update in Group A

    // Q * dt as standard deviations, for drawing the truth's process noise.
    StateVector processSigmaPerStep;
    processSigmaPerStep << noise.processXVariancePerSecond, noise.processYVariancePerSecond,
        noise.processHeadingVariancePerSecond, noise.processSpeedVariancePerSecond,
        noise.processYawRateVariancePerSecond, noise.processGyroBiasVariancePerSecond;
    processSigmaPerStep = (processSigmaPerStep * kTimeStepSeconds).cwiseSqrt();
    const double encoderSigma = std::sqrt(noise.encoderSpeedVariance);
    const double gyroSigma = std::sqrt(noise.gyroYawRateVariance);

    // Initial estimate and P0; the initial truth is drawn from N(estimate, P0)
    // so the filter's starting uncertainty is exactly right.
    StateVector initialEstimate;
    initialEstimate << 0.0, 0.0, 0.0, 0.3, 0.0, 0.0;
    StateVector initialSigma;
    initialSigma << 0.01, 0.01, 0.01, 0.1, 0.05, 0.002;
    const CovarianceMatrix initialCovariance = initialSigma.cwiseAbs2().asDiagonal();

    for (int runIndex = 0; runIndex < kConsistencyRunCount; ++runIndex) {
      std::mt19937 randomGenerator(1000 + runIndex);
      std::normal_distribution<double> unitNormal(0.0, 1.0);
      auto drawUnitNormalVector = [&] {
        StateVector draws;
        for (int index = 0; index < kStateSize; ++index) {
          draws(index) = unitNormal(randomGenerator);
        }
        return draws;
      };

      StateVector trueState = initialEstimate + initialSigma.cwiseProduct(drawUnitNormalVector());
      trueState(kHeading) = wrapAngle(trueState(kHeading));
      ExtendedKalmanFilter filter(wheelbaseMeters, noise, initialEstimate, initialCovariance);

      for (int stepIndex = 0; stepIndex < kConsistencyStepCount; ++stepIndex) {
        // Truth: the filter's own process model f, plus w ~ N(0, Q*dt).
        trueState = ExtendedKalmanFilter::predictedState(trueState, kTimeStepSeconds) +
                    processSigmaPerStep.cwiseProduct(drawUnitNormalVector());
        trueState(kHeading) = wrapAngle(trueState(kHeading));

        filter.predict(kTimeStepSeconds);

        // Only encoder and gyro here. The steering pseudo-measurement asserts
        // z = 0 = yaw_rate - k*speed + noise, but in this truth yaw_rate and
        // speed random-walk independently, so no noise with variance R_steer
        // could make that hold - using it would break the exact model match
        // this group depends on. (Group B exercises it.)
        const double measuredSpeed = trueState(kSpeed) + encoderSigma * unitNormal(randomGenerator);
        const ExtendedKalmanFilter::UpdateResult encoderResult =
            filter.updateEncoderSpeed(measuredSpeed);
        const double measuredYawRate = trueState(kYawRate) + trueState(kGyroBias) +
                                       gyroSigma * unitNormal(randomGenerator);
        const ExtendedKalmanFilter::UpdateResult gyroResult =
            filter.updateGyroYawRate(measuredYawRate);

        sums.neesSumPerStep[stepIndex] += normalizedEstimationErrorSquared(trueState, filter);
        sums.encoderNisSumPerStep[stepIndex] +=
            encoderResult.residual * encoderResult.residual / encoderResult.residualVariance;
        sums.gyroNisSumPerStep[stepIndex] +=
            gyroResult.residual * gyroResult.residual / gyroResult.residualVariance;
      }
    }
    return sums;
  }();
  return results;
}

std::vector<double> averageOverRuns(const std::vector<double>& sumsPerStep, int runCount) {
  std::vector<double> averages(sumsPerStep.size());
  for (std::size_t stepIndex = 0; stepIndex < sumsPerStep.size(); ++stepIndex) {
    averages[stepIndex] = sumsPerStep[stepIndex] / runCount;
  }
  return averages;
}

}  // namespace

// For a consistent filter, each step's NEES is chi-square with 6 dof; the
// average over M independent runs is chi-square(6M)/M. It should fall inside
// the two-sided 95% bounds at ~95% of steps; we require >= 90% (steps are
// correlated in time, so the observed fraction scatters around 95%).
TEST(ExtendedKalmanFilterConsistency, AveragedNeesWithinChiSquareBounds) {
  const std::vector<double> averagedNees =
      averageOverRuns(consistencyResults().neesSumPerStep, kConsistencyRunCount);
  const AverageChiSquareBounds bounds = averageChiSquareBounds95(kStateSize, kConsistencyRunCount);
  const double insideFraction = fractionInside(averagedNees, bounds.lower, bounds.upper);

  std::cout << "[   INFO   ] NEES mean " << mean(averagedNees) << " (ideal " << kStateSize
            << "), 95% band [" << bounds.lower << ", " << bounds.upper << "], inside at "
            << 100.0 * insideFraction << "% of steps\n";
  EXPECT_GE(insideFraction, 0.90);
}

// NIS = residual^2 / S is chi-square with 1 dof per update, so the same test
// applies with dof 1.
TEST(ExtendedKalmanFilterConsistency, AveragedNisWithinChiSquareBounds) {
  const AverageChiSquareBounds perStepBounds = averageChiSquareBounds95(1, kConsistencyRunCount);
  // Innovations of a consistent filter are also white in time, so the grand
  // mean over all runs and steps is chi-square(N)/N with N = runs * steps.
  // That is one single check, so it uses 99.9% bounds (z = 3.29) to keep the
  // false-alarm rate low; still only about +/-1.2% wide.
  const double grandSampleCount =
      static_cast<double>(kConsistencyRunCount) * kConsistencyStepCount;
  const double grandLower = chiSquareQuantile(grandSampleCount, -3.2905) / grandSampleCount;
  const double grandUpper = chiSquareQuantile(grandSampleCount, 3.2905) / grandSampleCount;

  const std::vector<double> encoderNis =
      averageOverRuns(consistencyResults().encoderNisSumPerStep, kConsistencyRunCount);
  const std::vector<double> gyroNis =
      averageOverRuns(consistencyResults().gyroNisSumPerStep, kConsistencyRunCount);

  for (const auto& [sensorName, averagedNis] :
       {std::pair{"encoder", encoderNis}, std::pair{"gyro", gyroNis}}) {
    const double insideFraction =
        fractionInside(averagedNis, perStepBounds.lower, perStepBounds.upper);
    const double grandMean = mean(averagedNis);
    std::cout << "[   INFO   ] " << sensorName << " NIS mean " << grandMean << " (ideal 1, band ["
              << grandLower << ", " << grandUpper << "]), per-step inside at "
              << 100.0 * insideFraction << "% of steps\n";
    EXPECT_GE(insideFraction, 0.90) << sensorName;
    EXPECT_GE(grandMean, grandLower) << sensorName;
    EXPECT_LE(grandMean, grandUpper) << sensorName;
  }
}

// ===========================================================================
// GROUP B - realistic drive: truth from BicycleModel on a scripted course.
// ===========================================================================

namespace {

// The wheelbase is config/vehicle.cfg's (MEASURED); the full lock is the
// 2026-09-22 value (0.246), deliberately not following the cfg (0.204 since
// 2026-10-22). Hardcoded so the expectations below don't silently shift when
// the config is re-measured.
constexpr double kWheelbaseMeters = 0.133;
constexpr double kFullLockSteeringRadians = 0.246;

constexpr double kCruiseSpeedMetersPerSecond = 0.3;
constexpr double kTrueGyroBias = 0.01;  // rad/s, ~0.57 deg/s
constexpr int kRealisticSeedCount = 20;

struct CourseSegment {
  double durationSeconds;
  double speedMetersPerSecond;
  double steeringAngleRadians;  // commanded
};

// Time to turn 90 degrees at full lock and cruise speed (as commanded).
double quarterTurnSeconds() {
  const double commandedYawRate =
      kCruiseSpeedMetersPerSecond * std::tan(kFullLockSteeringRadians) / kWheelbaseMeters;
  return (kPi / 2.0) / commandedYawRate;
}

std::vector<CourseSegment> makeScriptedCourse() {
  return {
      {5.0, 0.0, 0.0},                                                    // stopped
      {5.0, kCruiseSpeedMetersPerSecond, 0.0},                            // straight
      {quarterTurnSeconds(), kCruiseSpeedMetersPerSecond, kFullLockSteeringRadians},   // left
      {5.0, kCruiseSpeedMetersPerSecond, 0.0},                            // straight
      {quarterTurnSeconds(), kCruiseSpeedMetersPerSecond, -kFullLockSteeringRadians},  // right
      {5.0, kCruiseSpeedMetersPerSecond, 0.0},                            // straight
      {5.0, 0.0, 0.0},                                                    // stopped
  };
}

struct RealisticDriveResult {
  std::vector<double> neesPerStep;
  double filterFinalPositionError{0.0};
  double filterFinalHeadingError{0.0};
  double filterMaxHeadingError{0.0};
  double deadReckoningFinalPositionError{0.0};
  double deadReckoningFinalHeadingError{0.0};
  double deadReckoningMaxHeadingError{0.0};
  double finalGyroBiasEstimate{0.0};
  double finalGyroBiasSigma{0.0};
  double turningSpeedErrorSquaredSum{0.0};  // (filter speed - true speed)^2 while turning
  int turningStepCount{0};

  double turningSpeedErrorRms() const {
    return std::sqrt(turningSpeedErrorSquaredSum / turningStepCount);
  }
};

// Drives the scripted course once. The truth steers truthSteeringScale times
// the commanded angle (1.0 = ideal tires, 0.9 = 10% scrub); the filter is
// always told the commanded angle.
RealisticDriveResult simulateRealisticDrive(
    unsigned int seed, double truthSteeringScale,
    const ExtendedKalmanFilterNoise& noise = ExtendedKalmanFilterNoise{}) {
  // Sensor noise is drawn with the same variances the filter assumes, so the
  // only mismatch between truth and filter is the process model.
  const double encoderSigma = std::sqrt(noise.encoderSpeedVariance);
  const double gyroSigma = std::sqrt(noise.gyroYawRateVariance);

  VehicleConfig truthConfig;
  truthConfig.wheelbaseMeters = kWheelbaseMeters;
  truthConfig.maxSteeringAngleRadians = kFullLockSteeringRadians;
  BicycleModel truthModel(truthConfig);

  // The start pose defines the world frame, so x, y, heading are known almost
  // exactly (a tiny variance keeps P invertible for NEES). The truck starts at
  // rest. Bias: 0.02 rad/s sigma, twice the true 0.01, i.e. "some cheap-MEMS
  // offset, not known which".
  CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
  initialCovariance.diagonal() << 1e-6, 1e-6, 1e-6, 0.01, 0.01, 0.02 * 0.02;
  ExtendedKalmanFilter filter(kWheelbaseMeters, noise, StateVector::Zero(), initialCovariance);

  // Dead reckoning: encoder speed and raw gyro, same midpoint-heading formula.
  double deadReckoningX = 0.0;
  double deadReckoningY = 0.0;
  double deadReckoningHeading = 0.0;

  std::mt19937 randomGenerator(seed);
  std::normal_distribution<double> unitNormal(0.0, 1.0);
  RealisticDriveResult result;

  for (const CourseSegment& segment : makeScriptedCourse()) {
    const int segmentStepCount =
        static_cast<int>(std::lround(segment.durationSeconds / kTimeStepSeconds));
    const double trueSteeringAngle = truthSteeringScale * segment.steeringAngleRadians;
    const double trueSpeed = segment.speedMetersPerSecond;
    const double trueYawRate = trueSpeed * std::tan(trueSteeringAngle) / kWheelbaseMeters;

    for (int stepIndex = 0; stepIndex < segmentStepCount; ++stepIndex) {
      truthModel.step(trueSpeed, trueSteeringAngle, kTimeStepSeconds);

      const double measuredSpeed = trueSpeed + encoderSigma * unitNormal(randomGenerator);
      const double measuredYawRate =
          trueYawRate + kTrueGyroBias + gyroSigma * unitNormal(randomGenerator);

      filter.predict(kTimeStepSeconds);
      filter.updateEncoderSpeed(measuredSpeed);
      filter.updateGyroYawRate(measuredYawRate);
      filter.updateSteeringModel(segment.steeringAngleRadians);

      const double deadReckoningMidpointHeading =
          deadReckoningHeading + measuredYawRate * kTimeStepSeconds / 2.0;
      deadReckoningX += measuredSpeed * kTimeStepSeconds * std::cos(deadReckoningMidpointHeading);
      deadReckoningY += measuredSpeed * kTimeStepSeconds * std::sin(deadReckoningMidpointHeading);
      deadReckoningHeading = wrapAngle(deadReckoningHeading + measuredYawRate * kTimeStepSeconds);

      StateVector trueState;
      trueState << truthModel.pose().xMeters, truthModel.pose().yMeters,
          truthModel.pose().headingRadians, trueSpeed, trueYawRate, kTrueGyroBias;
      result.neesPerStep.push_back(normalizedEstimationErrorSquared(trueState, filter));

      const double filterHeadingError =
          std::abs(wrapAngle(filter.state()(kHeading) - trueState(kHeading)));
      const double deadReckoningHeadingError =
          std::abs(wrapAngle(deadReckoningHeading - trueState(kHeading)));
      result.filterMaxHeadingError = std::max(result.filterMaxHeadingError, filterHeadingError);
      result.deadReckoningMaxHeadingError =
          std::max(result.deadReckoningMaxHeadingError, deadReckoningHeadingError);
      result.filterFinalHeadingError = filterHeadingError;
      result.deadReckoningFinalHeadingError = deadReckoningHeadingError;
      result.filterFinalPositionError = std::hypot(filter.state()(kX) - trueState(kX),
                                                   filter.state()(kY) - trueState(kY));
      result.deadReckoningFinalPositionError =
          std::hypot(deadReckoningX - trueState(kX), deadReckoningY - trueState(kY));

      if (trueSpeed != 0.0 && segment.steeringAngleRadians != 0.0) {
        const double speedError = filter.state()(kSpeed) - trueSpeed;
        result.turningSpeedErrorSquaredSum += speedError * speedError;
        ++result.turningStepCount;
      }
    }
  }
  result.finalGyroBiasEstimate = filter.state()(kGyroBias);
  result.finalGyroBiasSigma = std::sqrt(filter.covariance()(kGyroBias, kGyroBias));
  return result;
}

void printDriveSummary(const char* label, const RealisticDriveResult& result) {
  std::cout << "[   INFO   ] " << label << ": heading error final filter "
            << result.filterFinalHeadingError << " vs dead reckoning "
            << result.deadReckoningFinalHeadingError << " rad (max over drive "
            << result.filterMaxHeadingError << " vs " << result.deadReckoningMaxHeadingError
            << "); position error final filter " << result.filterFinalPositionError
            << " vs dead reckoning " << result.deadReckoningFinalPositionError
            << " m; gyro bias " << result.finalGyroBiasEstimate << " +/- "
            << result.finalGyroBiasSigma << " rad/s\n";
}

// Gyro-bias tolerance. While stopped or driving straight, each 100 Hz cycle
// gives the gyro (sigma 0.01 rad/s) and the steering model (sigma_steer_min
// 0.01 rad/s) as two independent looks at true rotation, so bias information
// accrues at 1 / (0.01^2 + 0.01^2) = 5000 (rad/s)^-2 per step. Even the first
// 5 s stop alone (500 steps) gives sigma = 1/sqrt(2.5e6) = 6.3e-4 rad/s, and
// the course has ~20 s of such driving. 0.002 rad/s (20% of the true bias) is
// ~3x that single-stop sigma, so a correctly working filter clears it easily
// while a filter that fails to learn the bias (error ~0.01) cannot.
constexpr double kGyroBiasTolerance = 0.002;

}  // namespace

// ---------------------------------------------------------------------------
// Scrub tolerance for speed, derived from encoder noise alone.
//
// Speed. The steering model is the only update that touches speed besides
// the encoder, and only while turning (k != 0). With the encoder alone, the
// speed estimate is a scalar Kalman filter of a random walk (per-step process
// variance q = q_speed*dt) measured with variance R = R_encoder. Its
// steady-state prior variance solves the Riccati equation
//     Pprior^2 - q*Pprior - q*R = 0  ->  Pprior = (q + sqrt(q^2 + 4qR)) / 2,
// with gain K = Pprior / (Pprior + R). The truth's speed is constant, so the
// error follows e_k = (1-K)*e_{k-1} + K*n_k with n ~ N(0, R), whose
// stationary variance is
//     sigma_e^2 = K^2 * R / (1 - (1-K)^2).
// (Placeholders: K = 0.963, sigma_e = 0.0193 m/s.) If scrub leaves speed
// alone, the RMS speed error over N turning steps is sigma_e, and its
// sampling spread is about sigma_e / sqrt(2N) (errors are nearly white since
// 1-K = 0.04). The tolerance is the 3-sigma upper edge of that:
//     sigma_e * (1 + 3 / sqrt(2N))      (~ +9% for N ~ 550).
// A per-step max check can't do this job: a 0.02 m/s bias hides inside the
// normal 3-4 sigma per-step noise peaks, while the RMS reveals it.
//
// Position is deliberately not checked. A 200-seed sweep showed no position
// bound derived from encoder noise alone is robust: position also absorbs
// cross-track error from heading drift, and the design's own scrub offset
// (~0.8% of distance in turns, see docs/kalman-filter.md). Speed is where an
// over-trusted steering model does its damage; position only integrates it.
// ---------------------------------------------------------------------------

double encoderOnlySpeedErrorSigma(const ExtendedKalmanFilterNoise& noise) {
  const double processVariancePerStep = noise.processSpeedVariancePerSecond * kTimeStepSeconds;
  const double measurementVariance = noise.encoderSpeedVariance;
  const double steadyStatePriorVariance =
      (processVariancePerStep +
       std::sqrt(processVariancePerStep * processVariancePerStep +
                 4.0 * processVariancePerStep * measurementVariance)) /
      2.0;
  const double steadyStateGain =
      steadyStatePriorVariance / (steadyStatePriorVariance + measurementVariance);
  const double retainedFraction = 1.0 - steadyStateGain;
  return std::sqrt(steadyStateGain * steadyStateGain * measurementVariance /
                   (1.0 - retainedFraction * retainedFraction));
}

double turningSpeedErrorRmsTolerance(const ExtendedKalmanFilterNoise& noise,
                                     int turningStepCount) {
  return encoderOnlySpeedErrorSigma(noise) * (1.0 + 3.0 / std::sqrt(2.0 * turningStepCount));
}

TEST(ExtendedKalmanFilterRealisticDrive, BeatsDeadReckoningOnHeadingAndPosition) {
  const RealisticDriveResult result = simulateRealisticDrive(7, 1.0);
  printDriveSummary("ideal tires", result);
  EXPECT_LT(result.filterFinalHeadingError, result.deadReckoningFinalHeadingError);
  EXPECT_LT(result.filterFinalPositionError, result.deadReckoningFinalPositionError);
}

TEST(ExtendedKalmanFilterRealisticDrive, GyroBiasConverges) {
  const RealisticDriveResult result = simulateRealisticDrive(7, 1.0);
  EXPECT_NEAR(result.finalGyroBiasEstimate, kTrueGyroBias, kGyroBiasTolerance);
  // And the filter's own sigma must be honest about it (99.7% bound).
  EXPECT_NEAR(result.finalGyroBiasEstimate, kTrueGyroBias, 3.0 * result.finalGyroBiasSigma);
}

// Only the *upper* NEES bound is enforced. The filter's process model lets
// x, y, heading, speed and yaw_rate random-walk (Q > 0), but the BicycleModel
// truth moves deterministically and speed/steering change only at segment
// boundaries, so the filter's P is larger than its real error: NEES below 6
// (conservative) is expected and acceptable. NEES above the upper bound would
// mean overconfidence - the filter claiming accuracy it doesn't have - which
// is the failure that matters.
TEST(ExtendedKalmanFilterRealisticDrive, NotOverconfident) {
  std::vector<double> neesSumPerStep;
  for (int seedIndex = 0; seedIndex < kRealisticSeedCount; ++seedIndex) {
    const RealisticDriveResult result = simulateRealisticDrive(100 + seedIndex, 1.0);
    neesSumPerStep.resize(result.neesPerStep.size(), 0.0);
    for (std::size_t stepIndex = 0; stepIndex < result.neesPerStep.size(); ++stepIndex) {
      neesSumPerStep[stepIndex] += result.neesPerStep[stepIndex];
    }
  }
  const std::vector<double> averagedNees = averageOverRuns(neesSumPerStep, kRealisticSeedCount);
  const AverageChiSquareBounds bounds = averageChiSquareBounds95(kStateSize, kRealisticSeedCount);
  const double belowUpperFraction = fractionInside(averagedNees, 0.0, bounds.upper);

  std::cout << "[   INFO   ] realistic NEES mean " << mean(averagedNees) << ", max "
            << *std::max_element(averagedNees.begin(), averagedNees.end()) << ", upper 95% bound "
            << bounds.upper << ", below it at " << 100.0 * belowUpperFraction << "% of steps\n";
  EXPECT_GE(belowUpperFraction, 0.90);
}

// Truth turns with 0.9x the commanded angle (tire scrub); the filter is told
// the commanded angle. A design whose heading follows the steering model (the
// 5-state first draft in docs/kalman-filter.md) would over-rotate by 10% of
// each 90-degree turn: 0.1 * pi/2 = 0.157 rad after the left turn, about half
// of dead reckoning's worst error (bias * course length = 0.01 * ~31 s =
// 0.31 rad). Requiring the filter's worst heading error to be under a quarter
// of dead reckoning's rules that out. The final heading error can't be used:
// the right turn undoes the left turn's scrub error, so the max over the
// drive is checked instead.
//
// Note: this does NOT catch sigma_steer_relative being set too small. In this
// 6-state design an over-trusted steering model mostly corrupts the *speed*
// estimate (H_steer has a -k in the speed column), which shows up as position
// error, not heading error. TireScrubDoesNotCorruptSpeed covers that.
TEST(ExtendedKalmanFilterRealisticDrive, TireScrubDoesNotCorruptHeadingOrBias) {
  const RealisticDriveResult result = simulateRealisticDrive(7, 0.9);
  printDriveSummary("10% scrub", result);
  EXPECT_LT(result.filterMaxHeadingError, 0.25 * result.deadReckoningMaxHeadingError);
  EXPECT_NEAR(result.finalGyroBiasEstimate, kTrueGyroBias, kGyroBiasTolerance);
}

// An over-trusted steering model corrupts speed rather than heading (see the
// note above), so the speed error while turning is checked against an
// encoder-noise-only tolerance (derivation above).
TEST(ExtendedKalmanFilterRealisticDrive, TireScrubDoesNotCorruptSpeed) {
  const ExtendedKalmanFilterNoise noise;
  const RealisticDriveResult result = simulateRealisticDrive(7, 0.9, noise);
  const double speedTolerance = turningSpeedErrorRmsTolerance(noise, result.turningStepCount);
  std::cout << "[   INFO   ] 10% scrub: turning speed error RMS " << result.turningSpeedErrorRms()
            << " m/s (tolerance " << speedTolerance << ")\n";
  EXPECT_LE(result.turningSpeedErrorRms(), speedTolerance);
}

// Guards the test above: with sigma_steer_relative 10x too tight, the same
// check must fail, otherwise it isn't measuring anything.
TEST(ExtendedKalmanFilterRealisticDrive, ScrubCheckDetectsOvertrustedSteeringModel) {
  ExtendedKalmanFilterNoise overtrustingNoise;
  overtrustingNoise.steeringSigmaRelative /= 10.0;
  const RealisticDriveResult result = simulateRealisticDrive(7, 0.9, overtrustingNoise);
  const double speedTolerance =
      turningSpeedErrorRmsTolerance(overtrustingNoise, result.turningStepCount);
  std::cout << "[   INFO   ] 10% scrub, sigma_steer_relative/10: turning speed error RMS "
            << result.turningSpeedErrorRms() << " m/s (tolerance " << speedTolerance << ")\n";
  EXPECT_GT(result.turningSpeedErrorRms(), speedTolerance);
}
