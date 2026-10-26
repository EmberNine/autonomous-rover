#include "rover/odometry_fusion.hpp"

#include <cmath>
#include <stdexcept>

namespace rover {

namespace {

// Initial uncertainty once the filter starts: the pose is defined as zero,
// the truck is at rest, and the bias comes from ~200 averaged samples.
constexpr double kStartPoseVariance = 1e-6;
constexpr double kStartSpeedVariance = 1e-4;
constexpr double kStartYawRateVariance = 1e-4;
constexpr double kStartBiasVariance = 1e-8;

}  // namespace

OdometryFusion::OdometryFusion(const OdometryFusionConfig& config) : config_(config) {
  if (!(std::isfinite(config.wheelbaseMeters) && config.wheelbaseMeters > 0.0)) {
    throw std::invalid_argument("wheelbaseMeters must be positive and finite");
  }
  if (!(std::isfinite(config.rearTrackWidthMeters) && config.rearTrackWidthMeters > 0.0)) {
    throw std::invalid_argument("rearTrackWidthMeters must be positive and finite");
  }
}

void OdometryFusion::onEncoder(double stampSeconds,
                               std::optional<double> leftWheelSpeedMetersPerSecond) {
  if (!std::isfinite(stampSeconds) ||
      (leftWheelSpeedMetersPerSecond && !std::isfinite(*leftWheelSpeedMetersPerSecond))) {
    ++counts_.dropped;
    return;
  }
  encoderStamp_ = stampSeconds;
  encoderSpeed_ = leftWheelSpeedMetersPerSecond;
  encoderUsed_ = false;
}

void OdometryFusion::onSteering(double stampSeconds, double steeringAngleRadians) {
  // The core rejects |delta| >= pi/2; anything near that is not a steering angle.
  if (!std::isfinite(stampSeconds) || !std::isfinite(steeringAngleRadians) ||
      std::abs(steeringAngleRadians) >= 1.0) {
    ++counts_.dropped;
    return;
  }
  steeringStamp_ = stampSeconds;
  steeringAngle_ = steeringAngleRadians;
}

bool OdometryFusion::onGyro(double stampSeconds, double yawRateRadiansPerSecond) {
  if (!std::isfinite(stampSeconds) || !std::isfinite(yawRateRadiansPerSecond) ||
      (lastGyroStamp_ && stampSeconds <= *lastGyroStamp_)) {
    ++counts_.dropped;
    return false;
  }
  const std::optional<double> previousStamp = lastGyroStamp_;
  lastGyroStamp_ = stampSeconds;
  if (!filter_) {
    collectRest(stampSeconds, yawRateRadiansPerSecond);
    return false;
  }

  double timeStep = stampSeconds - *previousStamp;
  if (timeStep > config_.maxPredictStepSeconds) {
    ++counts_.gaps;
    timeStep = config_.maxPredictStepSeconds;
  }
  filter_->predict(timeStep);
  filter_->updateGyroYawRate(yawRateRadiansPerSecond);

  if (!encoderUsed_ && encoderSpeed_ &&
      std::abs(stampSeconds - *encoderStamp_) <= config_.encoderMaxAgeSeconds) {
    const double yawRate = filter_->state()(ExtendedKalmanFilter::kYawRateIndex);
    filter_->updateEncoderSpeed(config_.rearAxleLocked
                                    ? *encoderSpeed_
                                    : *encoderSpeed_ + yawRate * config_.rearTrackWidthMeters / 2.0);
    encoderUsed_ = true;
    ++counts_.encoderUpdates;
  }
  // At rest the model says "not turning": wrong whenever the truck is turned by hand or
  // set down facing elsewhere, and the filter then moves the bias and the heading to fit.
  const bool encoderAtRest =
      encoderSpeed_ && std::abs(stampSeconds - *encoderStamp_) <= config_.encoderMaxAgeSeconds &&
      std::abs(*encoderSpeed_) < config_.restSpeedMetersPerSecond;
  if (!encoderAtRest && steeringStamp_ &&
      std::abs(stampSeconds - *steeringStamp_) <= config_.steeringMaxAgeSeconds) {
    filter_->updateSteeringModel(steeringAngle_);
    ++counts_.steeringUpdates;
  }
  ++counts_.cycles;
  return true;
}

void OdometryFusion::collectRest(double stampSeconds, double yawRate) {
  const bool encoderFresh =
      encoderStamp_ && std::abs(stampSeconds - *encoderStamp_) <= config_.encoderMaxAgeSeconds;
  if (!encoderFresh || !encoderSpeed_) {
    return;  // can't tell whether it moves: pause
  }
  if (std::abs(*encoderSpeed_) >= config_.restSpeedMetersPerSecond) {
    restStart_.reset();  // moving: start again
    restSum_ = 0.0;
    restCount_ = 0;
    return;
  }
  if (!restStart_) {
    restStart_ = stampSeconds;
  }
  restSum_ += yawRate;
  ++restCount_;
  if (stampSeconds - *restStart_ < config_.restSecondsForBias) {
    return;
  }
  ExtendedKalmanFilter::StateVector initialState = ExtendedKalmanFilter::StateVector::Zero();
  initialState(ExtendedKalmanFilter::kGyroBiasIndex) = restSum_ / static_cast<double>(restCount_);
  ExtendedKalmanFilter::CovarianceMatrix initialCovariance =
      ExtendedKalmanFilter::CovarianceMatrix::Zero();
  initialCovariance(ExtendedKalmanFilter::kXIndex, ExtendedKalmanFilter::kXIndex) = kStartPoseVariance;
  initialCovariance(ExtendedKalmanFilter::kYIndex, ExtendedKalmanFilter::kYIndex) = kStartPoseVariance;
  initialCovariance(ExtendedKalmanFilter::kHeadingIndex, ExtendedKalmanFilter::kHeadingIndex) =
      kStartPoseVariance;
  initialCovariance(ExtendedKalmanFilter::kSpeedIndex, ExtendedKalmanFilter::kSpeedIndex) =
      kStartSpeedVariance;
  initialCovariance(ExtendedKalmanFilter::kYawRateIndex, ExtendedKalmanFilter::kYawRateIndex) =
      kStartYawRateVariance;
  initialCovariance(ExtendedKalmanFilter::kGyroBiasIndex, ExtendedKalmanFilter::kGyroBiasIndex) =
      kStartBiasVariance;
  filter_.emplace(config_.wheelbaseMeters, config_.noise, initialState, initialCovariance);
}

void OdometryFusion::resetPose() {
  if (!filter_) {
    return;
  }
  ExtendedKalmanFilter::StateVector state = filter_->state();
  ExtendedKalmanFilter::CovarianceMatrix covariance = filter_->covariance();
  for (int index : {ExtendedKalmanFilter::kXIndex, ExtendedKalmanFilter::kYIndex,
                    ExtendedKalmanFilter::kHeadingIndex}) {
    state(index) = 0.0;
    covariance.row(index).setZero();  // no correlation left with the other states
    covariance.col(index).setZero();
    covariance(index, index) = kStartPoseVariance;
  }
  filter_->reset(state, covariance);
}

}  // namespace rover
