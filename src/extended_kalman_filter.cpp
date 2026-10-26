#include "rover/extended_kalman_filter.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Eigenvalues>

#include "angle_math.hpp"

namespace rover {

namespace {

void requirePositiveAndFinite(double value, const std::string& name) {
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(name + " must be positive and finite");
  }
}

void requireNonNegativeAndFinite(double value, const std::string& name) {
  if (!(value >= 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(name + " must be non-negative and finite");
  }
}

void requireFinite(double value, const std::string& name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be a finite number");
  }
}

}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(double wheelbaseMeters,
                                           const ExtendedKalmanFilterNoise& noise,
                                           const StateVector& initialState,
                                           const CovarianceMatrix& initialCovariance)
    : wheelbaseMeters_(wheelbaseMeters),
      noise_(noise),
      state_(initialState),
      covariance_(initialCovariance) {
  requirePositiveAndFinite(wheelbaseMeters_, "wheelbaseMeters");

  requireNonNegativeAndFinite(noise_.processXVariancePerSecond, "processXVariancePerSecond");
  requireNonNegativeAndFinite(noise_.processYVariancePerSecond, "processYVariancePerSecond");
  requireNonNegativeAndFinite(noise_.processHeadingVariancePerSecond,
                              "processHeadingVariancePerSecond");
  requireNonNegativeAndFinite(noise_.processSpeedVariancePerSecond,
                              "processSpeedVariancePerSecond");
  requireNonNegativeAndFinite(noise_.processYawRateVariancePerSecond,
                              "processYawRateVariancePerSecond");
  requireNonNegativeAndFinite(noise_.processGyroBiasVariancePerSecond,
                              "processGyroBiasVariancePerSecond");

  // These three must be strictly positive: they are the only thing keeping
  // S = H*P*H^T + R away from zero when P is (nearly) zero in that direction.
  requirePositiveAndFinite(noise_.encoderSpeedVariance, "encoderSpeedVariance");
  requirePositiveAndFinite(noise_.gyroYawRateVariance, "gyroYawRateVariance");
  requirePositiveAndFinite(noise_.steeringSigmaMinRadiansPerSecond,
                           "steeringSigmaMinRadiansPerSecond");
  requireNonNegativeAndFinite(noise_.steeringSigmaRelative, "steeringSigmaRelative");

  validateStateAndCovariance(state_, covariance_);
  state_(kHeadingIndex) = wrapAngleToPlusMinusPi(state_(kHeadingIndex));
}

void ExtendedKalmanFilter::validateStateAndCovariance(
    const StateVector& candidateState, const CovarianceMatrix& candidateCovariance) {
  if (!candidateState.allFinite() || !candidateCovariance.allFinite()) {
    throw std::invalid_argument("initial state and covariance must be finite");
  }
  // isApprox is relative (scaled by the matrix norm), so it works for any units.
  if (!candidateCovariance.isApprox(candidateCovariance.transpose())) {
    throw std::invalid_argument("initial covariance must be symmetric");
  }
  const double smallestEigenvalue =
      Eigen::SelfAdjointEigenSolver<CovarianceMatrix>(candidateCovariance, Eigen::EigenvaluesOnly)
          .eigenvalues()
          .minCoeff();
  const double roundingTolerance = 1e-12 * candidateCovariance.cwiseAbs().maxCoeff();
  if (smallestEigenvalue < -roundingTolerance) {
    throw std::invalid_argument("initial covariance must be positive semi-definite");
  }
}

void ExtendedKalmanFilter::reset(const StateVector& initialState,
                                 const CovarianceMatrix& initialCovariance) {
  validateStateAndCovariance(initialState, initialCovariance);
  state_ = initialState;
  state_(kHeadingIndex) = wrapAngleToPlusMinusPi(state_(kHeadingIndex));
  covariance_ = initialCovariance;
}

ExtendedKalmanFilter::StateVector ExtendedKalmanFilter::predictedState(
    const StateVector& currentState, double timeStepSeconds) {
  const double heading = currentState(kHeadingIndex);
  const double speed = currentState(kSpeedIndex);
  const double yawRate = currentState(kYawRateIndex);

  // [heading_mid] Driving along the average heading of the step makes the
  // straight segment match the true arc's chord to O((yaw_rate*dt)^2).
  const double midpointHeading = heading + yawRate * timeStepSeconds / 2.0;

  StateVector nextState = currentState;  // speed, yaw_rate, gyro_bias carry over unchanged
  nextState(kXIndex) += speed * timeStepSeconds * std::cos(midpointHeading);
  nextState(kYIndex) += speed * timeStepSeconds * std::sin(midpointHeading);
  nextState(kHeadingIndex) = wrapAngleToPlusMinusPi(heading + yawRate * timeStepSeconds);
  return nextState;
}

ExtendedKalmanFilter::CovarianceMatrix ExtendedKalmanFilter::transitionJacobian(
    const StateVector& currentState, double timeStepSeconds) {
  const double speed = currentState(kSpeedIndex);
  const double midpointHeading =
      currentState(kHeadingIndex) + currentState(kYawRateIndex) * timeStepSeconds / 2.0;
  const double cosMidpoint = std::cos(midpointHeading);
  const double sinMidpoint = std::sin(midpointHeading);
  const double halfTimeStep = timeStepSeconds / 2.0;

  CovarianceMatrix jacobian = CovarianceMatrix::Identity();
  jacobian(kXIndex, kHeadingIndex) = -speed * timeStepSeconds * sinMidpoint;
  jacobian(kXIndex, kSpeedIndex) = timeStepSeconds * cosMidpoint;
  jacobian(kXIndex, kYawRateIndex) = -speed * timeStepSeconds * sinMidpoint * halfTimeStep;
  jacobian(kYIndex, kHeadingIndex) = speed * timeStepSeconds * cosMidpoint;
  jacobian(kYIndex, kSpeedIndex) = timeStepSeconds * sinMidpoint;
  jacobian(kYIndex, kYawRateIndex) = speed * timeStepSeconds * cosMidpoint * halfTimeStep;
  jacobian(kHeadingIndex, kYawRateIndex) = timeStepSeconds;
  return jacobian;
}

void ExtendedKalmanFilter::predict(double timeStepSeconds) {
  // Same rules as BicycleModel::step(): validate before touching anything.
  requireFinite(timeStepSeconds, "timeStepSeconds");
  if (timeStepSeconds < 0.0) {
    throw std::invalid_argument("timeStepSeconds must not be negative");
  }
  if (timeStepSeconds == 0.0) {
    return;
  }

  // F must be evaluated at the state *before* this step.
  const CovarianceMatrix jacobian = transitionJacobian(state_, timeStepSeconds);

  StateVector processVariancePerSecond;
  processVariancePerSecond << noise_.processXVariancePerSecond, noise_.processYVariancePerSecond,
      noise_.processHeadingVariancePerSecond, noise_.processSpeedVariancePerSecond,
      noise_.processYawRateVariancePerSecond, noise_.processGyroBiasVariancePerSecond;
  const CovarianceMatrix processNoise =
      (processVariancePerSecond * timeStepSeconds).asDiagonal();  // Q

  state_ = predictedState(state_, timeStepSeconds);
  covariance_ = jacobian * covariance_ * jacobian.transpose() + processNoise;
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateEncoderSpeed(
    double measuredSpeedMetersPerSecond) {
  requireFinite(measuredSpeedMetersPerSecond, "measuredSpeedMetersPerSecond");
  MeasurementRow measurementRow = MeasurementRow::Zero();
  measurementRow(kSpeedIndex) = 1.0;
  const double predictedMeasurement = state_(kSpeedIndex);
  return applyScalarUpdate(measurementRow, measuredSpeedMetersPerSecond - predictedMeasurement,
                           noise_.encoderSpeedVariance);
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateGyroYawRate(
    double measuredYawRateRadiansPerSecond) {
  requireFinite(measuredYawRateRadiansPerSecond, "measuredYawRateRadiansPerSecond");
  MeasurementRow measurementRow = MeasurementRow::Zero();
  measurementRow(kYawRateIndex) = 1.0;
  measurementRow(kGyroBiasIndex) = 1.0;
  const double predictedMeasurement = state_(kYawRateIndex) + state_(kGyroBiasIndex);
  return applyScalarUpdate(measurementRow,
                           measuredYawRateRadiansPerSecond - predictedMeasurement,
                           noise_.gyroYawRateVariance);
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateSteeringModel(
    double steeringAngleRadians) {
  requireFinite(steeringAngleRadians, "steeringAngleRadians");
  // tan() blows up at +/-pi/2; a steering angle there is physically meaningless.
  if (!(std::abs(steeringAngleRadians) < kPi / 2.0)) {
    throw std::invalid_argument("steeringAngleRadians must be in (-pi/2, pi/2)");
  }

  // [k] Path curvature commanded by the steering: yaw_rate = k * speed.
  const double curvaturePerMeter = std::tan(steeringAngleRadians) / wheelbaseMeters_;
  const double speed = state_(kSpeedIndex);

  MeasurementRow measurementRow = MeasurementRow::Zero();
  measurementRow(kSpeedIndex) = -curvaturePerMeter;
  measurementRow(kYawRateIndex) = 1.0;

  // z = 0 (pseudo-measurement), h = yaw_rate - k*speed.
  const double predictedMeasurement = state_(kYawRateIndex) - curvaturePerMeter * speed;
  const double residual = 0.0 - predictedMeasurement;

  // Tight when stopped or straight, loose while turning (tire scrub). Uses
  // the current speed *estimate*, since that is all the filter has.
  const double steeringSigma =
      noise_.steeringSigmaMinRadiansPerSecond +
      noise_.steeringSigmaRelative * std::abs(curvaturePerMeter * speed);
  return applyScalarUpdate(measurementRow, residual, steeringSigma * steeringSigma);
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::applyScalarUpdate(
    const MeasurementRow& measurementRow, double residual, double measurementVariance) {
  // S: scalar, so the Kalman gain is a plain division - no matrix inverse.
  const double residualVariance =
      (measurementRow * covariance_ * measurementRow.transpose())(0, 0) + measurementVariance;
  const StateVector kalmanGain = covariance_ * measurementRow.transpose() / residualVariance;

  state_ += kalmanGain * residual;
  state_(kHeadingIndex) = wrapAngleToPlusMinusPi(state_(kHeadingIndex));

  // Joseph form: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T. Algebraically the
  // same as (I - K*H) * P, but it stays symmetric positive semi-definite
  // despite rounding.
  const CovarianceMatrix identityMinusGainTimesRow =
      CovarianceMatrix::Identity() - kalmanGain * measurementRow;
  covariance_ = identityMinusGainTimesRow * covariance_ * identityMinusGainTimesRow.transpose() +
                kalmanGain * measurementVariance * kalmanGain.transpose();

  return UpdateResult{residual, residualVariance};
}

}  // namespace rover
