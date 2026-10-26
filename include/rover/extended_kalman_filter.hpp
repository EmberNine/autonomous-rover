#pragma once

#include <Eigen/Core>

namespace rover {

// Tuning knobs for ExtendedKalmanFilter. Symbols in [brackets] are the names
// used in docs/kalman-filter.md.
//
// !!! Every default below is a PLACEHOLDER, not a measured value. Replace them
// !!! with numbers from the ~1 h static IMU log and the encoder tests before
// !!! trusting the filter's reported uncertainty.
struct ExtendedKalmanFilterNoise {
  // Process noise: how fast each state's variance grows per second while
  // predicting [q_x ... q_gyro_bias]. Q = diag(these) * dt. Zero is allowed
  // (a "perfect model" for that state); negative is not.
  double processXVariancePerSecond{1e-4};          // [q_x]         m^2/s          PLACEHOLDER
  double processYVariancePerSecond{1e-4};          // [q_y]         m^2/s          PLACEHOLDER
  double processHeadingVariancePerSecond{1e-4};    // [q_heading]   rad^2/s        PLACEHOLDER
  double processSpeedVariancePerSecond{1.0};       // [q_speed]     (m/s)^2/s      PLACEHOLDER
  double processYawRateVariancePerSecond{1.0};     // [q_yaw_rate]  (rad/s)^2/s    PLACEHOLDER
  double processGyroBiasVariancePerSecond{1e-8};   // [q_gyro_bias] (rad/s)^2/s    PLACEHOLDER

  // Measurement noise. Must be > 0: R is what keeps S = H*P*H^T + R away from
  // zero when P is (nearly) zero in the measured direction, and K = P*H^T / S.
  double encoderSpeedVariance{4e-4};  // [R_encoder] (m/s)^2    PLACEHOLDER (sigma 0.02 m/s)
  double gyroYawRateVariance{1e-4};   // [R_gyro]    (rad/s)^2  PLACEHOLDER (sigma 0.01 rad/s)

  // Steering-model pseudo-measurement:
  //   sigma_steer = sigma_steer_min + sigma_steer_relative * |k * speed|
  // Note these are standard deviations, not variances; R_steer = sigma_steer^2.
  // sigma_steer_min must be > 0 (it is all of R_steer when stopped or
  // straight); sigma_steer_relative may be 0 (no extra looseness in turns).
  double steeringSigmaMinRadiansPerSecond{0.01};  // [sigma_steer_min] rad/s       PLACEHOLDER
  double steeringSigmaRelative{0.2};              // [sigma_steer_relative] unitless PLACEHOLDER
};

// Extended Kalman Filter fusing wheel-encoder speed, gyro yaw rate and the
// bicycle model (as a soft "pseudo-measurement"). The math is specified in
// docs/kalman-filter.md and this class follows it term for term.
//
// State (6 values, see the k*Index constants):
//   x, y       [m]      rear-axle center, world frame
//   heading    [rad]    yaw, always kept in (-pi, pi]
//   speed      [m/s]    forward speed of the rear-axle center
//   yaw_rate   [rad/s]  true rotation rate
//   gyro_bias  [rad/s]  gyro z-axis offset
//
// Typical loop: predict(dt) once per cycle, then call whichever update*()
// functions have fresh data this cycle, in any order.
//
// This class does not depend on ROS.
class ExtendedKalmanFilter {
 public:
  // Position of each quantity inside the state vector / covariance matrix.
  static constexpr int kXIndex = 0;
  static constexpr int kYIndex = 1;
  static constexpr int kHeadingIndex = 2;
  static constexpr int kSpeedIndex = 3;
  static constexpr int kYawRateIndex = 4;
  static constexpr int kGyroBiasIndex = 5;
  static constexpr int kStateSize = 6;

  // Fixed-size Eigen types: the sizes are known at compile time, so there is
  // no heap allocation and the compiler can unroll the small loops.
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;
  using CovarianceMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

  // What a scalar update saw, kept for later consistency checks (NIS).
  struct UpdateResult {
    double residual{0.0};          // z - h(state), before the update
    double residualVariance{0.0};  // S = H*P*H^T + R, before the update
  };

  // The initial state and covariance are required: there is no sensible
  // default uncertainty, so the caller must state it. The heading is wrapped
  // into (-pi, pi].
  //
  // Throws std::invalid_argument if:
  //   - wheelbaseMeters is not positive and finite,
  //   - any process-noise value or steeringSigmaRelative is negative or
  //     non-finite (zero is allowed),
  //   - encoderSpeedVariance, gyroYawRateVariance or
  //     steeringSigmaMinRadiansPerSecond is not positive and finite,
  //   - initialState/initialCovariance fail the same checks as reset().
  ExtendedKalmanFilter(double wheelbaseMeters, const ExtendedKalmanFilterNoise& noise,
                       const StateVector& initialState, const CovarianceMatrix& initialCovariance);

  // Re-initializes the state and covariance (e.g. after the rover is picked up
  // and put down somewhere known). The heading is wrapped into (-pi, pi].
  // Throws std::invalid_argument (leaving the filter unchanged) if anything is
  // non-finite, or the covariance is not symmetric positive semi-definite.
  void reset(const StateVector& initialState, const CovarianceMatrix& initialCovariance);

  const StateVector& state() const { return state_; }
  const CovarianceMatrix& covariance() const { return covariance_; }

  // Advances the state by timeStepSeconds with the midpoint-heading model and
  // grows the covariance: P' = F*P*F^T + Q*dt.
  //   - timeStepSeconds == 0 is a no-op.
  //   - Throws std::invalid_argument if timeStepSeconds is negative, NaN or
  //     infinite; the filter is left unchanged in that case.
  void predict(double timeStepSeconds);

  // Scalar measurement updates (Joseph form). Each throws
  // std::invalid_argument, leaving the filter unchanged, if its input is
  // NaN/infinite.
  //   updateEncoderSpeed:  z = measured speed [m/s],         h = speed
  //   updateGyroYawRate:   z = measured z-rotation [rad/s],  h = yaw_rate + gyro_bias
  //   updateSteeringModel: z = 0,  h = yaw_rate - k*speed,  k = tan(delta)/wheelbase,
  //                        where delta = steeringAngleRadians is the commanded
  //                        (effective) steering angle; must be in (-pi/2, pi/2).
  UpdateResult updateEncoderSpeed(double measuredSpeedMetersPerSecond);
  UpdateResult updateGyroYawRate(double measuredYawRateRadiansPerSecond);
  UpdateResult updateSteeringModel(double steeringAngleRadians);

  // The process model f(state, dt) and its Jacobian F = df/dstate, as pure
  // functions. predict() uses them; they are public so tests can check F
  // against finite differences of f. They do no argument validation.
  static StateVector predictedState(const StateVector& currentState, double timeStepSeconds);
  static CovarianceMatrix transitionJacobian(const StateVector& currentState,
                                             double timeStepSeconds);

 private:
  using MeasurementRow = Eigen::Matrix<double, 1, kStateSize>;  // H

  // Shared by the constructor and reset(): throws std::invalid_argument
  // unless both are finite and the covariance is symmetric PSD.
  static void validateStateAndCovariance(const StateVector& candidateState,
                                         const CovarianceMatrix& candidateCovariance);

  UpdateResult applyScalarUpdate(const MeasurementRow& measurementRow, double residual,
                                 double measurementVariance);

  double wheelbaseMeters_;
  ExtendedKalmanFilterNoise noise_;
  StateVector state_;
  CovarianceMatrix covariance_;
};

}  // namespace rover
