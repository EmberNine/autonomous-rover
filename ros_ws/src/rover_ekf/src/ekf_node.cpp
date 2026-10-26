// The rover's EKF as a ROS 2 node (group 6). Runs on the Pi, in its own
// container next to the bridge, so the 100 Hz inputs never cross Wi-Fi; RViz
// on the laptop only needs /odom and TF. All the logic is the ROS-free
// rover::OdometryFusion (include/rover/odometry_fusion.hpp, tested with the
// core); this file only converts messages and publishes.
//
//   in   /rover/imu/raw       angular_velocity.z: the yaw rate (IMU axes X forward,
//                             Y left, Z up); each message is one filter cycle
//        /rover/joint_states  rear_left_wheel_joint velocity [rad/s] -> m/s
//                             (meters_per_encoder_tick per 2*pi/4096 rad); no
//                             velocity = the estimate is not valid
//        /rover/steering      joint 'steering' [rad]: the angle the servo has
//   out  /odom                nav_msgs/Odometry, odom -> base_link (the rear-axle
//                             centre), pose and twist covariance from the filter
//        TF odom -> base_link
//        every publish_every-th cycle (2: 50 Hz)
//   srv  /rover/ekf/reset     std_srvs/Trigger: the pose back to (0, 0, 0)
//
// Nothing is published until the filter has started (2 s at rest seeds the
// gyro bias): keep the truck still after starting the node.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rover/odometry_fusion.hpp"
#include "rover/vehicle_config.hpp"

namespace {

using rover::ExtendedKalmanFilter;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerCount = 2.0 * kPi / 4096.0;  // AS5600, as the bridge's conversions.py
constexpr double kImuStaleSeconds = 0.2;
constexpr double kFlatVariance = 1e-9;  // z, roll, pitch: the floor is flat (2D filter)

// The steering model's noise, from the 2026-10-22 full-lock turning circles
// (config/vehicle.cfg): six runs of 0.212, 0.197, 0.202, 0.216, 0.185, 0.221 rad,
// sample standard deviation sigma_delta = 0.0134 rad. With k = tan(delta) / L:
//   relative: sigma_k / k = sigma_delta / (sin(delta) cos(delta)) at full lock
//             (0.204 rad) = 0.0134 / 0.1984 = 0.0675
//   minimum:  near straight sigma_k = sigma_delta / L, so the yaw-rate error is
//             |v| * 0.0134 / 0.133; at the teleop's top speed (0.15 m/s) 0.0151 rad/s
// Both assume the full-lock spread holds at every angle (only full lock was measured).
constexpr double kSteeringSigmaMin = 0.0151;
constexpr double kSteeringSigmaRelative = 0.0675;

class EkfNode : public rclcpp::Node {
 public:
  EkfNode() : rclcpp::Node("rover_ekf") {
    const std::string vehicleConfigPath =
        declare_parameter<std::string>("vehicle_cfg", "/repo/config/vehicle.cfg");
    wheelJoint_ = declare_parameter<std::string>("wheel_joint", "rear_left_wheel_joint");
    publishEvery_ = declare_parameter<int>("publish_every", 2);
    odomFrame_ = declare_parameter<std::string>("odom_frame", "odom");
    baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");

    const rover::VehicleConfig vehicle = rover::VehicleConfig::loadFromFile(vehicleConfigPath);
    if (!(vehicle.metersPerEncoderTick > 0.0)) {
      throw std::runtime_error("meters_per_encoder_tick missing in " + vehicleConfigPath);
    }
    metersPerRadian_ = vehicle.metersPerEncoderTick / kRadiansPerCount;

    rover::OdometryFusionConfig config;
    config.wheelbaseMeters = vehicle.wheelbaseMeters;
    config.rearTrackWidthMeters = vehicle.rearTrackWidthMeters;
    // The cfg's value (the truck: locked) unless overridden: the Gazebo model's AckermannSteering
    // drives its wheels like an ideal differential, so the sim runs this with false.
    config.rearAxleLocked = declare_parameter<bool>("rear_axle_locked", vehicle.rearAxleLocked);
    // R_gyro and q_gyro_bias: docs/kalman-filter.md (the 1 h static log gives
    // ~9e-8 at 100 Hz and ~1e-11; both raised for vibration and warm-up, 2026-10-22).
    config.noise.gyroYawRateVariance = declare_parameter<double>("gyro_yaw_rate_variance", 1e-6);
    config.noise.processGyroBiasVariancePerSecond =
        declare_parameter<double>("process_gyro_bias_variance_per_second", 1e-9);
    config.noise.steeringSigmaMinRadiansPerSecond =
        declare_parameter<double>("steering_sigma_min", kSteeringSigmaMin);
    config.noise.steeringSigmaRelative =
        declare_parameter<double>("steering_sigma_relative", kSteeringSigmaRelative);
    config.noise.encoderSpeedVariance =
        declare_parameter<double>("encoder_speed_variance", config.noise.encoderSpeedVariance);
    fusion_.emplace(config);

    const auto sensorQos = rclcpp::SensorDataQoS();
    imuSubscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "/rover/imu/raw", sensorQos, [this](const sensor_msgs::msg::Imu& message) { onImu(message); });
    jointSubscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/rover/joint_states", sensorQos,
        [this](const sensor_msgs::msg::JointState& message) { onJoints(message); });
    steeringSubscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/rover/steering", sensorQos,
        [this](const sensor_msgs::msg::JointState& message) { onSteering(message); });
    odomPublisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    resetService_ = create_service<std_srvs::srv::Trigger>(
        "/rover/ekf/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (!fusion_->started()) {
            response->success = false;
            response->message = "not started yet (keep the truck still for 2 s)";
            return;
          }
          fusion_->resetPose();
          response->success = true;
          response->message = "pose reset to (0, 0, 0)";
          RCLCPP_INFO(get_logger(), "pose reset to (0, 0, 0)");
        });
    // A steady clock, as the bridge's timers: a wall-clock step must not stall it.
    watchTimer_ = rclcpp::create_timer(this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME),
                                       std::chrono::milliseconds(100), [this] { watch(); });
    RCLCPP_INFO(get_logger(),
                "wheelbase %.3f m, rear track %.3f m, rear axle %s, %.4e m per count; waiting for 2 s "
                "at rest (keep the truck still)",
                vehicle.wheelbaseMeters, vehicle.rearTrackWidthMeters,
                config.rearAxleLocked ? "locked" : "open", vehicle.metersPerEncoderTick);
  }

 private:
  void onImu(const sensor_msgs::msg::Imu& message) {
    lastImuAt_ = steadyNow();
    const bool advanced =
        fusion_->onGyro(rclcpp::Time(message.header.stamp).seconds(), message.angular_velocity.z);
    if (fusion_->started() && !startLogged_) {  // the start itself happens in a call that returns false
      startLogged_ = true;
      RCLCPP_INFO(get_logger(), "started: gyro bias %.5f rad/s",
                  fusion_->filter().state()(ExtendedKalmanFilter::kGyroBiasIndex));
    }
    if (!advanced) {
      return;
    }
    if (++cycles_ % publishEvery_ == 0) {
      publish(message.header.stamp);
    }
  }

  void onJoints(const sensor_msgs::msg::JointState& message) {
    for (size_t index = 0; index < message.name.size(); ++index) {
      if (message.name[index] != wheelJoint_) {
        continue;
      }
      std::optional<double> speed;
      if (index < message.velocity.size()) {
        speed = message.velocity[index] * metersPerRadian_;
      }
      fusion_->onEncoder(rclcpp::Time(message.header.stamp).seconds(), speed);
      return;
    }
  }

  void onSteering(const sensor_msgs::msg::JointState& message) {
    if (!message.name.empty() && message.name[0] == "steering" && !message.position.empty()) {
      fusion_->onSteering(rclcpp::Time(message.header.stamp).seconds(), message.position[0]);
    }
  }

  void publish(const builtin_interfaces::msg::Time& stamp) {
    const auto& state = fusion_->filter().state();
    const auto& covariance = fusion_->filter().covariance();
    const double heading = state(ExtendedKalmanFilter::kHeadingIndex);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odomFrame_;
    odometry.child_frame_id = baseFrame_;
    odometry.pose.pose.position.x = state(ExtendedKalmanFilter::kXIndex);
    odometry.pose.pose.position.y = state(ExtendedKalmanFilter::kYIndex);
    odometry.pose.pose.orientation.z = std::sin(heading / 2.0);
    odometry.pose.pose.orientation.w = std::cos(heading / 2.0);
    // Row-major 6x6 over (x, y, z, roll, pitch, yaw); the filter's x, y, heading.
    const int filterIndex[3] = {ExtendedKalmanFilter::kXIndex, ExtendedKalmanFilter::kYIndex,
                                ExtendedKalmanFilter::kHeadingIndex};
    const int poseIndex[3] = {0, 1, 5};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        odometry.pose.covariance[poseIndex[row] * 6 + poseIndex[column]] =
            covariance(filterIndex[row], filterIndex[column]);
      }
    }
    for (int flat : {2, 3, 4}) {
      odometry.pose.covariance[flat * 6 + flat] = kFlatVariance;
    }
    odometry.twist.twist.linear.x = state(ExtendedKalmanFilter::kSpeedIndex);
    odometry.twist.twist.angular.z = state(ExtendedKalmanFilter::kYawRateIndex);
    odometry.twist.covariance[0] =
        covariance(ExtendedKalmanFilter::kSpeedIndex, ExtendedKalmanFilter::kSpeedIndex);
    odometry.twist.covariance[35] =
        covariance(ExtendedKalmanFilter::kYawRateIndex, ExtendedKalmanFilter::kYawRateIndex);
    for (int flat : {1, 2, 3, 4}) {
      odometry.twist.covariance[flat * 6 + flat] = kFlatVariance;
    }
    odomPublisher_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = baseFrame_;
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tfBroadcaster_->sendTransform(transform);
  }

  // 10 Hz: say when the IMU (the cycle) goes quiet and comes back; every 10 s the counts.
  void watch() {
    const double now = steadyNow();
    const bool quiet = !lastImuAt_ || now - *lastImuAt_ > kImuStaleSeconds;
    if (quiet && !imuQuiet_) {
      RCLCPP_WARN(get_logger(), "no /rover/imu/raw for %.0f ms: nothing published", kImuStaleSeconds * 1000.0);
    } else if (!quiet && imuQuiet_) {
      RCLCPP_INFO(get_logger(), "/rover/imu/raw back");
    }
    imuQuiet_ = quiet;
    if (now - lastReportAt_ >= 10.0) {
      lastReportAt_ = now;
      const auto& counts = fusion_->counts();
      if (fusion_->started()) {
        const auto& state = fusion_->filter().state();
        RCLCPP_INFO(get_logger(),
                    "x %.3f y %.3f heading %.3f rad, speed %.3f m/s, bias %.5f rad/s; cycles %ld, "
                    "encoder %ld, steering %ld, gaps %ld, dropped %ld",
                    state(ExtendedKalmanFilter::kXIndex), state(ExtendedKalmanFilter::kYIndex),
                    state(ExtendedKalmanFilter::kHeadingIndex), state(ExtendedKalmanFilter::kSpeedIndex),
                    state(ExtendedKalmanFilter::kGyroBiasIndex), counts.cycles, counts.encoderUpdates,
                    counts.steeringUpdates, counts.gaps, counts.dropped);
      } else {
        RCLCPP_INFO(get_logger(), "not started: waiting for 2 s at rest with a valid encoder");
      }
    }
  }

  static double steadyNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::optional<rover::OdometryFusion> fusion_;
  std::string wheelJoint_, odomFrame_, baseFrame_;
  long publishEvery_{2};
  double metersPerRadian_{0.0};
  long cycles_{0};
  std::optional<double> lastImuAt_;
  bool imuQuiet_{false};
  bool startLogged_{false};
  double lastReportAt_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steeringSubscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
  rclcpp::TimerBase::SharedPtr watchTimer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
