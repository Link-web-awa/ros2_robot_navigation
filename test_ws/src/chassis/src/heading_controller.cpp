// Copyright 2026
// PID heading-hold controller for a mobile chassis.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class HeadingController : public rclcpp::Node
{
public:
  HeadingController()
  : Node("heading_controller")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/car/imu_raw");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    gyro_full_scale_dps_ = declare_parameter<double>("gyro_full_scale_dps", 500.0);
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);
    kp_ = declare_parameter<double>("kp", 1.5);
    ki_ = declare_parameter<double>("ki", 0.0);
    kd_ = declare_parameter<double>("kd", 0.1);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.0);
    integral_limit_ = declare_parameter<double>("integral_limit", 0.5);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.01);

    if (
      gyro_full_scale_dps_ <= 0.0 || calibration_samples_ <= 0 ||
      max_angular_velocity_ <= 0.0 || integral_limit_ < 0.0 || yaw_tolerance_ < 0.0)
    {
      throw std::invalid_argument(
              "gyro_full_scale_dps, calibration_samples and max_angular_velocity must be "
              "positive; integral_limit and yaw_tolerance must be non-negative");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    imu_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HeadingController::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Keep the chassis still: calibrating gyro bias from %d samples on %s",
      calibration_samples_, imu_topic_.c_str());
  }

  ~HeadingController() override
  {
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

private:
  void imu_callback(const std_msgs::msg::Int16MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 6U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring imu_raw message: expected 6 values, got %zu", msg->data.size());
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    const double gyro_z_raw = static_cast<double>(msg->data[5]);

    if (!calibrated_) {
      gyro_bias_sum_ += gyro_z_raw;
      ++calibration_count_;
      if (calibration_count_ < calibration_samples_) {
        return;
      }
      gyro_bias_raw_ = gyro_bias_sum_ / static_cast<double>(calibration_count_);
      previous_time_ = now;
      calibrated_ = true;
      RCLCPP_INFO(
        get_logger(), "Gyro calibrated: z bias = %.3f raw; relative heading locked at zero",
        gyro_bias_raw_);
      publish_command(0.0);
      return;
    }

    const double dt = (now - previous_time_).seconds();
    previous_time_ = now;
    if (dt <= 0.0 || dt >= 1.0) {
      return;
    }

    const double gyro_z_dps =
      (gyro_z_raw - gyro_bias_raw_) * gyro_full_scale_dps_ / 32768.0;
    const double gyro_z_rad_s = gyro_z_dps * kPi / 180.0;
    yaw_ = normalize_angle(yaw_ + gyro_z_rad_s * dt);
    const double error = normalize_angle(-yaw_);

    integral_ = std::clamp(integral_ + error * dt, -integral_limit_, integral_limit_);
    const double derivative = dt > 1.0e-4 ? (error - previous_error_) / dt : 0.0;
    previous_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    if (std::abs(error) <= yaw_tolerance_) {
      output = 0.0;
    }
    publish_command(std::clamp(output, -max_angular_velocity_, max_angular_velocity_));
  }

  void publish_command(double angular_z)
  {
    geometry_msgs::msg::Twist command;
    command.angular.z = angular_z;
    cmd_vel_pub_->publish(command);
  }

  std::string imu_topic_;
  std::string cmd_vel_topic_;
  double gyro_full_scale_dps_{500.0};
  int calibration_samples_{200};
  double kp_{1.5};
  double ki_{0.0};
  double kd_{0.1};
  double max_angular_velocity_{1.0};
  double integral_limit_{0.5};
  double yaw_tolerance_{0.01};
  double gyro_bias_sum_{0.0};
  double gyro_bias_raw_{0.0};
  double yaw_{0.0};
  double integral_{0.0};
  double previous_error_{0.0};
  int calibration_count_{0};
  bool calibrated_{false};
  rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HeadingController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("heading_controller"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
