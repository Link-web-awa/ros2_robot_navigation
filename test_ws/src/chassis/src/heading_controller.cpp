// Copyright 2026
// PID heading-hold controller for a mobile chassis.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
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
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/car/imu");
    angular_z_topic_ =
      declare_parameter<std::string>("angular_z_topic", "/heading_controller/angular_z");
    kp_ = declare_parameter<double>("kp", 1.5);
    ki_ = declare_parameter<double>("ki", 0.0);
    kd_ = declare_parameter<double>("kd", 0.1);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.0);
    integral_limit_ = declare_parameter<double>("integral_limit", 0.5);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.01);

    if (
      max_angular_velocity_ <= 0.0 || integral_limit_ < 0.0 || yaw_tolerance_ < 0.0)
    {
      throw std::invalid_argument(
              "max_angular_velocity must be positive; integral_limit and yaw_tolerance "
              "must be non-negative");
    }

    angular_z_pub_ = create_publisher<std_msgs::msg::Float64>(angular_z_topic_, 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HeadingController::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Holding relative heading using processed IMU data from %s",
      imu_topic_.c_str());
  }

  ~HeadingController() override
  {
    if (angular_z_pub_) {
      angular_z_pub_->publish(std_msgs::msg::Float64{});
    }
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!std::isfinite(msg->angular_velocity.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring IMU message with non-finite angular z");
      return;
    }

    const rclcpp::Time now(msg->header.stamp, get_clock()->get_clock_type());
    if (!has_previous_time_) {
      previous_time_ = now;
      has_previous_time_ = true;
      publish_command(0.0);
      return;
    }

    const double dt = (now - previous_time_).seconds();
    previous_time_ = now;
    if (dt <= 0.0 || dt >= 1.0) {
      return;
    }

    yaw_ = normalize_angle(yaw_ + msg->angular_velocity.z * dt);
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
    std_msgs::msg::Float64 command;
    command.data = angular_z;
    angular_z_pub_->publish(command);
  }

  std::string imu_topic_;
  std::string angular_z_topic_;
  double kp_{1.5};
  double ki_{0.0};
  double kd_{0.1};
  double max_angular_velocity_{1.0};
  double integral_limit_{0.5};
  double yaw_tolerance_{0.01};
  double yaw_{0.0};
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_time_{false};
  rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_z_pub_;
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
