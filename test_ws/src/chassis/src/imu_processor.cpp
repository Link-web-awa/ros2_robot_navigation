// Copyright 2026
// Converts the chassis six-axis raw IMU data into a standard ROS 2 IMU message.
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStandardGravity = 9.80665;
constexpr double kInt16Scale = 32768.0;
}  // namespace

class ImuProcessor : public rclcpp::Node
{
public:
  ImuProcessor()
  : Node("imu_processor")
  {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/car/imu_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/car/imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    accel_full_scale_g_ = declare_parameter<double>("accel_full_scale_g", 2.0);
    gyro_full_scale_dps_ = declare_parameter<double>("gyro_full_scale_dps", 500.0);
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);

    if (
      accel_full_scale_g_ <= 0.0 || gyro_full_scale_dps_ <= 0.0 ||
      calibration_samples_ <= 0)
    {
      throw std::invalid_argument(
              "accel_full_scale_g, gyro_full_scale_dps and calibration_samples must be positive");
    }

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    raw_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      raw_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuProcessor::raw_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Keep the chassis still: calibrating gyro from %d samples on %s",
      calibration_samples_, raw_topic_.c_str());
  }

private:
  void raw_callback(const std_msgs::msg::Int16MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 6U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring raw IMU message: expected 6 values, got %zu", msg->data.size());
      return;
    }

    if (!calibrated_) {
      for (std::size_t axis = 0; axis < gyro_bias_sum_.size(); ++axis) {
        gyro_bias_sum_[axis] += static_cast<double>(msg->data[axis + 3U]);
      }
      ++calibration_count_;
      if (calibration_count_ < calibration_samples_) {
        return;
      }

      for (std::size_t axis = 0; axis < gyro_bias_raw_.size(); ++axis) {
        gyro_bias_raw_[axis] =
          gyro_bias_sum_[axis] / static_cast<double>(calibration_count_);
      }
      calibrated_ = true;
      RCLCPP_INFO(
        get_logger(), "Gyro calibrated: raw bias x=%.3f, y=%.3f, z=%.3f; publishing %s",
        gyro_bias_raw_[0], gyro_bias_raw_[1], gyro_bias_raw_[2], imu_topic_.c_str());
      return;
    }

    const double accel_scale = accel_full_scale_g_ * kStandardGravity / kInt16Scale;
    const double gyro_scale = gyro_full_scale_dps_ * kPi / (180.0 * kInt16Scale);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = get_clock()->now();
    imu_msg.header.frame_id = frame_id_;
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.linear_acceleration.x = static_cast<double>(msg->data[0]) * accel_scale;
    imu_msg.linear_acceleration.y = static_cast<double>(msg->data[1]) * accel_scale;
    imu_msg.linear_acceleration.z = static_cast<double>(msg->data[2]) * accel_scale;
    imu_msg.angular_velocity.x =
      (static_cast<double>(msg->data[3]) - gyro_bias_raw_[0]) * gyro_scale;
    imu_msg.angular_velocity.y =
      (static_cast<double>(msg->data[4]) - gyro_bias_raw_[1]) * gyro_scale;
    imu_msg.angular_velocity.z =
      (static_cast<double>(msg->data[5]) - gyro_bias_raw_[2]) * gyro_scale;
    imu_pub_->publish(imu_msg);
  }

  std::string raw_topic_;
  std::string imu_topic_;
  std::string frame_id_;
  double accel_full_scale_g_{2.0};
  double gyro_full_scale_dps_{500.0};
  int calibration_samples_{200};
  int calibration_count_{0};
  bool calibrated_{false};
  std::array<double, 3> gyro_bias_sum_{};
  std::array<double, 3> gyro_bias_raw_{};

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuProcessor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_processor"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
