#pragma once

#include <tf2_ros/buffer.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"

namespace laser_accumulator {
class LaserAccumulator : public rclcpp::Node {
 public:
  LaserAccumulator();
  ~LaserAccumulator();

 private:
  // ROS
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  std::string scan_topic_;

  void laserScanCallback(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr);

  void loadParameters();

  void manage_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  static uint8_t points_ [600][600];
};
}  // namespace laser_accumulator