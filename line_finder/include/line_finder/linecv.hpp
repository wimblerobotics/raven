#pragma once

#include <tf2_ros/buffer.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"

namespace linecv {
class LineCv : public rclcpp::Node {
 public:
  LineCv();
  ~LineCv();

 private:
  static const uint32_t kMAX_RANGE = 10;
  static const uint8_t kPIXELS_PER_METER = 40;
  static const uint32_t kMAP_PIXELS =
      (uint32_t)(kPIXELS_PER_METER * kMAX_RANGE * 2);
  static uint8_t points_[kMAP_PIXELS][kMAP_PIXELS];

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  uint8_t initial_persistence_count_;
  uint8_t max_persistence_count_;

  void laserScanCallback(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  void manage_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

}  // namespace linecv