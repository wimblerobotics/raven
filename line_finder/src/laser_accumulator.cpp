// ###ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator

#include "line_finder/laser_accumulator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace laser_accumulator {

LaserAccumulator::LaserAccumulator() : Node("laser_accumulator_node") {
  loadParameters();

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
  qos.avoid_ros_namespace_conventions(false);

  scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&LaserAccumulator::laserScanCallback, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LaserAccumulator::~LaserAccumulator() {}

void LaserAccumulator::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) const {
  float angle_min = scan_msg->angle_min;
  std::cout << "angle_min: " << angle_min << std::endl;
  try {
    tf2::TimePoint time_point = tf2_ros::fromMsg(scan_msg->header.stamp);
    bool canTransform =
        tf_buffer_->canTransform("odom", scan_msg->header.frame_id.c_str(),
                                 time_point, tf2::durationFromSec(1.0));
    if (!canTransform) {
      RCLCPP_WARN(get_logger(),
                  "Could not look up transform from %s to %s at time [%d.%d]",
                  scan_msg->header.frame_id.c_str(), "odom",
                  scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Transform available");
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("odom", scan_msg->header.frame_id,
                                      tf2::TimePointZero);
      double x = transform_stamped.transform.translation.x;
      double y = transform_stamped.transform.translation.y;
    }
  } catch (tf2::TransformException& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
  }
}

void LaserAccumulator::loadParameters() { scan_topic_ = "/scan"; }

}  // namespace laser_accumulator
