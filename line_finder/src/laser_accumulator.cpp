// ###ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator

#include "line_finder/laser_accumulator.hpp"

#include <matplotlibcpp.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
namespace plt = matplotlibcpp;

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

void LaserAccumulator::manage_scan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  static bool initialized = false;
  if (!initialized) {
    for (size_t i = 0; i < kMAP_PIXELS; i++) {
      for (size_t j = 0; j < kMAP_PIXELS; j++) {
        points_[i][j] = 0;
      }
    }

    initialized = true;
  }

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
      for (size_t i = 0; i < kMAP_PIXELS; i++) {
        for (size_t j = 0; j < kMAP_PIXELS; j++) {
          if (points_[i][j] > 0) {
            points_[i][j] -= 1;
          }
        }
      }

      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("odom", scan_msg->header.frame_id,
                                      tf2::TimePointZero);
      float angle = scan_msg->angle_min;
      for (float range : scan_msg->ranges) {
        if ((range < (float)kMAX_RANGE) && (range > -(float)kMAX_RANGE)) {
          geometry_msgs::msg::PointStamped in, out;
          in.header = scan_msg->header;
          in.point.x = range * cos(angle);
          in.point.y = range * sin(angle);
          in.point.z = 0;
          tf_buffer_->transform(in, out, "odom");
          int x_coord = (out.point.x * kPIXELS_PER_METER) + (kMAP_PIXELS / 2);
          int y_coord = (out.point.y * kPIXELS_PER_METER) + (kMAP_PIXELS / 2);
          if ((x_coord >= 0) && (y_coord >= 0) &&
              ((uint32_t)x_coord < kMAP_PIXELS) &&
              ((uint32_t)y_coord < kMAP_PIXELS) &&
              (points_[x_coord][y_coord] < kMAX_PERSISTENCE)) {
            points_[x_coord][y_coord] += kPERSISTENCE;
          }
        }
        angle += scan_msg->angle_increment;
      }
    }
  } catch (tf2::TransformException& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
  }
}

void LaserAccumulator::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  manage_scan(scan_msg);
  std::vector<float> xs(scan_msg->ranges.size());
  std::vector<float> ys(scan_msg->ranges.size());
  for (size_t x = 0; x < kMAP_PIXELS; x++) {
    for (size_t y = 0; y < kMAP_PIXELS; y++) {
      if (points_[x][y] > 0) {
        xs.push_back((x / (kPIXELS_PER_METER * 1.0) - kMAX_RANGE));
        ys.push_back((y / (kPIXELS_PER_METER * 1.0) - kMAX_RANGE));
      }
    }
  }

  plt::clf();
  plt::xlim(-(kMAX_RANGE * 1.0), kMAX_RANGE * 1.0);
  plt::ylim(-(kMAX_RANGE * 1.0), kMAX_RANGE * 1.0);
  plt::scatter(xs, ys);
  plt::pause(0.001);
}

void LaserAccumulator::loadParameters() { scan_topic_ = "/scan"; }

uint8_t LaserAccumulator::points_[kMAP_PIXELS][kMAP_PIXELS];

}  // namespace laser_accumulator
