// ###ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator

#include "line_finder/laser_accumulator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
#include <matplotlibcpp.h>

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

  // ###
  //  int n = 5000; // number of data points
  //  std::vector<double> x(n),y(n);
  //  for(int i=0; i<n; ++i) {
  //      double t = 2*M_PI*i/n;
  //      x.at(i) = 16*sin(t)*sin(t)*sin(t);
  //      y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
  //  }

  // // plot() takes an arbitrary number of (x,y,format)-triples.
  // // x must be iterable (that is, anything providing begin(x) and end(x)),
  // // y must either be callable (providing operator() const) or iterable.
  // plt::plot(x, y, "r-", x, [](double d) { return 12.5+abs(sin(d)); }, "k-");

  // // show plots
  // plt::show();  //###
}

LaserAccumulator::~LaserAccumulator() {}

void LaserAccumulator::manage_scan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  static bool initialized = false;
  if (!initialized) {
    for (size_t i = 0; i < 600; i++) {
      for (size_t j = 0; j < 600; j++) {
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
      for (size_t i = 0; i < 600; i++) {
        for (size_t j = 0; j < 600; j++) {
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
        if ((range < 50.0) && (range > -50.0)) {
          float x =
              (range * cos(angle)) - transform_stamped.transform.translation.x;
          float y =
              (range * sin(angle)) - transform_stamped.transform.translation.y;
          int x_coord = 300 + (x * 100);
          int y_coord = 300 + (y * 100);
          if ((x_coord >= 0) && (y_coord >= 0) && (x_coord < 600) &&
              (y_coord < 600) && (points_[x_coord][y_coord] < 127)) {
            points_[x_coord][y_coord] += 1;
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
  for (size_t x = 0; x < 600; x++) {
    for (size_t y = 0; y < 600; y++) {
      if (points_[x][y] > 0) {
        xs.push_back((x - 300.0) / 300.0);
        ys.push_back((y - 300.0) / 300.0);
      }
    }
  }

  plt::scatter(xs, ys);
  plt::pause(0.001);
}

void LaserAccumulator::loadParameters() { scan_topic_ = "/scan"; }

int8_t LaserAccumulator::points_[600][600];

}  // namespace laser_accumulator
