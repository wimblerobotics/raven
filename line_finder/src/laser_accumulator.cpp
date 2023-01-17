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

void LaserAccumulator::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) const {
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
      // double x = transform_stamped.transform.translation.x;
      // double y = transform_stamped.transform.translation.y;
      std::vector<float> xs(scan_msg->ranges.size());
      std::vector<float> ys(scan_msg->ranges.size());
      float angle = scan_msg->angle_min;
      for (float range : scan_msg->ranges) {
        if ((range < 50.0) && (range > -50.0)) {
          xs.push_back(range * cos(angle));
          ys.push_back(range * sin(angle));
        }
        angle += scan_msg->angle_increment;
      }
      plt::scatter(xs, ys);
      plt::show();
    }
  } catch (tf2::TransformException& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
  }
}

void LaserAccumulator::loadParameters() { scan_topic_ = "/scan"; }

}  // namespace laser_accumulator
