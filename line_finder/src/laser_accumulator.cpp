// ###ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator

#include "line_finder/laser_accumulator.hpp"

#include <matplotlibcpp.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
namespace plt = matplotlibcpp;

namespace laser_accumulator {

LaserAccumulator::LaserAccumulator()
    : Node("laser_accumulator_node"),
      do_plot_(false),
      initial_persistence_count_(3),
      max_persistence_count_(5) {
  loadParameters();

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
  qos.avoid_ros_namespace_conventions(false);

  lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "persistent_scan", 10);
  scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&LaserAccumulator::laserScanCallback, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &LaserAccumulator::parametersCallback, this, std::placeholders::_1));
}

LaserAccumulator::~LaserAccumulator() {}

rcl_interfaces::msg::SetParametersResult LaserAccumulator::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();

  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters) {
    if ((parameter.get_name() == "do_plot") &&
        (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)) {
      do_plot_ = parameter.as_bool();
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter '%s' changed: %s", parameter.get_name().c_str(),
                  do_plot_ ? "True" : "False");
    } else if ((parameter.get_name() == "initial_persistence_count") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_INTEGER)) {
      initial_persistence_count_ = parameter.as_int();
      if ((initial_persistence_count_ > 0) &&
          (initial_persistence_count_ < 250)) {
        RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                    "Parameter '%s' changed: %d", parameter.get_name().c_str(),
                    initial_persistence_count_);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("LaserAccumulator"),
                     "Parameter '%s' not in [1..249]: %d",
                     parameter.get_name().c_str(), initial_persistence_count_);
        result.successful = false;
        result.reason =
            "Parameter 'initial_persistence_count' is not in [1..249]";
        initial_persistence_count_ = 3;
      }
    } else if ((parameter.get_name() == "max_persistence_count") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_INTEGER)) {
      max_persistence_count_ = parameter.as_int();
      if ((max_persistence_count_ > 0) && (max_persistence_count_ < 250)) {
        RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                    "Parameter '%s' changed: %d", parameter.get_name().c_str(),
                    max_persistence_count_);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("LaserAccumulator"),
                     "Parameter '%s' not in [1..249]: %d",
                     parameter.get_name().c_str(), max_persistence_count_);
        result.successful = false;
        result.reason = "Parameter 'max_persistence_count' is not in [1..249]";
        max_persistence_count_ = 5;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("LaserAccumulator"),
                   "Invalid parameter: %s", parameter.get_name().c_str());
    }
  }

  return result;
}

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
              (points_[x_coord][y_coord] < max_persistence_count_)) {
            points_[x_coord][y_coord] += initial_persistence_count_;
          }
        }
        angle += scan_msg->angle_increment;
      }
    }
  } catch (tf2::TransformException &ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
  }
}

void LaserAccumulator::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  manage_scan(scan_msg);

  sensor_msgs::msg::LaserScan new_scan;
  new_scan = *scan_msg;
  new_scan.header.frame_id = "odom";
  // float angle = scan_msg->angle_min;
  for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
    new_scan.ranges[i] = 10'0000.0;  // Wipe out old scan value for the angle.
  }

  for (size_t x_coord = 0; x_coord < kMAP_PIXELS; x_coord++) {
    for (size_t y_coord = 0; y_coord < kMAP_PIXELS; y_coord++) {
      float offset = kMAP_PIXELS / 2.0;
      float px = x_coord - offset;  // Offset to represent real coordinates.
      float py = y_coord - offset;
      px = px / kPIXELS_PER_METER;  // Scale.
      py = py / kPIXELS_PER_METER;
      float distance_to_point = sqrt((px * px) + (py * py));
      float angle_to_point = atan2(py, px);
      int angle_slot =
          (angle_to_point - scan_msg->angle_min) / scan_msg->angle_increment;
      if (points_[x_coord][y_coord] > 0) {
        // Found a persisteng point.
        if (distance_to_point < new_scan.ranges[angle_slot]) {
          new_scan.ranges[angle_slot] = distance_to_point;
        }
      }
    }
  }

  lidar_pub_->publish(new_scan);

  if (do_plot_) {
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
}

void LaserAccumulator::loadParameters() {
  std::string do_plot_name = "do_plot";
  rcl_interfaces::msg::ParameterDescriptor do_plot_descriptor;
  do_plot_descriptor.name = do_plot_name;
  do_plot_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  do_plot_descriptor.description = "Show the scatter plot?";
  bool do_plot_default = false;
  declare_parameter(do_plot_name, do_plot_default, do_plot_descriptor);

  std::string initial_persistence_count_name = "initial_persistence_count";
  rcl_interfaces::msg::ParameterDescriptor initial_persistence_count_descriptor;
  initial_persistence_count_descriptor.name = initial_persistence_count_name;
  initial_persistence_count_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  initial_persistence_count_descriptor.description =
      "The initial persistence count for a scan point, in [1..249]";
  rcl_interfaces::msg::IntegerRange initial_persistence_count_range;
  initial_persistence_count_range.from_value = 1;
  initial_persistence_count_range.to_value = 249;
  initial_persistence_count_range.step = 1;
  initial_persistence_count_descriptor.integer_range.push_back(
      initial_persistence_count_range);
  int initial_persistence_count_default = 3;
  declare_parameter(initial_persistence_count_name,
                    initial_persistence_count_default,
                    initial_persistence_count_descriptor);

  std::string max_persistence_name = "max_persistence_count";
  rcl_interfaces::msg::ParameterDescriptor max_persistence_count_descriptor;
  max_persistence_count_descriptor.name = max_persistence_name;
  max_persistence_count_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  max_persistence_count_descriptor.description =
      "The maximum persistence count for a scan point, in [1..249]";
  rcl_interfaces::msg::IntegerRange max_persistence_count_range;
  max_persistence_count_range.from_value = 1;
  max_persistence_count_range.to_value = 249;
  max_persistence_count_range.step = 1;
  max_persistence_count_descriptor.integer_range.push_back(
      max_persistence_count_range);
  int max_persistence_count_default = 5;
  declare_parameter(max_persistence_name, max_persistence_count_default,
                    max_persistence_count_descriptor);

  scan_topic_ = "/scan";
}

uint8_t LaserAccumulator::points_[kMAP_PIXELS][kMAP_PIXELS];

}  // namespace laser_accumulator
