#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "line_finder/msg/closest_point_to_line_segment.hpp"
#include "line_finder/msg/closest_point_to_line_segment_list.hpp"
#include "line_finder/msg/line_segment.hpp"
#include "line_finder/msg/line_segment_list.hpp"
#include "line.h"
#include "line_extraction.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace line_finder {

class LineExtractionROS : public rclcpp::Node {
 public:
  // Constructor / destructor
  LineExtractionROS();
  ~LineExtractionROS();
  // Running
//   void run();

 private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<line_finder::msg::LineSegmentList>::SharedPtr
      line_publisher_;
  rclcpp::Publisher<line_finder::msg::ClosestPointToLineSegmentList>::SharedPtr
      closest_point_to_line_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;

  // Parameters
  double bearing_std_dev_;
  std::string frame_id_;
  double least_sq_angle_thresh_;
  double least_sq_radius_thresh_;
  double max_line_gap_;
  double max_range_;
  double min_line_length_;
  int min_line_points_;
  double min_range_;
  double min_split_dist_;
  double outlier_dist_;
  bool publish_markers_;
  double range_std_dev_;
  std::string scan_topic_;

  // Line extraction
  LineExtraction line_extraction_;
  bool data_cached_;  // true after first scan used to cache data
  // Members
  void loadParameters();
  void populateClosetPointToLineSegListMsg(
      const std::vector<Line> &,
      line_finder::msg::ClosestPointToLineSegmentList &line_list_msg);
  void populateLineSegListMsg(const std::vector<Line> &,
                              line_finder::msg::LineSegmentList &line_list_msg);
  void populateMarkerMsg(const std::vector<Line> &,
                         visualization_msgs::msg::Marker &msg);
  void populateIntersectionMarkers(const std::vector<Line> &,
                                   visualization_msgs::msg::Marker &msg);
  void cacheData(const sensor_msgs::msg::LaserScan::ConstSharedPtr);
  void laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
};

}  // namespace line_finder

#endif
