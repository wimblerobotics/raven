#include <rclcpp/rclcpp.hpp>

#include "line_finder/line_extraction_ros.hpp"

using namespace line_finder;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineExtractionROS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}