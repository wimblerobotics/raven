#include <rclcpp/rclcpp.hpp>

#include "line_finder/laser_accumulator.hpp"

using namespace laser_accumulator;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserAccumulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}