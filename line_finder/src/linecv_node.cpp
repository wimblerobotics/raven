#include "line_finder/linecv.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<linecv::LineCv>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
