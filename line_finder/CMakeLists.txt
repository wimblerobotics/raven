cmake_minimum_required(VERSION 3.8)
project(line_finder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClosestPointToLineSegment.msg"
  "msg/ClosestPointToLineSegmentList.msg"
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rclcpp rosidl_default_runtime)

include_directories(
  include
)

add_executable(
  lf 
  src/line_extraction_node.cpp
  src/line.cpp
  src/line_extraction.cpp
  src/line_extraction_ros.cpp
)

ament_target_dependencies(lf ${catkin_LIBRARIES})

rosidl_target_interfaces(
  lf
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

ament_target_dependencies(
  lf
  rclcpp
)

target_include_directories(
  lf
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(lf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  lf
  Eigen3
  "rclcpp"
  "rosidl_default_generators"
  "sensor_msgs"
  "visualization_msgs"
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS lf
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
#ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${dependencies})

ament_package()
