cmake_minimum_required(VERSION 3.8)
project(line_finder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

ament_target_dependencies(
  lf
  Eigen3
  rclcpp
  rosidl_default_generators
  sensor_msgs
  visualization_msgs
  yaml-cpp
  ${catkin_LIBRARIES}
)

target_include_directories(
  lf
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  laser_accumulator
  src/laser_accumulator.cpp
  src/laser_accumulator_node.cpp
)

target_link_libraries(laser_accumulator
  ${PYTHON_LIBRARIES}
)

ament_target_dependencies(
  laser_accumulator
  geometry_msgs
  matplotlib_cpp
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

target_include_directories(
  laser_accumulator
  PUBLIC
  matplotlib_cpp
  include_directories(${PYTHON_INCLUDE_DIRS})
  ${rclcpp_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_target_interfaces(
  lf
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(lf ${YAML_CPP_LIBRARIES})

install(
  TARGETS lf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS laser_accumulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install include files.
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install script files.
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
#ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${dependencies})

ament_package()
