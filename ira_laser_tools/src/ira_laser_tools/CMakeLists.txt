cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

set(dependencies
  laser_geometry
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)

find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


###########
## Build ##

###########

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger ${dependencies})
target_link_libraries(laserscan_multi_merger CloudPile ${PCL_LIBRARIES})

add_library(CloudPile src/CloudPile.cpp)
ament_target_dependencies(CloudPile rclcpp pcl_ros)
target_link_libraries(CloudPile ${PCL_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  laserscan_multi_merger CloudPile
  DESTINATION lib/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
