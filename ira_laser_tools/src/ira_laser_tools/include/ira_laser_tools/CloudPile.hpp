#ifndef IRA_LASER_TOOLS__CLOUDPILE_HPP_
#define IRA_LASER_TOOLS__CLOUDPILE_HPP_

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "builtin_interfaces/msg/time.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"

class CloudPile{
  public:
    CloudPile(pcl::PCLPointCloud2 first_cloud, builtin_interfaces::msg::Time stamp,
              rclcpp::Time creation_time, int index, int size);
    ~CloudPile();

    void add(int index, pcl::PCLPointCloud2 cloud);
    rclcpp::Time get_stamp_time();
    rclcpp::Time get_creation_time();
    pcl::PCLPointCloud2 merge_to_one_cloud();
    bool is_complete();
    bool is_index_filled(int index);

  private:
    int size;
    int cloud_count = 0;
    rclcpp::Time first_cloud_time_stamp;
    rclcpp::Time creation_time;
    std::vector<pcl::PCLPointCloud2> clouds;
    std::vector<bool> clouds_modified;
};



#endif  // IRA_LASER_TOOLS__CLOUDPILE_HPP_
