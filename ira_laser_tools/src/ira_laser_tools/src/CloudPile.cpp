#include "ira_laser_tools/CloudPile.hpp"

CloudPile::CloudPile(pcl::PCLPointCloud2 first_cloud, builtin_interfaces::msg::Time scan_stamp,
                     rclcpp::Time creation_time, int index, int size)
                      : size(size)
{

  this->clouds.resize(size);
  this->clouds_modified.resize(size);

  this->add(index, first_cloud);
  this->first_cloud_time_stamp = rclcpp::Time(scan_stamp);
  this->creation_time = creation_time;

}

CloudPile::~CloudPile(){
}

void CloudPile::add(int index, pcl::PCLPointCloud2 cloud){

  this->clouds[index] = cloud;
  this->clouds_modified[index] = true;
  this->cloud_count++;
}

pcl::PCLPointCloud2 CloudPile::merge_to_one_cloud(){

  pcl::PCLPointCloud2 merged_cloud = clouds[0];
  for (int i = 1; i < this->size; ++i) {
    if(this->clouds_modified[i])
      pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
  }

  return merged_cloud;
}

bool CloudPile::is_complete(){
  if (this->cloud_count == this->size)
    return true;
  else
    return false;
}

bool CloudPile::is_index_filled(int index){
  return this->clouds_modified[index];
}

rclcpp::Time CloudPile::get_stamp_time(){
  return this->first_cloud_time_stamp;
}

rclcpp::Time CloudPile::get_creation_time(){
  return (this->creation_time);
}