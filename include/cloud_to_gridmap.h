#ifndef CLOUD_TO_GRIDMAP_H
#define CLOUD_TO_GRIDMAP_H

#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "occupancy_gridmap.hpp"

class CloudToGridMap {
public:
  using Points = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  CloudToGridMap();
  ~CloudToGridMap();

  void initParams();
  void subscribeAndPublish();

  void globalMapHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  void set_map(const Points& map_points, double resolution, int width, int height, int pyramid_levels, int max_points_per_cell);

  std::shared_ptr<const OccupancyGridMap> gridmap() const;

private:
  using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
  Points2D slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const;

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_globalmap_;
  ros::Publisher  pub_gridmap_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_;

  std::vector<std::shared_ptr<OccupancyGridMap>> gridmap_pyramid;

  std::string sub_map_topic_;
  double map_max_z_, map_min_z_, map_resolution_;
  int map_pyramid_level_, max_points_per_cell_;
};

#endif //CLOUD_TO_GRIDMAP_H
