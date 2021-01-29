#include "../include/cloud_to_gridmap.h"

CloudToGridMap::CloudToGridMap()
{
  initParams();
  subscribeAndPublish();
}

CloudToGridMap::~CloudToGridMap()
{}

void CloudToGridMap::initParams()
{
  nh_.param<std::string>("/cloud_to_gridmap/sub_map_topic", sub_map_topic_, " ");

  nh_.param<double>("/cloud_to_gridmap/map_min_z", map_min_z_, 2.0);
  nh_.param<double>("/cloud_to_gridmap/map_max_z", map_max_z_, 2.5);
  nh_.param<double>("/cloud_to_gridmap/map_resolution", map_resolution_, 0.5);
  nh_.param<int>("/cloud_to_gridmap/map_pyramid_level", map_pyramid_level_, 1);
  nh_.param<int>("/cloud_to_gridmap/max_points_per_cell", max_points_per_cell_, 5);

  cloud_in_.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void CloudToGridMap::subscribeAndPublish()
{
  sub_globalmap_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_map_topic_, 1, &CloudToGridMap::globalMapHandler, this);
  pub_gridmap_   = nh_.advertise<nav_msgs::OccupancyGrid>("gridmap", 1, true);
}

void CloudToGridMap::globalMapHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *cloud_in_);

  double max_x, max_y;

  for(size_t i=0; i<cloud_in_->size(); i++)
  {
    double x = cloud_in_->points[i].x;
    double y = cloud_in_->points[i].y;

    if(i==0)
    {
      max_x = x;
      max_y = y;
    }

    if(abs(x) >= max_x) max_x = abs(x);
    if(abs(y) >= max_y) max_y = abs(y);
  }

  auto map_2d = slice(*cloud_in_, map_min_z_, map_max_z_);

  ROS_INFO_STREAM("Set Map " << map_2d.size() << " points");

  if (map_2d.size() < 128)
  {
    ROS_WARN_STREAM("Num points in the sliced map is too small!!");
    ROS_WARN_STREAM("Change the slice range parameters!!");
  }

  int map_width = max_x*2;
  int map_height = max_y*2;

  set_map(map_2d, map_resolution_, map_width/map_resolution_, map_height/map_resolution_, map_pyramid_level_, max_points_per_cell_);

  pub_gridmap_.publish(gridmap()->to_rosmsg());
}

CloudToGridMap::Points2D CloudToGridMap::slice(const pcl::PointCloud<pcl::PointXYZ> &cloud, double min_z, double max_z) const
{
  Points2D points_2d;
  points_2d.reserve(cloud.size());

  for (int i = 0; i < cloud.size(); i++)
  {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z)
    {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }

  return points_2d;
}

void CloudToGridMap::set_map(const CloudToGridMap::Points &map_points, double resolution, int width, int height, int pyramid_levels, int max_points_per_cell)
{
  gridmap_pyramid.resize(pyramid_levels);
  gridmap_pyramid[0].reset(new OccupancyGridMap(resolution, width, height));
  gridmap_pyramid[0]->insert_points(map_points, max_points_per_cell);

  for (int i = 1; i < pyramid_levels; i++)
  {
    gridmap_pyramid[i] = gridmap_pyramid[i - 1]->pyramid_up();
  }
}

std::shared_ptr<const OccupancyGridMap> CloudToGridMap::gridmap() const
{
  return gridmap_pyramid[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_to_gridmap");

  ROS_INFO("\033[1;32m---->\033[0m Cloud to GridMap Node is Started.");

  CloudToGridMap CG;

  ros::spin();
  return 0;
}
