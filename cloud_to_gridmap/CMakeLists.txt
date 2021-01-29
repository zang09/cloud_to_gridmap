cmake_minimum_required(VERSION 3.0.2)
project(cloud_to_gridmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES cloud_to_gridmap
  #CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs
  #DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(cloud_to_gridmap src/cloud_to_gridmap.cpp)
target_link_libraries(cloud_to_gridmap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

