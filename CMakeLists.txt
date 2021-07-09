cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_transform)

find_package(catkin REQUIRED COMPONENTS roscpp  pcl_ros geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros geometry_msgs
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_transform src/pointcloud_transform.cpp)
target_link_libraries(pointcloud_transform ${catkin_LIBRARIES})
