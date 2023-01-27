cmake_minimum_required(VERSION 3.0.2)
project(fast_rrt_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  costmap_2d
  nav_core
  base_local_planner
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs costmap_2d nav_core base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fast_rrt_star_planner_lib src/fast_rrt_star_planner.cpp src/fast_rrt_star_node.cpp src/fast_rrt_star_helper.cpp)
target_link_libraries(fast_rrt_star_planner_lib ${catkin_LIBRARIES})
