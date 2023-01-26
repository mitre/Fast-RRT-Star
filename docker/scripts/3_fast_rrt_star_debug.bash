#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash && \
rm -rf build/ devel/ && \
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
source devel/setup.bash && \
roslaunch husky_navigation move_base_fast_rrt_star_mapless_demo.launch
