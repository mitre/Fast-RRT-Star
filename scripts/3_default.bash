#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash && \
rm -rf build/ devel/ && \
catkin_make -DCMAKE_BUILD_TYPE=Release && \
source devel/setup.bash && \
roslaunch husky_navigation move_base_mapless_demo.launch
