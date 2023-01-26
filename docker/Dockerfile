ARG ROS_DISTRO=noetic
FROM ros:$ROS_DISTRO-ros-base
ARG DEBIAN_FRONTEND=noninteractive

# Install ros desktop, husky dependencies, fast_rrt_ros dependencies, and valgrind for debugging
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-husky-desktop \
    ros-$ROS_DISTRO-husky-simulator \
    ros-$ROS_DISTRO-husky-navigation \
    ros-$ROS_DISTRO-costmap-2d \
    ros-$ROS_DISTRO-nav-core \
    ros-$ROS_DISTRO-base-local-planner \
    valgrind

# Copy launch files and husky description to appropriate locations
ARG HUSKY_NAV_LAUNCH=husky_navigation/launch
ARG HUSKY_DES_URDF=husky_description/urdf
COPY ./docker/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star.launch /opt/ros/$ROS_DISTRO/share/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star.launch
COPY ./docker/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_debug.launch /opt/ros/$ROS_DISTRO/share/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_debug.launch
COPY ./docker/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_mapless_demo.launch /opt/ros/$ROS_DISTRO/share/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_mapless_demo.launch
COPY ./docker/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_mapless_demo_debug.launch /opt/ros/$ROS_DISTRO/share/$HUSKY_NAV_LAUNCH/move_base_fast_rrt_star_mapless_demo_debug.launch
COPY ./docker/$HUSKY_DES_URDF/husky.urdf.xacro /opt/ros/$ROS_DISTRO/share/$HUSKY_DES_URDF/husky.urdf.xacro

# Declare some environment variables
ENV ROS_WORKSPACE=/root/catkin_ws
ENV ROS_PACKAGE_PATH=/root/catkin_ws/src:/opt/ros/$ROS_DISTRO/share

# Add setup.bash sourcing to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

# Copy catkin_ws CMakeLists, scripts, and planner configuration to image
WORKDIR /root/catkin_ws/
COPY ./docker/CMakeLists.txt ./src/CMakeLists.txt
COPY ./scripts/ ./scripts
COPY ./config/planner.yaml /opt/ros/$ROS_DISTRO/share/husky_navigation/config/planner.yaml

# Copy Fast-RRT* to catkin workspace in image
WORKDIR /root/catkin_ws/src/fast_rrt_ros
COPY ./include/ ./include/
COPY ./src/ ./src/
COPY ./CMakeLists.txt ./CMakeLists.txt
COPY ./fast_rrt_star_planner_plugin.xml ./fast_rrt_star_planner_plugin.xml
COPY ./LICENSE ./LICENSE
COPY ./package.xml ./package.xml
COPY ./README.md ./README.md
