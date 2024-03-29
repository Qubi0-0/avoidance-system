#!/bin/bash
cd PX4-Autopilot

DONT_RUN=1 make px4_sitl_default gazebo
source ~/projects/mgr/devel/setup.zsh
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

cd launch
roslaunch avoidance.launch
