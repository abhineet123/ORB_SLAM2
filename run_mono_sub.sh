#!/usr/bin/env bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:`pwd`/Examples/ROS

# Start subscriber on KITTI 00
#rosrun ORB_SLAM2 Monosub 5 3 29 -25 48 -12 0.55 0.50 1 5

# Start subscriber on indoor scene
rosrun ORB_SLAM2 Monosub 30 2 5 -5 5 -5 0.55 0.50 1 5
