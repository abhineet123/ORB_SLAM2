#!/usr/bin/env bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:`pwd`/Examples/ROS

rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt \
  Examples/Monocular/KITTI00-02.yaml \
  /home/${USER}/Documents/dataset/KITTI/2011_09_26/2011_09_26_drive_0002_sync/image_00
