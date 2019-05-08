#!/usr/bin/env bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:`pwd`/Examples/ROS

print_help()
{
  echo -e "Usage:\n\t$0 path_to_camera_settings"
  echo -e "\nExamples:"
  echo -e "\n\t$0 Examples/RGB-D/TUM1.yaml"
}

if [ "$#" -lt "1" ]; then
  print_help
  exit 0
fi

rosrun ORB_SLAM2 RGBDpub Vocabulary/ORBvoc.txt $1
