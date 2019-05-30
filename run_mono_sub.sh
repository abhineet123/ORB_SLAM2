#!/usr/bin/env bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:`pwd`/Examples/ROS

print_help()
{
  echo -e "Usage:\n\t$0 scale_factor resize_factor cloud_max_x cloud_min_x cloud_max_z cloud_min_z free_thresh occupied_thresh"
  echo -e "\nExamples:"
  echo -e "\t$0 5 3 29 -25 48 -12 0.55 0.50 1 5 (For outdoor scene)"
  echo -e "\t$0 30 2 5 -5 5 -5 0.55 0.50 1 5 (For indoor scene)"
}

if [ "$#" -lt "10" ]; then
  print_help
  exit 0
fi

rosrun ORB_SLAM2 Monosub $@
