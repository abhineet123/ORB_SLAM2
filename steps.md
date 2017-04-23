install usb_cam with:
 sudo apt-get install ros-indigo-usb-cam

copy the launch file:
/opt/ros/indigo/share/usb_cam/launch/usb_cam-test.launch
to an editable location and change video_device to /dev/video1 to use the external camera

run
roslaunch ~/usb_cam-test.launch
to start it

add:
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/abhineet/ORB_SLAM2/Examples/ROS
export LIBRARY_PATH=${LIBRARY_PATH}:/home/abhineet/ORB_SLAM2/lib:/home/abhineet/ORB_SLAM2/Thirdparty/DBoW2/lib:/home/abhineet/ORB_SLAM2/Thirdparty/g2o/lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/abhineet/ORB_SLAM2/lib:/home/abhineet/ORB_SLAM2/Thirdparty/DBoW2/lib:/home/abhineet/ORB_SLAM2/Thirdparty/g2o/lib
to bashrc

---> run orb slam with:: <!--- 
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/mono.yaml /camera/image_raw:=usb_cam/image_raw
---> using params:

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 674.828820
Camera.fy: 671.076483
Camera.cx: 313.837639
Camera.cy: 249.457471

Camera.k1: -0.074566
Camera.k2: 0.099668
Camera.p1: -0.000740
Camera.p2: -0.002609
Camera.k3: 0.000000

# Camera frames per second 
Camera.fps: 30.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

gmapping launch location: <!--- 
/opt/ros/indigo/share/turtlebot_navigation/launch/gmapping_demo.launch
---> run orb slam on KITTI with: <!--- 
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml /home/abhineet/KITTI/00
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI04-12.yaml /home/abhineet/KITTI/05
---> run on TUM-RGBD with: <!--- 
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM3.yaml /home/abhineet/TUM-RGBD/rgbd_dataset_freiburg3_walking_halfsphere
---> navigate using the map and visualize with: <!--- 
roslaunch turtlebot_gazebo turtlebot_world.launch
---> or <!--- 
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/empty.world
---> for empty world <!--- 
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/abhineet/ORB_SLAM2/grid_map.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch
---> running orb slam with gmapping: <!--- 
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml  /home/abhineet/KITTI/00
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node
rosrun gmapping slam_gmapping
rosrun rviz rviz
---> 
Displays->Add->Map
set Topic to /map

get all map points for the rosbag: <!--- 
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/demo_cam.yaml /camera/image_raw:=usb_cam/image_raw
--->  start publisher with dataset: <!--- 
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml /home/abhineet/KITTI/00 0
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/TUM3.yaml /home/abhineet/TUM-RGBD/rgbd_dataset_freiburg3_walking_halfsphere
--->  start publisher with camera: <!--- 
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/mono.yaml 0
---> start publisher with default topic (/camera/image_raw): <!--- 
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/mono.yaml -1
---> start publisher with custom topic: <!--- 
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/demo_cam.yaml -1 /usb_cam/image_raw
---> 
rosrun ORB_SLAM2 Monosub <scale_factor> <resize_factor> <cloud_max_x> <cloud_min_x> <cloud_max_z> <cloud_min_z> <free_thresh> <occupied_thresh> <use_local_counters> <visit_thresh> <use_gaussian_counters> <use_boundary_detection> <use_height_thresholding> <normal_thresh_deg> <enable_goal_publishing> <show_camera_location>

start subscriber on kitti 00: <!--- 
rosrun ORB_SLAM2 Monosub 5 3 29 -25 48 -12 0.55 0.50 1 5
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.45 0.40 1 5 1 0 1

rosrun ORB_SLAM2 Monosub 1 3 29 -25 48 -12 0.55 0.50 1 5
---> with Gaussian counters: <!--- 
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 1
---> with Gaussian counters and contour detecton <!--- 
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 1 1
---> with contour and without gaussian counters <!--- 
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 0 1
---> with Gaussian counters and ground point filtering, no contour detection and normal thresholding of 75 degrees: <!--- 
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 1 0 1 75
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 0 0 0 75
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 1 0 1 1
---> without Gaussian counters and with ground point filtering and no contour: <!--- 
rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 0 0 1
---> start subscriber on rosbag: <!--- 
rosrun ORB_SLAM2 Monosub 30 5 2 -2 2 -2 0.55 0.50 1 5
rosrun ORB_SLAM2 Monosub 30 2 6 -6 6 -6 0.55 0.50 1 5
---> with camera location <!--- 
rosrun ORB_SLAM2 Monosub 20 1 10 -15 20 -10 0.55 0.50 1 5 1 1 1 75
rosrun ORB_SLAM2 Monosub 20 1 10 -15 20 -10 0.55 0.50 1 5 1 0 1 75
rosrun ORB_SLAM2 Monosub 20 1 10 -15 20 -10 0.55 0.50 1 5 0 0 1 75 
rosrun ORB_SLAM2 Monosub 10 2 10 -15 20 -10 0.55 0.50 1 5 0 1 0
---> running rosbag of recording: <!--- 
rosbag play "/media/abhineet/Win 8/bags/2017-04-03-20-35-36.bag" -r 0.5
rosbag play "/media/abhineet/Win 8/bags/2017-04-09-22-08-04.bag"
---> faulty rosbag <!--- 
rosbag play "/media/abhineet/Win 8/bags/2017-04-03-13-37-07.bag"
---> running rviz with preloaded map display and topic name: <!--- 
rosrun rviz rviz -d grid_map.rviz
rosrun rviz rviz -d navigation.rviz
---> test sequence to get goal publishing thing working: <!--- 
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/empty.world
roslaunch amcl_demo.launch
roslaunch view_navigation.launch
rosrun robot_pose_publisher robot_pose_publisher
---> record a rosbag of orbslam and grid mapping: <!--- 
rosbag record pts_and_pose all_kf_and_pts map map_metadata move_base_simple/goal initialpose --output-name=rb_kitti00_gauss_height.bag
rosbag record pts_and_pose all_kf_and_pts --output-name=rb_csc3_pub.bag
---> play this rosbag  <!--- 
rosbag play ~/rb_kitti00_gauss_height.bag
rosbag play ~/rb_kitti00_pub.bag
rosbag play ~/rb_csc3_pub.bag
rosbag play ~/rb_csc3_pub_2.bag
rosbag play ~/rb_csc3_pub_3.bag
---> record and play the subscriber rosbag  <!--- 
rosbag record map map_metadata --output-name=rb_kitti00_sub.bag
rosbag record map map_metadata --output-name=rb_csc_sub.bag
rosbag play ~/rb_kitti00_sub_f0_55_o0_50_l1_v5_g1_b1_h1_n45_c350.bag
rosbag play ~/rb_csc_sub_f0_55_o0_50_l1_v5_g1_b1_h1_n75_c350.bag
---> 

