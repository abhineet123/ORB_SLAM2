/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <time.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Monopub");
    ros::start();

	if (argc != 4){
		cerr << endl << "Usage: rosrun ORB_SLAM2 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id" << endl;
		return 1;
	}
	cv::VideoCapture cap_obj;
	bool read_from_camera = false;
	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	if (isInteger(std::string(argv[3]))) {
		read_from_camera = true;
		int camera_id = atoi(argv[3]);
		printf("Reading images from camera with id %d\n", camera_id);
		cap_obj.open(camera_id);
		if (!(cap_obj.isOpened())) {
			printf("Camera stream could not be initialized successfully\n");
			ros::shutdown();
			return EXIT_FAILURE;
		}
		int img_height = cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
		int img_width = cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);
		printf("Images are of size: %d x %d\n", img_width, img_height);
	} else {
		LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
	}
	int n_images = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nodeHandler;
	//ros::Publisher pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	//ros::Publisher pub_kf = nodeHandler.advertise<geometry_msgs::PoseStamped>("camera_pose", 1000);

	//ImageGrabber igb(&SLAM, &pub_cloud);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	// Main loop
	ros::Rate loop_rate(5);
	cv::Mat im;
	double tframe = 0;
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	for (int frame_id = 0; read_from_camera || frame_id < n_images; ++frame_id){
		
		if (read_from_camera) {
			cap_obj.read(im);
#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
			tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
			//printf("fps: %f\n", 1.0 / tframe);
		} else {
			// Read image from file
			im = cv::imread(vstrImageFilenames[frame_id], CV_LOAD_IMAGE_UNCHANGED);
			tframe = vTimestamps[frame_id];
		}
		if (im.empty()){
			cerr << endl << "Failed to load image at: " << vstrImageFilenames[frame_id] << endl;
			return 1;
		}
		// Pass the image to the SLAM system
		cv::Mat curr_pose = SLAM.TrackMonocular(im, tframe);
	
		if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {

			ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			//while (pKF->isBad())
			//{
			//	Trw = Trw*pKF->mTcp;
			//	pKF = pKF->GetParent();
			//}

			vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
			sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

			// Transform all keyframes so that the first keyframe is at the origin.
			// After a loop closure the first keyframe might not be at the origin.
			cv::Mat Two = vpKFs[0]->GetPoseInverse();

			Trw = Trw*pKF->GetPose()*Two;
			cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
			cv::Mat Tcw = lit*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
			//geometry_msgs::Pose camera_pose;
			//std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
			std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
			int n_map_pts = map_points.size();

			//printf("n_map_pts: %d\n", n_map_pts);

			//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

			geometry_msgs::PoseArray pt_array;
			//pt_array.poses.resize(n_map_pts + 1);

			geometry_msgs::Pose camera_pose;

			camera_pose.position.x = twc.at<float>(0);
			camera_pose.position.y = twc.at<float>(1);
			camera_pose.position.z = twc.at<float>(2);

			camera_pose.orientation.x = q[0];
			camera_pose.orientation.y = q[1];
			camera_pose.orientation.z = q[2];
			camera_pose.orientation.w = q[3];

			pt_array.poses.push_back(camera_pose);

			//printf("Done getting camera pose\n");

			for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

				if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat wp = map_points[pt_id-1]->GetWorldPos();

				if (wp.empty()) {
					//printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = wp.at<float>(0);
				curr_pt.position.y = wp.at<float>(1);
				curr_pt.position.z = wp.at<float>(2);
				pt_array.poses.push_back(curr_pt);
				//printf("Done getting map point %d\n", pt_id);
			}
			//sensor_msgs::PointCloud2 ros_cloud;
			//pcl::toROSMsg(*pcl_cloud, ros_cloud);
			//ros_cloud.header.frame_id = "1";
			//ros_cloud.header.seq = ni;

			//printf("valid map pts: %lu\n", pt_array.poses.size()-1);

			//printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
			//pub_cloud.publish(ros_cloud);
			pt_array.header.frame_id = "1";
			pt_array.header.seq = frame_id;
			pub_pts_and_pose.publish(pt_array);
			//pub_kf.publish(camera_pose);
		}
		//cv::imshow("Press escape to exit", im);
		//if (cv::waitKey(1) == 27) {
		//	break;
		//}
		ros::spinOnce();
		loop_rate.sleep();
	}
	//ros::spin();

    // Stop all threads
    SLAM.Shutdown();
	geometry_msgs::PoseArray pt_array;
	pt_array.header.seq = 0;
	pub_pts_and_pose.publish(pt_array);
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while (!fTimes.eof())
	{
		string s;
		getline(fTimes, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	string strPrefixLeft = strPathToSequence + "/image_0/";

	const int nTimes = vTimestamps.size();
	vstrImageFilenames.resize(nTimes);

	for (int i = 0; i<nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
	}
}


