//#include <Eigen/Dense>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

// parameters
const float scale_factor = 3;
const float resize_factor = 5;
const float grid_max_x = 29.0;
const float grid_min_x = -25.0;
const float grid_max_z = 48.0;
const float grid_min_z = -12.0;

const float free_thresh = 0.55;
const float occupied_thresh = 0.50;

const float upper_left_x = -1.5;
const float upper_left_y = -2.5;
const int resolution = 10;
cv::Mat occupied_counter, visit_counter;
float norm_factor_x, norm_factor_z;
int h, w;

using namespace std;

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud){
	ROS_INFO("I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
		pt_cloud->header.seq);
}
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose){
	ROS_INFO("I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
		camera_pose->header.seq);
}
void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){
	ROS_INFO("I heard: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
		pts_and_pose->header.seq);
	updateGridMap(pts_and_pose);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Monosub");
	ros::start();

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z*scale_factor;
	w = grid_res_x*scale_factor;
	printf("grid_size: (%d, %d)\n", h, w);

	occupied_counter.create(h, w, CV_32SC1);
	visit_counter.create(h, w, CV_32SC1);
	occupied_counter.setTo(cv::Scalar(0));
	visit_counter.setTo(cv::Scalar(0));


	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);

	ros::NodeHandle nodeHandler;
	ros::Subscriber sub_cloud = nodeHandler.subscribe("cloud_in", 1000, cloudCallback);
	//ros::Subscriber sub_kf = nodeHandler.subscribe("camera_pose", 1000, kfCallback);
	ros::Subscriber sub_pt_array = nodeHandler.subscribe("pts_and_pose", 1000, ptCallback);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	ros::shutdown();

	return 0;
}

void getMixMax(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose,
	geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt) {

	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < pts_and_pose->poses.size(); ++i){
		const geometry_msgs::Point& curr_pt = pts_and_pose->poses[i].position;
		if (curr_pt.x < min_pt.x) { min_pt.x = curr_pt.x; }
		if (curr_pt.y < min_pt.y) { min_pt.y = curr_pt.y; }
		if (curr_pt.z < min_pt.z) { min_pt.z = curr_pt.z; }

		if (curr_pt.x > max_pt.x) { max_pt.x = curr_pt.x; }
		if (curr_pt.y > max_pt.y) { max_pt.y = curr_pt.y; }
		if (curr_pt.z > max_pt.z) { max_pt.z = curr_pt.z; }
	}
}

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){

	//cout << endl << "Saving grid map to " << filename << " ..." << endl;

	// Init Grid Statictics
	// We set the resolution as 10mm, all points and keyframes are in the range of (-3, -4) to (3, 1),
	// so the grid map size is 600x500
	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;


	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;

	float okf_pos_x = kf_location.x;
	float okf_pos_y = kf_location.z;

	int okf_pos_grid_x = int(floor((okf_pos_x - grid_min_x) * norm_factor_x));
	int okf_pos_grid_y = int(floor((okf_pos_y - grid_min_z) * norm_factor_z));

	if (okf_pos_grid_x < 0 || okf_pos_grid_x >= w)
		return;

	if (okf_pos_grid_y < 0 || okf_pos_grid_y >= h)
		return;

	for (int i = 1; i < pts_and_pose->poses.size(); i++){

		const geometry_msgs::Point &curr_pt = pts_and_pose->poses[i].position;

		float mp_pos_x = curr_pt.x*scale_factor;
		float mp_pos_y = curr_pt.z*scale_factor;

		int mp_pos_grid_x = int(floor((mp_pos_x - grid_min_x) * norm_factor_x));
		int mp_pos_grid_y = int(floor((mp_pos_y - grid_min_z) * norm_factor_z));


		if (mp_pos_grid_x < 0 || mp_pos_grid_x >= w)
			continue;

		if (mp_pos_grid_y < 0 || mp_pos_grid_y >= h)
			continue;

		// Increment the occupency account of the grid cell where map point is located
		++occupied_counter.at<int>(mp_pos_grid_y, mp_pos_grid_x);

		//cout << "----------------------" << endl;
		//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

		// Get all grid cell that the line between keyframe and map point pass through
		int x0 = okf_pos_grid_x;
		int y0 = okf_pos_grid_y;
		int x1 = mp_pos_grid_x;
		int y1 = mp_pos_grid_y;
		bool steep = (abs(y1 - y0) > abs(x1 - x0));
		if (steep){
			x0 = okf_pos_grid_y;
			y0 = okf_pos_grid_x;
			x1 = mp_pos_grid_y;
			y1 = mp_pos_grid_x;
		}
		if (x0 > x1){
			x0 = mp_pos_grid_y;
			x1 = okf_pos_grid_y;
			y0 = mp_pos_grid_x;
			y1 = okf_pos_grid_x;
		}
		int deltax = x1 - x0;
		int deltay = abs(y1 - y0);
		double error = 0;
		double deltaerr = ((double)deltay) / ((double)deltax);
		int y = y0;
		int ystep = (y0 < y1) ? 1 : -1;
		for (int x = x0; x <= x1; x++){
			if (steep) {
				++visit_counter.at<int>(x, y);
				//++visit_counter[y][x];
			}
			else {
				++visit_counter.at<int>(y, x);
				//++visit_counter[x][y];
			}
			error = error + deltaerr;
			if (error >= 0.5)
			{
				y = y + ystep;
				error = error - 1.0;
			}
		}
	}
	cv::Mat grid_map(h, w, CV_32FC1, cv::Scalar(0));
	cv::Mat grid_map_thresh(h, w, CV_8UC1, cv::Scalar(0));

	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++){
			int visits = visit_counter.at<int>(j, i);
			int occupieds = occupied_counter.at<int>(j, i);

			if (visits == 0 || occupieds == 0){
				grid_map.at<float>(j, i) = 0.5;
			} else {
				grid_map.at<float>(j, i) = 1 - float(occupieds / visits);
			}
			if (grid_map.at<float>(j, i) >= free_thresh) {
				grid_map_thresh.at<uchar>(j, i) = 255;
			}
			else if (grid_map.at<float>(j, i) < free_thresh && grid_map.at<float>(j, i) >= occupied_thresh) {
				grid_map_thresh.at<uchar>(j, i) = 128;
			} else {
				grid_map_thresh.at<uchar>(j, i) = 0;
			}
		}
	}
	cv::imshow("grid_map_thresh", grid_map_thresh);
	cv::waitKey(1);
	//cout << endl << "Grid map saved!" << endl;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps){
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

	for (int i = 0; i < nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
	}
}


