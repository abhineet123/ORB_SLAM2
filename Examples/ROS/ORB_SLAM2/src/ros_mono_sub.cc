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
float scale_factor = 3;
float resize_factor = 5;
float grid_max_x = 29.0;
float grid_min_x = -25.0;
float grid_max_z = 48.0;
float grid_min_z = -12.0;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
float upper_left_x = -1.5;
float upper_left_y = -2.5;
const int resolution = 10;

cv::Mat occupied_counter, visit_counter;
cv::Mat grid_map, grid_map_thresh, grid_map_thresh_resized;
float norm_factor_x, norm_factor_z;
int h, w;

using namespace std;

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud){
	ROS_INFO("I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
		pt_cloud->header.seq);
}
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose){
	ROS_INFO("I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
		camera_pose->header.seq);
}
void saveMap(unsigned int id = 0) {
	if (id > 0) {
		cv::imwrite("grid_map_" + to_string(id) + ".jpg", grid_map);
		cv::imwrite("grid_map_thresh_" + to_string(id) + ".jpg", grid_map_thresh);
		cv::imwrite("grid_map_thresh_resized" + to_string(id) + ".jpg", grid_map_thresh_resized);
	}
	else {
		cv::imwrite("grid_map.jpg", grid_map);
		cv::imwrite("grid_map_thresh.jpg", grid_map_thresh);
		cv::imwrite("grid_map_thresh_resized.jpg", grid_map_thresh_resized);
	}

}
void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	if (pts_and_pose->header.seq==0) {
		cv::destroyAllWindows();
		saveMap();
		ros::shutdown();
		exit(0);
	}
	updateGridMap(pts_and_pose);
}
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	if (all_kf_and_pts->header.seq == 0) {
		cv::destroyAllWindows();
		saveMap();
		ros::shutdown();
		exit(0);
	}
	resetGridMap(all_kf_and_pts);
}
void parseParams(int argc, char **argv);
void printParams();


int main(int argc, char **argv){
	ros::init(argc, argv, "Monosub");
	ros::start();

	parseParams(argc, argv);
	printParams();

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;
	printf("grid_size: (%d, %d)\n", h, w);

	occupied_counter.create(h, w, CV_32SC1);
	visit_counter.create(h, w, CV_32SC1);
	occupied_counter.setTo(cv::Scalar(0));
	visit_counter.setTo(cv::Scalar(0));

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	printf("norm_factor_x: %f\n", norm_factor_x);
	printf("norm_factor_z: %f\n", norm_factor_z);

	ros::NodeHandle nodeHandler;
	ros::Subscriber sub_pts_and_pose = nodeHandler.subscribe("pts_and_pose", 1000, ptCallback);
	ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("all_kf_and_pts", 1000, loopClosingCallback);
	//ros::Subscriber sub_cloud = nodeHandler.subscribe("cloud_in", 1000, cloudCallback);
	//ros::Subscriber sub_kf = nodeHandler.subscribe("camera_pose", 1000, kfCallback);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	ros::shutdown();
	cv::destroyAllWindows();
	saveMap();

	return 0;
}

void getMixMax(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose,
	geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt) {

	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	for (unsigned int i = 0; i < pts_and_pose->poses.size(); ++i){
		const geometry_msgs::Point& curr_pt = pts_and_pose->poses[i].position;
		if (curr_pt.x < min_pt.x) { min_pt.x = curr_pt.x; }
		if (curr_pt.y < min_pt.y) { min_pt.y = curr_pt.y; }
		if (curr_pt.z < min_pt.z) { min_pt.z = curr_pt.z; }

		if (curr_pt.x > max_pt.x) { max_pt.x = curr_pt.x; }
		if (curr_pt.y > max_pt.y) { max_pt.y = curr_pt.y; }
		if (curr_pt.z > max_pt.z) { max_pt.z = curr_pt.z; }
	}
}
void processMapPt(const geometry_msgs::Point &curr_pt,
	int kf_pos_grid_x, int kf_pos_grid_z) {
	float pt_pos_x = curr_pt.x*scale_factor;
	float pt_pos_z = curr_pt.z*scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));


	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied_counter.at<int>(pt_pos_grid_z, pt_pos_grid_x);

	//cout << "----------------------" << endl;
	//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

	// Get all grid cell that the line between keyframe and map point pass through
	int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep){
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1){
		swap(x0, x1);
		swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x){
		if (steep) {
			++visit_counter.at<int>(x, y);
			//++visit_counter[y][x];
		}
		else {
			++visit_counter.at<int>(y, x);
			//++visit_counter[x][y];
		}
		error = error + deltaerr;
		if (error >= 0.5){
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void getGridMap() {
	for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			int visits = visit_counter.at<int>(row, col);
			int occupieds = occupied_counter.at<int>(row, col);

			if (visits == 0 || occupieds == 0){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1 - float(occupieds / visits);
			}
			if (grid_map.at<float>(row, col) >= free_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else {
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
		}
	}
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}
void showGridMap(unsigned int id = 0) {
	cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);
	int key = cv::waitKey(1);
	if (key == 27) {
		cv::destroyAllWindows();
		ros::shutdown();
		exit(0);
	}
	else if (key == 'f') {
		free_thresh -= 1;
		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'F') {
		free_thresh += 1;
		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'o') {
		occupied_thresh -= 1;
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 'O') {
		occupied_thresh -= 1;
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 's') {
		saveMap(id);
	}
}


void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){

	//cout << endl << "Saving grid map to " << filename << " ..." << endl;

	// Init Grid Statictics
	// We set the resolution as 10mm, all points and keyframes are in the range of (-3, -4) to (3, 1),
	// so the grid map size is 600x500

	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//printf("max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor, 
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);
	//printf("min_pt: %f, %f\n", min_pt.x*scale_factor, min_pt.z*scale_factor);
	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;

	float kf_pos_x = kf_location.x*scale_factor;
	float kf_pos_z = kf_location.z*scale_factor;

	int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;

	for (unsigned int pt_id = 1; pt_id < pts_and_pose->poses.size(); ++pt_id){
		processMapPt(pts_and_pose->poses[pt_id].position, kf_pos_grid_x, kf_pos_grid_z);
	}
	getGridMap();
	showGridMap(pts_and_pose->header.seq);
	//cout << endl << "Grid map saved!" << endl;
}

void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	visit_counter.setTo(0);
	occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
		printf("Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);

	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < all_kf_and_pts->poses.size(); ++kf_id){
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
			printf("Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}
		float kf_pos_x = kf_location.x*scale_factor;
		float kf_pos_z = kf_location.z*scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			return;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			return;

		if (id + n_pts >= all_kf_and_pts->poses.size()) {
			printf("Unexpected end of the input array: only %u out of %u elements found",
				id + n_pts, all_kf_and_pts->poses.size());
			return;
		}
		for (unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
			processMapPt(all_kf_and_pts->poses[++id].position, kf_pos_grid_x, kf_pos_grid_z);
		}
	}	
	getGridMap();
	showGridMap(all_kf_and_pts->header.seq);
}

void parseParams(int argc, char **argv) {
	int arg_id = 1;
	if (argc > arg_id){
		scale_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		resize_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		grid_max_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		grid_min_x = atof(argv[arg_id++]);
	}	
	if (argc > arg_id){
		grid_max_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		grid_min_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		free_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		occupied_thresh = atof(argv[arg_id++]);
	}
}

void printParams() {
	printf("Using params:\n");
	printf("scale_factor: %f\n", scale_factor);
	printf("resize_factor: %f\n", resize_factor);
	printf("grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);
	//printf("grid_min: %f, %f\n", grid_min_x, grid_min_z);
	printf("free_thresh: %f\n", free_thresh);
	printf("occupied_thresh: %f\n", occupied_thresh);
}

