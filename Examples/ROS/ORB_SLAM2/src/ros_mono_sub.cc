//#include <Eigen/Dense>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// parameters
float scale_factor = 3;
float resize_factor = 5;
float cloud_max_x = 10;
float cloud_min_x = -10.0;
float cloud_max_z = 16;
float cloud_min_z = -5;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
float thresh_diff = 0.01;
int visit_thresh = 0;
float upper_left_x = -1.5;
float upper_left_y = -2.5;
const int resolution = 10;
unsigned int use_local_counters = 0;
unsigned int use_gaussian_counters = 0;
unsigned int gaussian_kernel_size = 3;
bool show_camera_location = false;
bool add_contour = false;
int cam_radius = 2;

// no. of keyframes between successive goal messages are published
unsigned int goal_gap = 20;


float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
cv::Mat grid_map_rgb;
cv::Mat gauss_kernel;
float norm_factor_x, norm_factor_z;
int h, w;
unsigned int n_kf_received;
bool loop_closure_being_processed = false;
ros::Publisher pub_grid_map;
ros::Publisher pub_goal;
ros::Publisher pub_initial_pose;
nav_msgs::OccupancyGrid grid_map_msg;

//#ifdef COMPILEDWITHC11
//std::chrono::steady_clock::time_point start_time, end_time;
//#else
//std::chrono::monotonic_clock::time_point start_time, end_time;
//#endif
//bool got_start_time;

float kf_pos_x, kf_pos_z;
int kf_pos_grid_x, kf_pos_grid_z;
geometry_msgs::Quaternion kf_orientation;
unsigned int kf_id = 0;
unsigned int init_pose_id = 0, goal_id = 0;

using namespace std;

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud);
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose);
void saveMap(unsigned int id = 0);
void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts);
void parseParams(int argc, char **argv);
void printParams();
void showGridMap(unsigned int id = 0);
void getMixMax(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose,
	geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt);
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
	cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
void getGridMap();

int main(int argc, char **argv){
	ros::init(argc, argv, "Monosub");
	ros::start();

	////tell the action client that we want to spin a thread by default
	//MoveBaseClient ac("move_base", true);
	//move_base_msgs::MoveBaseGoal goal;
	////wait for the action server to come up
	//while (!ac.waitForServer(ros::Duration(5.0))){
	//	ROS_INFO("Waiting for the move_base action server to come up");
	//}
	////we'll send a goal to the robot to move 1 meter forward
	//goal.target_pose.header.frame_id = "base_link";

	parseParams(argc, argv);
	printParams();


	grid_max_x = cloud_max_x*scale_factor;
	grid_min_x = cloud_min_x*scale_factor;
	grid_max_z = cloud_max_z*scale_factor;
	grid_min_z = cloud_min_z*scale_factor;
	printf("grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;
	printf("grid_size: (%d, %d)\n", h, w);
	n_kf_received = 0;
	
	global_occupied_counter.create(h, w, CV_32FC1);
	global_visit_counter.create(h, w, CV_32FC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	grid_map_msg.data.resize(h*w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = 1.0/scale_factor;

	grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);
	grid_map_rgb.create(h*resize_factor, w*resize_factor, CV_8UC3);
	printf("output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32FC1);
	local_visit_counter.create(h, w, CV_32FC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	gauss_kernel = cv::getGaussianKernel(gaussian_kernel_size, -1);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	printf("norm_factor_x: %f\n", norm_factor_x);
	printf("norm_factor_z: %f\n", norm_factor_z);

	ros::NodeHandle nodeHandler;
	ros::Subscriber sub_pts_and_pose = nodeHandler.subscribe("pts_and_pose", 1000, ptCallback);
	ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("all_kf_and_pts", 1000, loopClosingCallback);
	pub_grid_map = nodeHandler.advertise<nav_msgs::OccupancyGrid>("map", 1000);
	pub_goal = nodeHandler.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
	pub_initial_pose = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

	//ros::Subscriber sub_cloud = nodeHandler.subscribe("cloud_in", 1000, cloudCallback);
	//ros::Subscriber sub_kf = nodeHandler.subscribe("camera_pose", 1000, kfCallback);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	ros::shutdown();
	cv::destroyAllWindows();
	saveMap();

	return 0;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud){
	ROS_INFO("I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
		pt_cloud->header.seq);
}
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose){
	ROS_INFO("I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
		camera_pose->header.seq);
}
void saveMap(unsigned int id) {
	printf("saving maps with id: %u\n", id);
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
	//if (pts_and_pose->header.seq==0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	printf("Received exit message\n");
	//	ros::shutdown();
	//	exit(0);
	//}
//	if (!got_start_time) {
//#ifdef COMPILEDWITHC11
//		start_time = std::chrono::steady_clock::now();
//#else
//		start_time = std::chrono::monotonic_clock::now();
//#endif
//		got_start_time = true;
//	}
	if (loop_closure_being_processed){ return; }
	updateGridMap(pts_and_pose);

//#ifdef COMPILEDWITHC11
//	end_time = std::chrono::steady_clock::now();
//#else
//	end_time = std::chrono::monotonic_clock::now();
//#endif
//	double curr_time = std::chrono::duration_cast<std::chrono::duration<double>>(start_time - end_time).count();

	grid_map_msg.info.map_load_time = ros::Time::now();
	pub_grid_map.publish(grid_map_msg);

	if (kf_id == 0) {
		geometry_msgs::PoseWithCovariance init_pose;
		init_pose.pose.position.x = kf_pos_grid_x;
		init_pose.pose.position.y = kf_pos_grid_z;
		init_pose.pose.position.z = 0;
		init_pose.pose.orientation = kf_orientation;
		cv::Mat(6, 6, CV_64FC1, init_pose.covariance.elems).setTo(0);
		geometry_msgs::PoseWithCovarianceStamped init_pose_stamped;
		init_pose_stamped.header.frame_id = "base_frame";
		init_pose_stamped.header.stamp = ros::Time::now();
		init_pose_stamped.header.seq = ++init_pose_id;
		init_pose_stamped.pose = init_pose;
		pub_initial_pose.publish(init_pose_stamped);
	}
	else if (kf_id % goal_gap == 0) {
		geometry_msgs::PoseStamped goal;
		goal.pose.position.x = kf_pos_grid_x;
		goal.pose.position.y = kf_pos_grid_z;
		goal.pose.orientation = kf_orientation;
		goal.header.frame_id = "base_frame";
		goal.header.stamp = ros::Time::now();
		goal.header.seq = ++goal_id;
		pub_goal.publish(goal);
	}
	++kf_id;

	//goal.target_pose.header.stamp = ros::Time::now();
	//goal.target_pose.pose.position.x = kf_pos_grid_x;
	//goal.target_pose.pose.position.y = kf_pos_grid_z;
	//goal.target_pose.pose.orientation = pts_and_pose->poses[0].orientation;
	//ROS_INFO("Sending goal");
	//ac.sendGoal(goal);
	//ac.waitForResult();
	//if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	//	ROS_INFO("Hooray, the base moved 1 meter forward");
	//else
	//	ROS_INFO("The base failed to move forward 1 meter for some reason");
}
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (all_kf_and_pts->header.seq == 0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	ros::shutdown();
	//	exit(0);
	//}
	loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
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
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied, 
	cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z) {
	float pt_pos_x = curr_pt.x*scale_factor;
	float pt_pos_z = curr_pt.z*scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));


	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<float>(pt_pos_grid_z, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

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
			++visited.at<float>(x, y);
		}
		else {
			++visited.at<float>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5){
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z) {
	unsigned int end_id = start_id + n_pts;
	if (use_local_counters) {
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}

		for (int row = 0; row < h; ++row){
			for (int col = 0; col < w; ++col){
				if (local_map_pt_mask.at<uchar>(row, col) == 0) {
					local_occupied_counter.at<float>(row, col) = 0;
				}
				else {
					local_occupied_counter.at<float>(row, col) = local_visit_counter.at<float>(row, col);
				}
			}
		}
		if (use_gaussian_counters) {
			cv::filter2D(local_occupied_counter, local_occupied_counter, CV_32F, gauss_kernel);
			cv::filter2D(local_visit_counter, local_visit_counter, CV_32F, gauss_kernel);
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
	}
	else {
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
	}
}

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){

	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//printf("max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor, 
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);

	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	//printf("Received frame %u \n", pts_and_pose->header.seq);

	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	kf_orientation = pts_and_pose->poses[0].orientation;

	kf_pos_x = kf_location.x*scale_factor;
	kf_pos_z = kf_location.z*scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;
	++n_kf_received;
	unsigned int n_pts = pts_and_pose->poses.size() - 1;
	//printf("Processing key frame %u and %u points\n",n_kf_received, n_pts);
	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

	getGridMap();
	showGridMap(pts_and_pose->header.seq);
	//cout << endl << "Grid map saved!" << endl;
}

void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
		printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id){
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
			printf("resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}
		float kf_pos_x = kf_location.x*scale_factor;
		float kf_pos_z = kf_location.z*scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size()) {
			printf("resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;
	}	
	getGridMap();
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	printf("Done. Time taken: %f secs\n", ttrack);
	pub_grid_map.publish(grid_map_msg);
	showGridMap(all_kf_and_pts->header.seq);
}

void getGridMap() {
	for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			float visits = global_visit_counter.at<float>(row, col);
			float occupieds = global_occupied_counter.at<float>(row, col);

			if (visits <= visit_thresh){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
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
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
		}
	}
	if (add_contour) {
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(grid_map_thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		for (unsigned int i = 0; i<contours.size(); i++){
			drawContours(grid_map_thresh, contours, i, CV_RGB(0, 0, 0), 2, 8, hierarchy, 0, cv::Point());
		}

	}
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}
void showGridMap(unsigned int id) {
	cv::imshow("grid_map_msg", cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data())));
	if (show_camera_location) {
		grid_map_thresh_resized.convertTo(grid_map_rgb, grid_map_rgb.type());
		cv::circle(grid_map_rgb, cv::Point(kf_pos_grid_x*resize_factor, kf_pos_grid_z*resize_factor),
			cam_radius, CV_RGB(255, 0, 0));
		cv::imshow("grid_map_thresh_resized", grid_map_rgb);
	} else {
		cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);
	}	
	//cv::imshow("grid_map", grid_map);
	int key = cv::waitKey(1);
	int key_mod = key % 256;
	if (key == 27 || key_mod == 27) {
		cv::destroyAllWindows();
		ros::shutdown();
		exit(0);
	}
	else if (key == 'f' || key_mod == 'f') {
		free_thresh -= thresh_diff;
		if (free_thresh <= occupied_thresh){ free_thresh = occupied_thresh + thresh_diff; }

		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'F' || key_mod == 'F') {
		free_thresh += thresh_diff;
		if (free_thresh > 1){ free_thresh = 1; }
		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'o' || key_mod == 'o') {
		occupied_thresh -= thresh_diff;
		if (free_thresh < 0){ free_thresh = 0; }
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 'O' || key_mod == 'O') {
		occupied_thresh += thresh_diff;
		if (occupied_thresh >= free_thresh){ occupied_thresh = free_thresh - thresh_diff; }
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 's' || key_mod == 's' || key == 'S' || key_mod == 'S') {
		saveMap(id);
	}
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
		cloud_max_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		cloud_min_x = atof(argv[arg_id++]);
	}	
	if (argc > arg_id){
		cloud_max_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		cloud_min_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		free_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		occupied_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		use_local_counters = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		visit_thresh = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		use_gaussian_counters = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		gaussian_kernel_size = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		show_camera_location = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		cam_radius = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		add_contour = atoi(argv[arg_id++]);
	}
}

void printParams() {
	printf("Using params:\n");
	printf("scale_factor: %f\n", scale_factor);
	printf("resize_factor: %f\n", resize_factor);
	printf("cloud_max: %f, %f\t cloud_min: %f, %f\n", cloud_max_x, cloud_max_z, cloud_min_x, cloud_min_z);
	//printf("cloud_min: %f, %f\n", cloud_min_x, cloud_min_z);
	printf("free_thresh: %f\n", free_thresh);
	printf("occupied_thresh: %f\n", occupied_thresh);
	printf("use_local_counters: %d\n", use_local_counters);
	printf("visit_thresh: %d\n", visit_thresh);
	printf("use_gaussian_counters: %d\n", use_gaussian_counters);
	printf("gaussian_kernel_size: %d\n", gaussian_kernel_size);
	printf("show_camera_location: %d\n", show_camera_location);
	printf("cam_radius: %d\n", cam_radius);
	printf("add_contour: %d\n", add_contour);
}

