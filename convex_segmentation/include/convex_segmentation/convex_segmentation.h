#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/features2d/features2d.hpp>
#include <nav_msgs/GetMap.h>

class convex_segmentation{

public:
	// constructor
	convex_segmentation();
	virtual ~convex_segmentation(){};

private:

	ros::NodeHandle nh_;
	ros::NodeHandle nh2_;
	ros::CallbackQueue my_queue_;
	ros::NodeHandle private_nh_;

	tf::TransformListener tf_listener_;
	ros::ServiceClient map_client_;					// map service client 
	ros::ServiceServer map_server_;
	ros::Publisher map_pub_;
	ros::Timer timer_;
	ros::Timer transform_timer_;

	// parameters
	int num_robots_;
	float delay_;

	tf::Transform maps_transform_;
	std::string frame_id_1_, frame_id_2_;

	// fused map
	bool map_initialized_;
	nav_msgs::OccupancyGrid fused_map_;
	tf::Transform map1_to_map2_;

 	boost::mutex map_mutex_;

	// callbacks
	void timer_callback(const ros::TimerEvent& t);
	bool map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);

	// methods
	void segment(const cv::Mat& map, cv::Mat& segmented);
	void flood_fill(const cv::Mat& distance, cv::Mat& labels);
	void rec_flood_fill(const cv::Mat& distance, cv::Mat& labels, cv::Mat& seeds, int i, int j, int& numlabels);

	// map structs conversions
	void OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, cv::Mat& mask);

	void process_map(nav_msgs::GetMap* request, cv::Mat* mapasimg, cv::Mat* mask);

};
