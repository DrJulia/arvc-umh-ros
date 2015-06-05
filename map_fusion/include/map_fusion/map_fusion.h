#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/features2d/features2d.hpp>
#include <nav_msgs/GetMap.h>

class map_fusion{

public:
	// constructor
	map_fusion();
	virtual ~map_fusion(){};

private:

	ros::NodeHandle nh_;
	ros::NodeHandle nh2_;
	ros::CallbackQueue my_queue_;
	ros::NodeHandle private_nh_;

	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;
	std::vector<ros::ServiceClient> map_clients_;					// map service client 
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
	void tranform_publish_callback(const ros::TimerEvent& t);
	bool map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);

	// methods
	void find_features(cv::Mat& map, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int indexrobot, const cv::Mat& mask);

	void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
                         const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                         std::list<cv::DMatch>& filteredMatches12, int knn=1, float disth=100.0f);

	void align_maps(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch> matches, tf::Transform& map1_to_map2, const cv::Mat& map1, const cv::Mat& map2);

	void fusemaps(const nav_msgs::OccupancyGrid& map1, const nav_msgs::OccupancyGrid& map2, const tf::Transform& map1_to_map2, nav_msgs::OccupancyGrid& outmap );

	// map structs conversions
	void OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, cv::Mat& mask);

	void process_map_thread(int n, nav_msgs::GetMap* request, std::vector<cv::KeyPoint>* keypoints, cv::Mat* descriptors, cv::Mat* mapasimg, cv::Mat* mask);

};
