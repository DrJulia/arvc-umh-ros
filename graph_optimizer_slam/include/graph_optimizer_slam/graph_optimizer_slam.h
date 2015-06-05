#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/features2d/features2d.hpp>
#include <nav_msgs/GetMap.h>
#include <graph_optimizer_slam/global_graph.h>

class graph_optimizer_slam{

public:
	// constructor
	graph_optimizer_slam();
	virtual ~graph_optimizer_slam(){};

private:

	// ros node handles and tfs
	ros::NodeHandle nh_;
	ros::NodeHandle nh2_;
	ros::CallbackQueue queue_;
	ros::AsyncSpinner myspinner_;
	ros::NodeHandle private_nh_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

	// servers, clients, publishers and subscribers:
	ros::ServiceServer map_server_;
	ros::Publisher map_pub_;
	ros::Timer transform_timer_;
	std::vector<ros::Subscriber> local_maps_sub_;	

	// parameters
	int num_robots_;
	std::string robot_frame_;
	double keyMapDistanceTh_;
	double keyMapTimeTh_;
	double overlapingTh_;
	bool debugging_;
	std::string ref_frame_, map_frame_;
	
	// internal state variables
	bool map_initialized_;
	nav_msgs::OccupancyGrid fused_map_;

	tf::Transform map_transform_;
	global_graph graph;

	// sinchronization mutex to access the map
 	boost::mutex map_mutex_;

	// callbacks
	void local_map_callback(const nav_msgs::OccupancyGrid& lmap);
	bool map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);
	void tranform_publish_callback(const ros::TimerEvent& t);

	// other methods
	bool checkForKeyMap( const nav_msgs::OccupancyGrid& newmap);
	void global_map_fusion();
	void global_map_fusion_DT();
	void fusemap(const cv::Mat map_to_insert, const tf::Transform& inserted_map_origin, nav_msgs::OccupancyGrid& map_to_fuse_with );
};
