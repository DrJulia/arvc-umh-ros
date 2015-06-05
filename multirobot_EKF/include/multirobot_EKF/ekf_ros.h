//*  ekf_ros
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This node runs a multirobot EKF with AR-markers                    
*                                                          
*   Subscribes to: 
*            - Multiple topics of ar_pose/ARMarkers especified by marker_topics: markers detected in each robot
*   
*   Publishes: 
*	     - sensor_msgs/PointCloud markers_map: global_markers_map for visualization in RVIZ
*            - nav_msgs/Odometry for each robot
*
*   Params:  
*            - int number_of_robots
*            - string marker_sources
*/ 

#pragma once
#ifndef __MULTIROBOT_EKF_SLAM_WRAPPER_FOR_ROS__
#define __MULTIROBOT_EKF_SLAM_WRAPPER_FOR_ROS__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <multirobot_EKF/EKF_SLAM.h>
#include <ar_pose/ARMarkers.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class ekf_ros{

public:

	ekf_ros();
	~ekf_ros();
	
private:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// Inputs
	tf::TransformListener tf_listener_;						// transforms listener 
	tf::TransformBroadcaster tf_broadcaster_;						// transforms listener 
	
	// parameters
	int num_robots_;
	std::string marker_sources_;
	int nummarks_;
	double maxdist_;
	double mahTh_, descTh_;
	bool matchByDist_, matchByDesc_;
	double alpha1_, alpha2_, alpha3_, alpha4_;
	double covini_x_, covini_y_, covini_th_;
	std::string ekf_frame_;
	std::string robot_frame_;
	std::string odom_frame_;
	std::string camera_frame_;

	// inputs
	message_filters::Subscriber<ar_pose::ARMarkers>* marker_sub_;			// pointer to array of markers subscriber
	tf::MessageFilter<ar_pose::ARMarkers>** marker_sub_filter_;			// filter to syncronize markers and odometries

	// outputs
	ros::Publisher map_pub_;							// map publisher as a point cloud
	ros::Publisher* localization_pub_;						// pointer to array of localization publishers

	EKFSLAM* slam_;									// the EKF object
	pose* lastodom_;
	tf::Transform* camera_transforms_;

	// callbacks
	void markers_callback(const ar_pose::ARMarkers::ConstPtr& markers, int robotid);
	void fillMapMessage(visualization_msgs::MarkerArray& msg, const ros::Time& stamp);

};

#endif
