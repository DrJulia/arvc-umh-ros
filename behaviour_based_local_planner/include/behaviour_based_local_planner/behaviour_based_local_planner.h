//*  behaviour_based_local_planner
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This node consists in a reactive behaviour based planner for exploration of the expected safe zone around the robot                  
*                                                          
*   Subscribes: 
*            - sensor_msgs::laser_scan
*                                                       
*   Publishes:  
*            - cmd_vel
*
*   Action Server:
*            - configure in mode local exploration or go to gateway
*
*   Params:  
*            - int actionradius: radius (in cells) for the area assigned to the nodes on the tree, should me similar to the max range of the sensor
*            - int dilationradius: in cells, used to filter and remove noise and narrow areas, should be similar to the robot circumscribed radius
*/ 

#ifndef __BEHAVIOUR_BASED_LOCAL_PLANNER__
#define __BEHAVIOUR_BASED_LOCAL_PLANNER__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <binMap.h>
#include <OGMros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <behaviour_based_local_planner/localPotentialField.h>
#include <behaviour_based_local_planner/SetState.h>
#include <behaviour_based_local_planner/SetGateway.h>

class local_planner{

public:

	static const int REACTIVE_EXPLORATION_STATE = 0;
	static const int CHANGE_ZONE_STATE = 1;
	static const int ACTIVE_LOCALIZATION_STATE = 2;

	local_planner();
	virtual ~local_planner();
	
	typedef struct gateway{
		float x;
		float y;
		float scale;
		gateway(): x(0.0f),y(0.0f), scale(0.0f){}
	}gateway;

protected:

	ros::NodeHandle nh_;
	ros::NodeHandle nh2_;
	ros::CallbackQueue queue_;
	ros::AsyncSpinner myspinner_;
	ros::NodeHandle private_nh_;
	tf::TransformListener tf_listener_;
	boost::mutex map_mutex;
	boost::mutex gateway_mutex_;

	// subscribers
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_filter_sub_;	// laser topic subscriber
	tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;		// filters laser until transforms are available

	// publishers	
	ros::Publisher speed_pub_;
	ros::Publisher map_pub_;
	ros::Publisher map_meta_data_pub_;
	ros::Publisher slam_map_pub_;
	ros::Publisher slam_map_meta_data_pub_;

	// service clients
	ros::ServiceClient global_map_client_;

	// servers	
	ros::ServiceServer state_server_;
	ros::ServiceServer gateway_server_;

	// Data
	nav_msgs::OccupancyGrid map_;
	std::vector<unsigned int> occCount_;
	std::vector<unsigned int> totalCount_;
	OGMros omap_;

	nav_msgs::OccupancyGrid SLAMmap_;
	std::vector<unsigned int> SLAMoccCount_;
	std::vector<unsigned int> SLAMtotalCount_;
	OGMros SLAMomap_;

	geometry_msgs::Twist speedCommand_;
	nav_msgs::OccupancyGrid last_global_map_;

	// parameters
	int actionradius_;
	int dilationradius_;
	int width_;
	int height_;
	double resolution_;
	string ref_frame_;
	string robot_frame_;
	bool publish_map_;
	int timeHorizon_;
	double markingRange_;
	int numMarkingCells_;
	double control_loop_freq_;
	int numrobots_;
	std::vector<std::string> robotr_frame_;
	std::string other_robots_frames_;

	// main update
	ros::Timer timer_;
	void timer_callback(const ros::TimerEvent& t);

	// topics callback
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

	//servers callbacks
	bool gateway_callback(behaviour_based_local_planner::SetGateway::Request& request, behaviour_based_local_planner::SetGateway::Response& response);
	bool state_callback(behaviour_based_local_planner::SetState::Request& request, behaviour_based_local_planner::SetState::Response& response);

	// auxiliary functions
	void clearFootprint(const point& rcell);
	void clearing(const sensor_msgs::LaserScan& scan, const tf::StampedTransform& laser_pose);
	void marking(const sensor_msgs::LaserScan& scan, const tf::StampedTransform& laser_pose);
	void clear();
	void clearSLAMmap();
	void rollwindowx(int diff, const nav_msgs::OccupancyGrid& globalMap, const tf::StampedTransform& ref_to_map);
	void rollwindowy(int diff, const nav_msgs::OccupancyGrid& globalMap, const tf::StampedTransform& ref_to_map);

	// behaviour control
	localPotentialField globalField_;
	void processOMap(const point& cell, const gridMapInterface& omap, const binMap& esz, localPotentialField& avObs, localPotentialField& goFro, localPotentialField& goUZ, localPotentialField& gates);
	void processGoPre(const point& poscell, const binMap& ppmap, const binMap&  esz, localPotentialField& goPre);
	std::vector<pointf> gateway_; // gateway
	void processGoGoal(const point& pos, const binMap& esz, localPotentialField& goGoal);
	void processAvRob(const point& poscell, const std::vector<point> robotcells, const binMap& esz, localPotentialField& avRob);
	void updateField (const point& poscell, const std::vector<point> robotcells);
	pointf getResponse(const localPotentialField& lpf, const pose& currentPos);
	speed regulator(float currentHeading, pointf& f);
	double vmax_;
	double wmax_;
	double k1_;
	double k2_;

	// States
	void setState(int state);
	int state_;
	int localMinimaCounter_;

	// beahaviour parameters
	int lzwidth_;
	int lzheight_;

	bool avoidObstaclesEnabled_;
	bool goToFrontierEnabled_;
	bool goToUnexploredZonesEnabled_;
	bool goToGatewaysEnabled_;
	bool avoidOtherRobotsEnabled_;
	bool goToPrecisePosesEnabled_;
	bool goToGoalEnabled_;

	double avoidObstaclesWeight_;
	double goToFrontierWeight_;
	double goToUnexploredZonesWeight_;
	double goToGatewaysWeight_;
	double avoidOtherRobotsWeight_;
	double goToPrecisePosesWeight_;
	double goToGoalWeight_;

	double avoidObstaclesWidth_;
	double goToFrontierWidth_;
	double goToUnexploredZonesWidth_;
	double goToGatewaysWidth_;
	double avoidOtherRobotsWidth_;
	double goToPrecisePosesWidth_;
	double goToGoalWidth_;

};

#endif
