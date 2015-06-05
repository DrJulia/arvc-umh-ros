//*  people_finder_mapper                                  
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This node, creates and updates a map of the areas where it is more likely to found a human.                              
*                                                          
*   Subscribes to: 
*            - nav_msgs/GridCells sensed_area: cells scanned by the sensor
*            - nav_msgs/OccupancyGrid map       	                   
*                                                          
*   Publishes: 
*	     - iuro_people_finder_mapper/people_finder_map people_finder_map: the output probability map
*            - sensor_msgs/Image people_finder_map_as_image: to view the map easily in rviz
*
*   Services: 
*            - get_people_finder_map: serves the map
*
*   Params:  
*            - double person_speed
*            - string robot_frame     
*/ 

#ifndef _IURO_PEOPLE_FINDER_MAPPER_ROS_
#define _IURO_PEOPLE_FINDER_MAPPER_ROS_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GridCells.h>
#include <binMap.h>
#include <iuro_people_finder_mapper/get_people_finder_map.h>
#include <iuro_people_finder_mapper/people_finder_map.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <iuro_people_finder_mapper/resetAction.h>
#include <actionlib/server/simple_action_server.h>

class people_finder_mapper{

public:

	people_finder_mapper();

	virtual ~people_finder_mapper();

protected:

	// Callbacks
	bool mapCallback(iuro_people_finder_mapper::get_people_finder_map::Request &req, iuro_people_finder_mapper::get_people_finder_map::Response &res);
	void omap_callback(const nav_msgs::OccupancyGrid::ConstPtr& omap);
	void sensed_area_callback(const nav_msgs::GridCells::ConstPtr& sensed_area);
	void resetCallback(const iuro_people_finder_mapper::resetGoalConstPtr& msg);
	
	void initialize(const nav_msgs::OccupancyGrid& map, const point& rpos);
	void clearSensedArea(const binMap& area);
	void clearOccupiedArea(const nav_msgs::OccupancyGrid& omap, const point& rpos);
	void expandUncertainty(const nav_msgs::OccupancyGrid& omap);
	float countProb(const binMap& area) const;
	float countAccessible(const point* p, int numpoints, binMap& accessible, const nav_msgs::OccupancyGrid& omap) const;
	float floodfill(const int& nx, const int& ny, binMap& processed, const nav_msgs::OccupancyGrid& omap) const;
	void rollwindowx(int diff);
	void rollwindowy(int diff);
	void normalizemap();
	void checksum();

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// Inputs
	tf::TransformListener tf_listener_;						// transforms listener 
	
	//ros::Subscriber sensed_area_sub_;					
	message_filters::Subscriber<nav_msgs::GridCells> sensed_area_filter_sub_;	// laser topic subscriber
	tf::MessageFilter<nav_msgs::GridCells>* sensed_area_filter_;			// filters laser until transforms are available	
	//ros::Subscriber omap_sub_;							// subscribe to occupancy grid map
	message_filters::Subscriber<nav_msgs::OccupancyGrid> omap_filter_sub_;		// laser topic subscriber
	tf::MessageFilter<nav_msgs::OccupancyGrid>* omap_filter_;			// filters laser until transforms are available	
	
	// Outputs
	ros::Publisher map_pub_;							// map topic publisher
	ros::Publisher map_meta_data_pub_;						// map info publisher
	ros::Publisher map_as_image_pub_;						// map as image publisher
	//ros::Publisher cam_info_pub_;							// map as image publisher cam info

	// Services
	ros::ServiceServer map_serv_;							// map service server 
	actionlib::SimpleActionServer<iuro_people_finder_mapper::resetAction> reset_as_;

	// Parameters
	std::string robot_frame_;
	double person_speed_;
	int occupied_threshold_;
	bool use_unknown_;

	// Map data
	iuro_people_finder_mapper::people_finder_map* pfmap_; 
	//sensor_msgs::CameraInfo ci_;
	sensor_msgs::Image img_;
	nav_msgs::OccupancyGrid omap_;

	// auxiliar
 	boost::mutex map_mutex_;
	bool initialized_;
	ros::Time lastTimeExpanded_;	
	int offsetx_;
	int offsety_;
	float def_prob_;

private:

	static void* workingThread(void * arguments);
	float newThread(const point* p, int numpoints, binMap& accessible, const nav_msgs::OccupancyGrid& omap, people_finder_mapper* pfm);

};

#endif
