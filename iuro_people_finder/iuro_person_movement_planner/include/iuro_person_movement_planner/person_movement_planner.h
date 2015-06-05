//*  person_movement_planner                                  
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This class sends "random" goals to move_base. 
*                Future work: make it more human like behaviour. Long distance goals to office or lab labeled locations, switching to idle motions when the goal is achieved, and repeat again
*                and again.       
*                                                          
*   Action client to: move_base  
*
*   Service client of: map_server::GetMap  	           
*
*   Params:  
*            - 
*            -     
*/ 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#ifndef __PERSON__MOVEMENT__PLANNER__
#define __PERSON__MOVEMENT__PLANNER__

class person_movement_planner{

public:
	person_movement_planner();

private:

	// ros node, listener and publishers
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;

	// clients
	ros::ServiceClient map_client_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

	// data
	nav_msgs::OccupancyGrid map_;

	void planning_thread();
	
	bool check_goal(int x, int y, int r);
};




#endif
