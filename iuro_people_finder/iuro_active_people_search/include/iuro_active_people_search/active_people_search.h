//*  active_people_search                                  
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This node consists of a planner that sends goals to move_base in order to actively seek persons.                             
*                                                          
*   Client of: 
*            - nav_msgs::occupancy_grid map
*            - iuro_people_finder_mapper::people_finder_map people_finder_map       	                   
*                                                          
*   Action Client to: 
*	     - move_base/goal
*
*   Action Services: 
*            - 
*
*   Params:  
*            - int actionradius: radius (in cells) for the area assigned to the nodes on the tree, should me similar to the max range of the sensor
*            - int dilationradius: in cells, used to filter and remove noise and narrow areas, should be similar to the robot circumscribed radius
*	     - int min_cluster_size: in cells, gateway cells clusters of less of this size are not considered
*/ 

#ifndef _IURO_ACTIVE_PEOPLE_SEARCH_ROS_
#define _IURO_ACTIVE_PEOPLE_SEARCH_ROS_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <binMap.h>
#include <iuro_people_finder_mapper/get_people_finder_map.h>
#include <iuro_people_finder_mapper/people_finder_map.h>
#include <iuro_people_finder_mapper/resetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <iuro_active_people_search/treeNode.h>
#include <OGMros.h>

class active_people_search{

public:

	active_people_search();
	virtual ~active_people_search(){};
	
	typedef struct gateway{
		float x;
		float y;
		float scale;
		gateway(): x(0.0f),y(0.0f), scale(0.0f){}
	}gateway;

protected:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	tf::TransformListener tf_listener_;

	// Clients
	ros::ServiceClient map_client_;							// map service client 
	ros::ServiceClient people_map_client_;						// people finder map service client 

	// Action Clients
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

	// Action Servers
	

	// Data
	OGMros* map_;

	// parameters
	int actionradius_;
	int dilationradius_;
	int min_cluster_size_;
	string robot_frame_;

	// main thread
	void planning_thread();

	/// recursively evaluates all the nodeso of the tree
	void evalTree(treeNode& tree, const iuro_people_finder_mapper::people_finder_map& people_map);
	/// evals the value of a node of the tree
	void evalNode(treeNode& node, const iuro_people_finder_mapper::people_finder_map& people_map);
};

#endif
