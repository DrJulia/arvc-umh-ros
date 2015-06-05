//*  explore_tree_planner
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This node consists of a planner that sends goals to move_base in order to explore the environment                          
*                                                          
*   Client of: 
*            - nav_msgs::occupancy_grid map
*           
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

#ifndef __EXPLORE_TREE_PLANNER__
#define __EXPLORE_TREE_PLANNER__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <binMap.h>
//#include <actionlib/server/simple_action_server.h>
//#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <explore_tree_planner/treeNode.h>
#include <OGMros.h>
#include <visualization_msgs/Marker.h>

class explore_tree_planner{

public:

	explore_tree_planner();
	virtual ~explore_tree_planner(){};
	
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
	ros::ServiceClient map_client_;					// map service client 
	ros::ServiceClient state_client_;				
	ros::ServiceClient gateway_client_;				

	// publishers	
	ros::Publisher rvizMarkerPub_;
	visualization_msgs::Marker rvizMarker_;

	// Action Clients
	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

	// Action Servers	

	// Data
	OGMros* map_;

	// parameters
	int actionradius_;
	int sensorradius_;
	int dilationradius_;
	int min_cluster_size_;
	string robot_frame_;
	double control_loop_freq_;

	// main update
//	void planning_thread();
	ros::Timer timer_;
	void timer_callback(const ros::TimerEvent& t);

	/// recursively evaluates all the nodeso of the tree
	void evalTree(treeNode& tree, const nav_msgs::OccupancyGrid& map, int& numGateways, int& numLeafs);
	/// evals the value of a node of the tree
	void evalNode(treeNode& node, const nav_msgs::OccupancyGrid& map);
};

#endif
