//**********************************************************************************
//**                                                                              **
//**   Description:     active_people_search                                      **
//**                                                                              **
//**       Project:     IURO Project                                              **
//**                                                                              **
//**       Created:     14 October 2011                                           **
//**        Author:     Miguel Juliá                                              **
//**       Contact:     mjulia@umh.es                                             **
//**										  **
//**********************************************************************************

#include "iuro_active_people_search/active_people_search.h"
#include "iuro_people_finder_mapper/get_people_finder_map.h"

active_people_search::active_people_search():
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(1.0)),
	move_base_ac_("move_base", true)
{
	// occupancy gridmap client
	map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
	
	// people finder map client
	people_map_client_ = nh_.serviceClient<iuro_people_finder_mapper::get_people_finder_map>("get_people_finder_map");

	// asking for the map
	nav_msgs::GetMap srv;
	bool res=false;
	ros::Duration delay(1.0);
	do{
		ROS_INFO("requesting static_map...");
		res = map_client_.call(srv); 	// we are going to asume a static map, that's why we ask get it from the service instead of subscribing to it
		if (res){
			ROS_INFO("map received");
			map_ = new OGMros(srv.response.map);
	  	}
		else{ 	ROS_ERROR("Failed to call service static_map");
			delay.sleep();
			ROS_INFO("trying again...");
		}
	} while (!res); 

	// setting up move_base action client
	ROS_INFO("waiting for move_base to start...");
  	move_base_ac_.waitForServer();
	ROS_INFO("move_base client running");

	// reading parameters
	private_nh_.param<int>("actionradius", actionradius_, 160);
	private_nh_.param<int>("dilationradius", dilationradius_, 4);
	private_nh_.param<int>("min_cluster_size", min_cluster_size_, 8);
	private_nh_.param<string>("robot_frame", robot_frame_, "base_link");

	// running the goal selection thread
	ROS_INFO("running the person movement planner thread");
	boost::thread workerThread( &active_people_search::planning_thread, this);

}


void active_people_search::planning_thread(){

	while(ros::ok()){  

		// 1. Get the last people finder probability map
		ROS_INFO("1. Requesting people map");
		iuro_people_finder_mapper::get_people_finder_map srv;
		iuro_people_finder_mapper::people_finder_map* people_map;
		bool res=false;
		ros::Duration delay(1.0);
		do{
			res = people_map_client_.call(srv);
			if (res){
				ROS_INFO("people map received");
				people_map = &srv.response.map;
		  	}
			else{ 	ROS_ERROR("Failed to call service get_people_finder_map");
				delay.sleep();
				ROS_INFO("trying again...");
			}
		} while (!res); 

		// 2. Get the robot cell with tf listener
		ROS_INFO("2. Getting people cell");
		tf::StampedTransform ref_to_robot;
		ros::Time stamp = ros::Time::now();
		try{
			tf_listener_.waitForTransform (people_map->header.frame_id, robot_frame_, stamp, ros::Duration(1.0)); 
			tf_listener_.lookupTransform  (people_map->header.frame_id, robot_frame_, stamp, ref_to_robot); 
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("Transform exception!!!");
		}
		geometry_msgs::Point rcell;
		rcell.x = (int)((ref_to_robot.getOrigin().getX()-map_->getXOrigin())/people_map->info.resolution-0.5);
		rcell.y = (int)((ref_to_robot.getOrigin().getY()-map_->getYOrigin())/people_map->info.resolution-0.5);
		ROS_INFO("The robot cell is (%f,%f)",rcell.x,rcell.y);

		// 3. Dynamic programing for partitioning of the environment in a tree with root at the robot cell
		ROS_INFO("3. Creating tree");
		treeNode* t = treeNode::createTreePartitioning(*map_, rcell, dilationradius_, actionradius_, min_cluster_size_);
		
		// 4. Goal decision process: finding the node that maximizes P(n)/C(n) 
		//    where P(n)=people_finder_map_probability in the areas asociated to the branch until node n and C(n) is the cost as distance travelled
		ROS_INFO("4. Finding the maximum");
		evalTree(*t,*people_map); 					// evaluates the P(n)/C(n) for each node
		double maxProfit;
		const treeNode* bestNode = t->findMaxProfitNode(maxProfit); 	// find the max
		std::vector<geometry_msgs::Point> goal_sequence; 		// we store here the sequence of wait points until the final goal
		const treeNode* currentNode = bestNode;
		while(!currentNode->isRoot()){
			goal_sequence.push_back(currentNode->getCell());	// it is saved in inverse order, the last node in the vector is the first way point in the route
			ROS_INFO("adding (%f,%f) to the route", currentNode->getCell().x, currentNode->getCell().y);
			currentNode = &currentNode->getParent();
		}
		delete t;
		
		// 5. Follow the planned route
		ROS_INFO("5. Following the route");
		for (int i = (int)(goal_sequence.size()-1); i >=0 ; i--){ 		// for each wait point to the goal... (inverse order)
			// send the goal and wait until arrival
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "/map";
			// transform to global coordinates
			
			double rxpos = (goal_sequence[i].x+0.5)*map_->getResolution()+map_->getXOrigin(); // TODO: check signs
			double rypos = (goal_sequence[i].y+0.5)*map_->getResolution()+map_->getYOrigin();
			goal.target_pose.pose.position.x = rxpos;
			goal.target_pose.pose.position.y = rypos;
			goal.target_pose.pose.orientation.w = 1.0;
			
			ROS_INFO("sending goal %d/%d (%f,%f)", (int)goal_sequence.size()-i, (int)goal_sequence.size(), rxpos, rypos);
			move_base_ac_.sendGoal(goal);
			
			bool arrived = false;
			bool error = false;
			do {
				move_base_ac_.waitForResult(ros::Duration(1.0));
				if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ arrived = true; continue;}
				if (move_base_ac_.getState() != actionlib::SimpleClientGoalState::ACTIVE) error = true;		
			}while (!arrived && !error);
			
			if (arrived) ROS_INFO("arrived to goal %d/%d (%f,%f)", (int)goal_sequence.size()-i, (int)goal_sequence.size(), rxpos, rypos);

			if (error){
				ROS_INFO("error to achieve goal %d/%d (%f,%f), replanning...", (int)goal_sequence.size()-i, (int)goal_sequence.size(), rxpos, rypos);
				break; // we plan again if there is an error
			}
		}
	}
}

void active_people_search::evalTree(treeNode& tree, const iuro_people_finder_mapper::people_finder_map& people_map){
	evalNode(tree, people_map);
	for (int n = 0; n < tree.getNChildren(); n++){
		evalTree(tree.getChildren(n), people_map);
	}
}

void active_people_search::evalNode(treeNode& node, const iuro_people_finder_mapper::people_finder_map& people_map){

	double value=0;
	for (int i = node.getArea().getRoi().x ; i < node.getArea().getRoi().x + node.getArea().getRoi().width ; i++){
		for (int j = node.getArea().getRoi().y ; j < node.getArea().getRoi().y + node.getArea().getRoi().height ; j++){   
			if (node.getArea().get(i,j)){ 										// for each cell in the region associated to the node
				value += people_map.data[j*people_map.info.width+i];
			}
		}
	}
	node.setAreaValue(value);
	if (!node.isRoot()){
		node.setCumulatedAreaValue(node.getParent().getCumulatedAreaValue() + value);
		node.setProfit(node.getCumulatedAreaValue()/node.getGlobalCost());
	}
	else{
		node.setCumulatedAreaValue(value);
		node.setProfit(0.0);
	}
}




