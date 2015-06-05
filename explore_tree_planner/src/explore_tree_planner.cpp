//**********************************************************************************
//**                                                                              **
//**   Description:     explore_tree_planner                                      **
//**                                                                              **
//**       Project:                                                               **
//**                                                                              **
//**       Created:     12 November 2012                                          **
//**        Author:     Miguel Juliá                                              **
//**       Contact:     mjulia@umh.es                                             **
//**										  **
//**********************************************************************************

#include <behaviour_based_local_planner/behaviour_based_local_planner.h>
#include <explore_tree_planner/explore_tree_planner.h>

explore_tree_planner::explore_tree_planner():
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(1.0)),
//	move_base_ac_("move_base", true),
	map_(0)
{
	// setting up move_base action client
//	ROS_INFO("waiting for move_base to start...");
// 	move_base_ac_.waitForServer();
//	ROS_INFO("move_base client running");

	map_client_ = nh_.serviceClient<nav_msgs::GetMap>("dynamic_map");
	state_client_ = nh_.serviceClient<behaviour_based_local_planner::SetState>("SetState");
	gateway_client_ = nh_.serviceClient<behaviour_based_local_planner::SetGateway>("SetGateway");

	rvizMarkerPub_ = nh_.advertise < visualization_msgs::Marker > ("visualization_marker", 0);

	// reading parameters
	private_nh_.param<int>("actionradius", actionradius_, 140);
	private_nh_.param<int>("sensorradius", sensorradius_, 120);
	private_nh_.param<int>("dilationradius", dilationradius_, 7);
	private_nh_.param<int>("min_cluster_size", min_cluster_size_, 12);
	private_nh_.param<string>("robot_frame", robot_frame_, "base_link");
	private_nh_.param<double>("control_loop_freq", control_loop_freq_, 0.05);

	// running the goal selection thread
	//ROS_INFO("running the explore tree planner thread");
	//boost::thread workerThread( &explore_tree_planner::planning_thread, this);

	// running the control loop timer
	ROS_INFO("initiating high level control loop");
	timer_ = nh_.createTimer(ros::Duration(1/control_loop_freq_), boost::bind(&explore_tree_planner::timer_callback, this, _1));

}




//void explore_tree_planner::planning_thread(){
void explore_tree_planner::timer_callback(const ros::TimerEvent& t){
	if(ros::ok()){ 
		// 1. Get the last map
		ROS_INFO("1. Requesting map");
		nav_msgs::GetMap srv;
		bool res=false;
		ros::Duration delay(0.2);
		do{
			ROS_INFO("requesting dynamic_map...");
			res = map_client_.call(srv); 	
			if (res){
				ROS_INFO("map received %dx%d",srv.response.map.info.width,srv.response.map.info.height);
				if (map_) delete map_;				
				map_ = new OGMros(srv.response.map);
		  	}
			else{ 	ROS_ERROR("Failed to call service map");
				delay.sleep();
				ROS_INFO("trying again...");
			}
		} while (!res); 

		// 2. Get the robot cell with tf listener
		ROS_INFO("2. Getting robot cell...");
		tf::StampedTransform ref_to_robot;
		ros::Time stamp = ros::Time::now();
		try{
			tf_listener_.waitForTransform (srv.response.map.header.frame_id, robot_frame_, stamp, ros::Duration(1.0)); 
			tf_listener_.lookupTransform  (srv.response.map.header.frame_id, robot_frame_, stamp, ref_to_robot); 
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("Transform exception!!!");
		}
		geometry_msgs::Point rcell;

		ROS_INFO("tf pose: %f, %f\n", ref_to_robot.getOrigin().getX(),ref_to_robot.getOrigin().getY());
		ROS_INFO("map ref: %f, %f\n", map_->getXOrigin(),map_->getYOrigin());

		rcell.x = (int)((ref_to_robot.getOrigin().getX()-map_->getXOrigin())/srv.response.map.info.resolution-0.5);
		rcell.y = (int)((ref_to_robot.getOrigin().getY()-map_->getYOrigin())/srv.response.map.info.resolution-0.5);
		ROS_INFO("      The robot cell is (%f,%f)",rcell.x,rcell.y);

		// 3. Dynamic programing for partitioning of the environment in a tree with root at the robot cell
		ROS_INFO("3. Creating the exploration tree...");
		ros::WallTime initPlanTime = ros::WallTime::now();	
		treeNode* t = treeNode::createTreePartitioning(*map_, rcell, dilationradius_, actionradius_, sensorradius_, min_cluster_size_);
		ros::WallTime endPlanTime = ros::WallTime::now();	
		double totalTime = endPlanTime.toSec() - initPlanTime.toSec();

		// 4. Goal decision process: finding the node that maximizes P(n)/C(n) 
		//    where P(n)=people_finder_map_probability in the areas asociated to the branch until node n and C(n) is the cost as distance travelled
		ROS_INFO("4. Finding the maximum...");
		int numGateways=0, numLeafs=0;
		evalTree(*t,srv.response.map, numGateways, numLeafs); 					// evaluates the P(n)/C(n) for each node
		double maxProfit;
		const treeNode* bestNode = t->findMaxProfitNode(maxProfit); 	// find the max
		ROS_INFO("TOTAL TIME = %f secs, Branches: %d, Leafs: %d, Avg: %f node/sec", totalTime, numGateways, numLeafs, numGateways/totalTime);

		behaviour_based_local_planner::SetState statereqmsg;
		behaviour_based_local_planner::SetGateway gatewayreqmsg;
		geometry_msgs::Point pt;					// we store here the sequence of wait points until the final goal
		gatewayreqmsg.request.header.frame_id=srv.response.map.header.frame_id;

		rvizMarker_.header.frame_id = srv.response.map.header.frame_id;
		rvizMarker_.header.stamp = srv.response.map.header.stamp;
		rvizMarker_.action = visualization_msgs::Marker::ADD;
		rvizMarker_.type = visualization_msgs::Marker::ARROW;
		rvizMarker_.scale.x = 0.15f;
		rvizMarker_.scale.y = 0.25f;
		rvizMarker_.scale.z = 0.0f;
		rvizMarker_.color.r = 1.0f;
		rvizMarker_.color.g = 0.0f;
		rvizMarker_.color.b = 0.0f;
		rvizMarker_.color.a = 1.0f;
		rvizMarker_.ns = "decision";
		rvizMarker_.lifetime = ros::Duration (1/control_loop_freq_);

		const treeNode* currentNode = bestNode;
		while(!currentNode->isRoot()){					// we go back from the best to the root
			//if(!currentNode->isLeaf()){				// we add all the gateways to the list
				ROS_INFO("  adding node (%f,%f) to the route", currentNode->getCell().x, currentNode->getCell().y);
				pt.x = srv.response.map.info.origin.position.x + (currentNode->getCell().x +0.5)*srv.response.map.info.resolution;
				pt.y = srv.response.map.info.origin.position.y + (currentNode->getCell().y +0.5)*srv.response.map.info.resolution;
				gatewayreqmsg.request.gateway.push_back(pt);			// it is saved in inverse order, the last node in the vector is the first way point in the route

				rvizMarker_.points.clear();
				geometry_msgs::Point p;
				p.x= currentNode->getParent().getX()*srv.response.map.info.resolution + srv.response.map.info.origin.position.x; 
				p.y= currentNode->getParent().getY()*srv.response.map.info.resolution + srv.response.map.info.origin.position.y;
				rvizMarker_.points.push_back(p);
				rvizMarker_.points.push_back(pt);
				rvizMarker_.id++;
				rvizMarkerPub_.publish (rvizMarker_);		// publish the arrows of the planned path
			//}
			currentNode = &currentNode->getParent();
		}

		if (gatewayreqmsg.request.gateway.size() > 1){					// if we have intermediate gateways
			// go to gateway
			ROS_ERROR("GATEWAY");
			statereqmsg.request.state = local_planner::CHANGE_ZONE_STATE;
			gateway_client_.call(gatewayreqmsg);
			state_client_.call(statereqmsg);

		}
		else{
			// go reactive
			ROS_ERROR("REACTIVE");
			statereqmsg.request.state = local_planner::REACTIVE_EXPLORATION_STATE;
			state_client_.call(statereqmsg);
		}

		delete t;




//		if (goal_sequence.size() <= 1){
//			move_base_msgs::MoveBaseGoal goal;
//			goal.target_pose.header.frame_id = srv.response.map.header.frame_id;
//			double rxpos = (rcell.x-20)*map_->getResolution()+map_->getXOrigin(); 
//			double rypos = (rcell.y)*map_->getResolution()+map_->getYOrigin();
//			goal.target_pose.pose.position.x = rxpos;
//			goal.target_pose.pose.position.y = rypos;
//			goal.target_pose.pose.orientation.w = 1.0;
//			move_base_ac_.sendGoal(goal);
//			move_base_ac_.waitForResult(ros::Duration(5.0));
//		}
//		else{
//			// 5. Follow the planned route
//			ROS_INFO("5. Following the route (%d, nodes)", goal_sequence.size()-1);
//			for (int i = (int)(goal_sequence.size()-2); i >=0 ; i--){ 		// for each way point to the goal... (inverse order)
//				// send the goal and wait until arrival
//				move_base_msgs::MoveBaseGoal goal;
//				goal.target_pose.header.frame_id = srv.response.map.header.frame_id;
//				// transform to global coordinates
//		
//				double rxpos = (goal_sequence[i].x+0.5)*map_->getResolution()+map_->getXOrigin(); 
//				double rypos = (goal_sequence[i].y+0.5)*map_->getResolution()+map_->getYOrigin();
//				double rxpos2 = (goal_sequence[i+1].x+0.5)*map_->getResolution()+map_->getXOrigin(); 
//				double rypos2 = (goal_sequence[i+1].y+0.5)*map_->getResolution()+map_->getYOrigin();
//				goal.target_pose.pose.position.x = rxpos;
//				goal.target_pose.pose.position.y = rypos;
//				double yaw = atan2(rypos-rypos2,rxpos-rxpos2);
//				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
//				//goal.target_pose.pose.orientation.w = 1.0;
//		
//				ROS_INFO("     sending goal %d/%d (%f,%f)", (int)goal_sequence.size()-i-1, (int)goal_sequence.size()-1, rxpos, rypos);
//				move_base_ac_.sendGoal(goal);
//				//break;
//				bool arrived = false;
//				bool error = false;
//				do {
//					move_base_ac_.waitForResult(ros::Duration(1.0));
//					if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ arrived = true; continue;}
//					if (move_base_ac_.getState() != actionlib::SimpleClientGoalState::ACTIVE) error = true;		
//				}while (!arrived && !error);

//				
//				if (arrived){
//					ROS_INFO("arrived to goal %d/%d (%f,%f)", (int)goal_sequence.size()-i-1, (int)goal_sequence.size()-1, rxpos, rypos);
//					break;
//				}

//				if (error){
//					ROS_INFO("error to achieve goal %d/%d (%f,%f), replanning...", (int)goal_sequence.size()-i-1, (int)goal_sequence.size()-1, rxpos, rypos);
//					break; // we plan again if there is an error
//				}
//			}
//		}	
	}
}



void explore_tree_planner::evalTree(treeNode& tree, const nav_msgs::OccupancyGrid& map, int& numGateways, int& numLeafs){	
	rvizMarker_.header.frame_id = map.header.frame_id;
	rvizMarker_.header.stamp = map.header.stamp;
	rvizMarker_.action = visualization_msgs::Marker::ADD;
	rvizMarker_.type = visualization_msgs::Marker::ARROW;
	rvizMarker_.scale.x = 0.05f;
	rvizMarker_.scale.y = 0.15f;
	rvizMarker_.scale.z = 0.0f;
	rvizMarker_.ns = "exploration_tree";
	rvizMarker_.lifetime = ros::Duration (1/control_loop_freq_);
	geometry_msgs::Point p;
	evalNode(tree, map);

	if (tree.isLeaf()) numLeafs++;
	else numGateways++;
	
	for (int n = 0; n < tree.getNChildren(); n++){
		evalTree(tree.getChildren(n), map,numGateways, numLeafs);

		rvizMarker_.points.clear();
		rvizMarker_.id++;

		p.x= tree.getX()*map.info.resolution+map.info.origin.position.x; 
		p.y= tree.getY()*map.info.resolution+map.info.origin.position.y;
		rvizMarker_.points.push_back(p);

		p.x= tree.getChildren(n).getX()*map.info.resolution+map.info.origin.position.x; 
		p.y= tree.getChildren(n).getY()*map.info.resolution+map.info.origin.position.y;
		rvizMarker_.points.push_back(p);

		if (tree.getChildren(n).isLeaf()){
			rvizMarker_.color.r = 0.0f;
			rvizMarker_.color.g = 1.0f;
			rvizMarker_.color.b = 0.0f;
			rvizMarker_.color.a = 1.0f;
		}
		else{
			rvizMarker_.color.r = 0.0f;
			rvizMarker_.color.g = 0.0f;
			rvizMarker_.color.b = 1.0f;
			rvizMarker_.color.a = 1.0f;
		}
		rvizMarkerPub_.publish (rvizMarker_);
	}
}

void explore_tree_planner::evalNode(treeNode& node, const nav_msgs::OccupancyGrid& map){
	double value=0;
	if (node.isLeaf()){
		value = pow(node.getSize(),2);
//		for (int i = node.getArea().getRoi().x ; i < node.getArea().getRoi().x + node.getArea().getRoi().width ; i++){
//			for (int j = node.getArea().getRoi().y ; j < node.getArea().getRoi().y + node.getArea().getRoi().height ; j++){   
//				if (node.getArea().get(i,j)){ 										// for each cell in the region associated to the node
//					if ((uchar)map.data[j*map.info.width+i] == 255){				
//						value += 1;
//					}
//				}
//			}
//		}
	}
	node.setAreaValue(value);
//	if (!node.isLeaf())	ROS_INFO("Gateway value %f", value);
//	if (node.isLeaf())	ROS_INFO("Leaf value %f", value);
	if (!node.isRoot()){
		node.setCumulatedAreaValue(node.getParent().getCumulatedAreaValue() + value);
		node.setProfit(node.getCumulatedAreaValue() * exp(-0.025*node.getGlobalCost()) );
	}
	else{
		node.setCumulatedAreaValue(value);
		node.setProfit(0.0);
	}
}





