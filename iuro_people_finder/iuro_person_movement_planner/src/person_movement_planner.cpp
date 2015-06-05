//**********************************************************************************
//**                                                                              **
//**   Description:     person_movement_planner                                   **
//**                                                                              **
//**       Project:     IURO Project                                              **
//**                                                                              **
//**       Created:     October 2011                                              **
//**        Author:     Miguel Juliá                                              **
//**       Contact:     mjulia@umh.es                                             **
//**										  **
//**********************************************************************************

#include <iuro_person_movement_planner/person_movement_planner.h>
#include <nav_msgs/GetMap.h>
#include <boost/thread.hpp>


person_movement_planner::person_movement_planner():
	nh_(),
	move_base_client_("move_base", true)
{

	// get map client
	map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");

	// asking for the map
	nav_msgs::GetMap srv;
	bool res=false;
	ros::Duration delay(1.0);
	do{
		ROS_INFO("requesting static_map...");
		res = map_client_.call(srv);
		if (res){
			ROS_INFO("map received");
			map_ = srv.response.map;
	  	}
		else{ 	ROS_ERROR("Failed to call service static_map");
			delay.sleep();
			ROS_INFO("trying again...");
		}
	} while (!res); 

	// setting up move_base action client
	ROS_INFO("waiting for move_base to start...");
  	move_base_client_.waitForServer();
	ROS_INFO("move_base client running");



	// running the goal selection thread
	ROS_INFO("running the person movement planner thread");
	boost::thread workerThread( &person_movement_planner::planning_thread, this);

}


void person_movement_planner::planning_thread(){

	ROS_INFO("person movement planner thread is running");

	srand ( ros::Time::now().nsec);

	while(ros::ok()){

		// choosing a random cell
		int rcellx = rand() % map_.info.width;
		int rcelly = rand() % map_.info.height;

		// check if it is free
		if (check_goal(rcellx, rcelly, 6)) continue;
		
		// transform to global coordinates
		double ang = tf::getYaw(map_.info.origin.orientation);
		double rxpos = (rcellx+0.5)*map_.info.resolution*cos(ang) - (rcelly+0.5)*map_.info.resolution*sin(ang) + map_.info.origin.position.x; // TODO: check the signs, not sure
		double rypos = (rcellx+0.5)*map_.info.resolution*sin(ang) + (rcelly+0.5)*map_.info.resolution*cos(ang) + map_.info.origin.position.y;

		ROS_INFO("new goal = [%d, %d] (%f, %f) --> %d ", rcellx, rcelly, rxpos, rypos, map_.data[rcellx+map_.info.width*rcelly]);
		
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.pose.position.x = rxpos;
		goal.target_pose.pose.position.y = rypos;
		goal.target_pose.pose.orientation.w = 1.0;

		move_base_client_.sendGoal(goal);

		bool arrived = false;
		bool error = false;
		do {
			move_base_client_.waitForResult(ros::Duration(5.0));
			if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ arrived = true; continue;}
			if (move_base_client_.getState() != actionlib::SimpleClientGoalState::ACTIVE) error = true;		
		}while (!arrived && !error);
		  		
	}

}

bool person_movement_planner::check_goal(int x, int y, int r){
	if (x < r || x > map_.info.width - r || y < r || y > map_.info.width - r) return true;
	bool itsok = true;
	for (int i = x - r; i < x + r; i++){
		for (int j = y - r; j < y + r; j++){
			if (map_.data[i+map_.info.width*j] > 50 || map_.data[i+map_.info.width*j] < 0){
				itsok = false;
				break;
			}
		}
	}
	return (!itsok);
}


