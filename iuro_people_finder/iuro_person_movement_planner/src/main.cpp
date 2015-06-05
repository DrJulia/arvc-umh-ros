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

#include <ros/ros.h>
#include <iuro_person_movement_planner/person_movement_planner.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "person_movement_planner");
	person_movement_planner pmp;
	ros::spin();
	return 0;
	
}

