#include "behaviour_based_local_planner/behaviour_based_local_planner.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "behaviour_based_local_planner");
	
	local_planner bblp;
	
	//ros::MultiThreadedSpinner spinner(2); // Use 4 threads
	//spinner.spin();

	ros::spin();

	return 0;
	
}
