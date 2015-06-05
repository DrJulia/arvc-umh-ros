#include "graph_optimizer_slam/graph_optimizer_slam.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "graph_optimizer_slam");
	
	graph_optimizer_slam mos;
	
	ros::spin();

//	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
//	spinner.spin();
	
	return 0;
	
}
