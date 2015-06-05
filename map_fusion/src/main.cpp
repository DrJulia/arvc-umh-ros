#include "map_fusion/map_fusion.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "map_fusion");
	
	map_fusion mf;
	
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin();
	
	return 0;
	
}
