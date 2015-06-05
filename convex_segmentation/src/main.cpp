#include "convex_segmentation/convex_segmentation.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "convex_segmentation");
	
	convex_segmentation cs;
	
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin();
	
	return 0;
	
}
