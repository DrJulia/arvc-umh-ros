#include "explore_tree_planner/explore_tree_planner.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "explore_tree_planner");
	
	explore_tree_planner etp;
	
	ros::spin();
	
	return 0;
	
}
