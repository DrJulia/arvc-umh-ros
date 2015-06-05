#include "iuro_active_people_search/active_people_search.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "active_people_search");
	
	active_people_search aps;
	
	ros::spin();
	
	return 0;
	
}
