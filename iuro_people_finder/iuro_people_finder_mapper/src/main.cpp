 #include "iuro_people_finder_mapper/people_finder_mapper.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "people_finder_mapper");
	
	people_finder_mapper pfm;
	
	ros::spin();
	
	return 0;
	
}
