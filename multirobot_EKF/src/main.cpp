

#include <multirobot_EKF/ekf_ros.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "multirobot_EKF");
	
	ekf_ros ekf;
	
	ros::spin();
	
	return 0;
	
}
