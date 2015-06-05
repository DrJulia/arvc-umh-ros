#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"  

tf::TransformListener* tf_listener;

void callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	static int counter = 0;
	tf::StampedTransform map_to_robot;
	try{
		tf_listener->waitForTransform("map", "base_link", scan->header.stamp, ros::Duration(0.5));		
		tf_listener->lookupTransform ("map", "base_link", scan->header.stamp, map_to_robot); 
	}
	catch(tf::TransformException& ex){
//		ROS_ERROR("transform exception!!! %s", ex.what());
		printf("%d, %f, %f, %f;\n", counter, map_to_robot.getOrigin().getX(), map_to_robot.getOrigin().getY(), tf::getYaw(map_to_robot.getRotation()) );
	}

	printf("%d, %f, %f, %f;\n", counter, map_to_robot.getOrigin().getX(), map_to_robot.getOrigin().getY(), tf::getYaw(map_to_robot.getRotation()) );

	counter++;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "ground_truth_log");

  ros::NodeHandle n;

  tf_listener = new tf::TransformListener();

  ros::Subscriber sub = n.subscribe("scan", 1000, callback);

  ros::spin();

  return 0;
}

