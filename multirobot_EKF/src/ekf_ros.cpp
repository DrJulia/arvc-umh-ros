
#include <opencv2/core/core.hpp>
#include <multirobot_EKF/ekf_ros.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>

ekf_ros::ekf_ros():private_nh_("~"), tf_listener_(nh_, ros::Duration(3.0)){

	// read parameters
	ROS_INFO("loading parameters...");
	private_nh_.param<int>("number_of_robots", num_robots_, 1);
	private_nh_.param<int>("nummarks", nummarks_, 500);
	private_nh_.param<double>("maxdist", maxdist_, 5.0);
	private_nh_.param<double>("mahTh", mahTh_, 9);
	private_nh_.param<double>("descTh", descTh_, 0.5);
	private_nh_.param<bool>("matchByDist", matchByDist_, false); matchByDesc_ = !matchByDist_;
	private_nh_.param<double>("alpha1", alpha1_, 0.3);
	private_nh_.param<double>("alpha2", alpha2_, 0.3);
	private_nh_.param<double>("alpha3", alpha3_, 0.3);
	private_nh_.param<double>("alpha4", alpha4_, 0.3);
	private_nh_.param<double>("covini_x", covini_x_, 1.0);
	private_nh_.param<double>("covini_y", covini_y_, 1.0);
	private_nh_.param<double>("covini_th", covini_th_, M_PI/2);
	private_nh_.param<std::string>("ekf_frame", ekf_frame_, "/ekf");
	private_nh_.param<std::string>("robot_frame", robot_frame_, "/base_link");
	private_nh_.param<std::string>("odom_frame", odom_frame_, "/odom");
	private_nh_.param<std::string>("camera_frame", camera_frame_, "/camera");
	
	std::string marker_sources;
	private_nh_.param<std::string>("markers_sources", marker_sources_, std::string(""));
	
	// create and initialize EKF object
	ROS_INFO("creating SLAM object...");
	slam_ = new EKFSLAM(num_robots_, nummarks_, maxdist_, mahTh_, descTh_, matchByDist_, matchByDesc_,
             	   	    alpha1_, alpha2_, alpha3_, alpha4_, covini_x_, covini_y_, covini_th_ );
	lastodom_ = new pose[num_robots_];
	camera_transforms_ = new tf::Transform[num_robots_];

	ROS_INFO("initializing...");
	// intialilize
	std::stringstream ss(marker_sources_);
	std::string source;

	tf_listener_.setUsingDedicatedThread(true);
	for (int r=0; r<num_robots_; r++){
		ss >> source;
		tf::StampedTransform odometry;
		tf::StampedTransform camera_transform;
		bool done=false;
		do{
			try{
				ros::Time now = ros::Time::now();
				ROS_INFO("waiting for transform between %s and %s...", (source+odom_frame_).c_str(), (source+robot_frame_).c_str());				
				// odometry transform traslate points in local robot coordinates to global odometry coordinates
				tf_listener_.waitForTransform (source+odom_frame_, source+robot_frame_, now, ros::Duration(0.5)); 
				ROS_INFO("...");
				tf_listener_.lookupTransform  (source+odom_frame_, source+robot_frame_, now, odometry);
				ROS_INFO("done");

				ROS_INFO("waiting for transform between %s and %s...", (source+robot_frame_).c_str(), (source+camera_frame_).c_str());
				// camera_transform traslates points in camera coordinates to local robot coordinates.
				tf_listener_.waitForTransform (source+robot_frame_, source+camera_frame_, now, ros::Duration(0.5)); 
				ROS_INFO("...");
				tf_listener_.lookupTransform  (source+robot_frame_, source+camera_frame_, now, camera_transform); 
				ROS_INFO("done");
				done = true;
			}
			catch(tf::TransformException&){
				ROS_ERROR("ekf_ros initialization: could not find tranform");
			}
		} while(!done);
		
		lastodom_[r] = odometry;
		camera_transforms_[r]=camera_transform;

		// this does nothing, just to show the values
		ROS_INFO("camera pose: x: %f, y: %f, z: %f", camera_transform.getOrigin().getX(), camera_transform.getOrigin().getY(), camera_transform.getOrigin().getZ());
		double roll,pitch,yaw;
		tf::Matrix3x3(camera_transform.getRotation()).getRPY(roll, pitch, yaw);
		ROS_INFO("camera pose: roll: %f, pitch: %f, yaw: %f",roll, pitch, yaw);
	}

	ROS_INFO("subscribing to %d sources...", num_robots_);
	// subscribe to everything
	marker_sub_ = new message_filters::Subscriber<ar_pose::ARMarkers>[num_robots_];
	marker_sub_filter_ = new tf::MessageFilter<ar_pose::ARMarkers>*[num_robots_];
	ss.clear();	
	ss.seekg (0, std::ios::beg); // rewind
	for (int r=0; r<num_robots_; r++){
		ss >> source;
		std::string str = source+"/ar_pose_marker";
		ROS_INFO("subscribing to topic: %s", str.c_str());
		marker_sub_[r].subscribe(nh_, source + "/ar_pose_marker", 10);
		marker_sub_filter_[r] = new tf::MessageFilter<ar_pose::ARMarkers>(marker_sub_[r], tf_listener_, source + "/odom", 1);
		marker_sub_filter_[r]->registerCallback(boost::bind(&ekf_ros::markers_callback, this, _1, r));
	}

	ROS_INFO("advertising topics...");
	// advert everything
//	map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("markers_map", 1, true);
        map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers_map", 1, true);
	localization_pub_ = new ros::Publisher[num_robots_];
	ss.clear();	
	ss.seekg (0, std::ios::beg); // rewind
	for (int r=0; r<num_robots_; r++){
		ss >> source;
		localization_pub_[r] = nh_.advertise<nav_msgs::Odometry>(source+"/ekf_pose", 1, true);
	}
}

ekf_ros::~ekf_ros(){
	if (localization_pub_) delete[] localization_pub_;
	for (int r=0; r<num_robots_; r++) if (marker_sub_filter_[r]) delete marker_sub_filter_[r];
	if (marker_sub_filter_) delete[] marker_sub_filter_;
	if (marker_sub_) delete[] marker_sub_;
	if (lastodom_) delete[] lastodom_;
	if (camera_transforms_) delete[] camera_transforms_;
}

void ekf_ros::markers_callback(const ar_pose::ARMarkers::ConstPtr& markers, int robotid){
//	ROS_INFO("markers received");
	std::stringstream ss(marker_sources_);
	std::string source;
	tf::StampedTransform* odometry = new tf::StampedTransform[num_robots_];
	for (int r=0; r<num_robots_; r++){
		// get the tf
		ss >> source;
		try{
			// odometry transform traslate points in local robot coordinates to global odometry coordinates
			tf_listener_.waitForTransform (source+odom_frame_, source+robot_frame_, markers->header.stamp, ros::Duration(1)); 
			tf_listener_.lookupTransform  (source+odom_frame_, source+robot_frame_, markers->header.stamp, odometry[r]); 
		}
		catch(tf::TransformException&){
			ROS_ERROR("markers_callback: could not find tranform");
			return;
		}
		pose newodom(odometry[r]);
		// predict
//		ROS_INFO("slam predict...");
		slam_->predict(r, lastodom_[r], newodom);
//		ROS_INFO("done");
		lastodom_[r] = newodom;
	}

	// update
//	ROS_INFO("slam update...");
	slam_->update(robotid, *markers, camera_transforms_[robotid] );
//	ROS_INFO("done");

	// publish the new poses and map
	ss.clear();	
	ss.seekg (0, std::ios::beg); // rewind

	static nav_msgs::Odometry locmsg;
	locmsg.header.stamp 	= markers->header.stamp;
	locmsg.header.frame_id 	= ekf_frame_;
	locmsg.header.seq++;
	tf::Transform transform;

	for (int r=0; r<num_robots_; r++){
		// get the tf
		ss >> source;

		pose robotpose = slam_->getPos(r);
		//robotpose.print("ROBOT IS HERE:> ");
		transform.setOrigin( tf::Vector3(robotpose.x, robotpose.y, 0) );
		transform.setRotation( tf::createQuaternionFromYaw(robotpose.th) );

		tf::poseTFToMsg(transform, locmsg.pose.pose);
	
//		ROS_INFO("publishing pose");
		localization_pub_[r].publish(locmsg);
//		ROS_INFO("publishing tf");
//		tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), markers->header.stamp, source+robot_frame_, ekf_frame_ ));

		tf_broadcaster_.sendTransform(tf::StampedTransform(transform*odometry[r].inverse(), markers->header.stamp,  ekf_frame_, source+odom_frame_  ) );
	}
	delete[] odometry;

//	static sensor_msgs::PointCloud markers_map;
//	markers_map.header.stamp 	= markers->header.stamp;
//	markers_map.header.frame_id 	= ekf_frame_;
//	markers_map.header.seq++;
//	markers_map.points.clear();
//	geometry_msgs::Point32 p;
//	for (int marker=0; marker<slam_->getNumMarks(); marker++){
//		pose4 markerpose = slam_->getMark(marker);
//		p.x = markerpose.x;
//		p.y = markerpose.y;
//		p.z = markerpose.z;
//		markers_map.points.push_back(p);
//	}

	visualization_msgs::MarkerArray markers_map;
	fillMapMessage(markers_map, markers->header.stamp);
//	ROS_INFO("publishing map");
	map_pub_.publish(markers_map);
}

void ekf_ros::fillMapMessage(visualization_msgs::MarkerArray& msg, const ros::Time& stamp){

	static visualization_msgs::Marker marker;

	marker.header.frame_id = ekf_frame_;
	marker.header.stamp = stamp;
	marker.header.seq++;

	for (int r = 0; r< num_robots_; r++){

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "robots_poses";
		marker.id = r;
		
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = visualization_msgs::Marker::SPHERE;
		
		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		pose robotpose = slam_->getPos(r);
		marker.pose.position.x = robotpose.x;
		marker.pose.position.y = robotpose.y;
		marker.pose.position.z = 0.0;
		//marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,robotpose.th);
		
		Ematrix robotcov = slam_->getCovariance(r);
		
		cv::Mat cov = cv::Mat::zeros(3,3,CV_32FC1);
		cov.at<float>(0,0)=robotcov.get(0,0);
		cov.at<float>(0,1)=robotcov.get(0,1);
		cov.at<float>(1,0)=robotcov.get(1,0);
		cov.at<float>(1,1)=robotcov.get(1,1);
		cv::Mat eigenvalues, eigenvectors;
		cv::eigen(cov,eigenvalues,eigenvectors);
		
		//marker.scale.x = 3*sqrt(robotcov.get(0,0));
		//marker.scale.y = 3*sqrt(robotcov.get(1,1));
		marker.scale.x = 3*sqrt(eigenvalues.at<float>(0,0));
		marker.scale.y = 3*sqrt(eigenvalues.at<float>(0,1));
		marker.scale.z = 0.1;
		tf::Matrix3x3 rotmat(eigenvectors.at<float>(0,0),eigenvectors.at<float>(1,0),eigenvectors.at<float>(2,0),
				   eigenvectors.at<float>(0,1),eigenvectors.at<float>(1,1),eigenvectors.at<float>(2,1),
				   eigenvectors.at<float>(0,2),eigenvectors.at<float>(1,2),eigenvectors.at<float>(2,2));
		// get euler angles
		double roll, pitch, yaw;
		rotmat.getEulerYPR(yaw,pitch,roll);
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		msg.markers.push_back(marker);
	}
	

	for (int m = 0; m< slam_->getNumMarks(); m++){

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "markers_map";
		marker.id = m;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = visualization_msgs::Marker::SPHERE;

		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		pose4 markerpose = slam_->getMark(m);
		marker.pose.position.x = markerpose.x;
		marker.pose.position.y = markerpose.y;
		marker.pose.position.z = markerpose.z;
		//marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,markerpose.th);

		Ematrix markercov = slam_->getMarkCovariance(m);
		cv::Mat cov = cv::Mat::zeros(3,3,CV_32FC1);
		cov.at<float>(0,0)=markercov.get(0,0);
		cov.at<float>(0,1)=markercov.get(0,1);
		cov.at<float>(0,2)=markercov.get(0,2);
		cov.at<float>(1,0)=markercov.get(1,0);
		cov.at<float>(1,1)=markercov.get(1,1);
		cov.at<float>(1,2)=markercov.get(1,2);
		cov.at<float>(2,0)=markercov.get(2,0);
		cov.at<float>(2,1)=markercov.get(2,1);
		cov.at<float>(2,2)=markercov.get(2,2);
		cv::Mat eigenvalues, eigenvectors;
		cv::eigen(cov,eigenvalues,eigenvectors);
		marker.scale.x = 3*sqrt(eigenvalues.at<float>(0,0));
		marker.scale.y = 3*sqrt(eigenvalues.at<float>(0,1));
		marker.scale.z = 3*sqrt(eigenvalues.at<float>(0,2));
		tf::Matrix3x3 rotmat(eigenvectors.at<float>(0,0),eigenvectors.at<float>(1,0),eigenvectors.at<float>(2,0),
				   eigenvectors.at<float>(0,1),eigenvectors.at<float>(1,1),eigenvectors.at<float>(2,1),
				   eigenvectors.at<float>(0,2),eigenvectors.at<float>(1,2),eigenvectors.at<float>(2,2));
		// get euler angles
		double roll, pitch, yaw;
		rotmat.getEulerYPR(yaw,pitch,roll);
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		msg.markers.push_back(marker);
	}


}

