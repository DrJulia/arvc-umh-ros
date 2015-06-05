//
//
//  This class is used to create a graph of submaps of the environment and their fusion in a global map
//
//

#include <graph_optimizer_slam/graph_optimizer_slam.h>
#include <gridmap_description_tools/gridmap_description_tools.h>
#include <sstream>
#include <string>
#include <exception>
#include <opencv/highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;
using namespace std;
using namespace gridmap_description;

// constructor
graph_optimizer_slam::graph_optimizer_slam():
	nh_(),
	nh2_(),
	myspinner_(1, &queue_),
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(1.0)),
	map_initialized_(false)
{
	nh2_.setCallbackQueue(&queue_);
	myspinner_.start();

	// lets wait 5 seconds so we have some data in the first map
	//ros::Duration(5).sleep();

	// read params
	private_nh_.param<int>("num_robots", num_robots_, 1);
	private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
	private_nh_.param<std::string>("map_frame", map_frame_, "/global_map");
	private_nh_.param<double>("keyMapDistanceTh", keyMapDistanceTh_, 0.5);
	private_nh_.param<double>("keyMapTimeTh", keyMapTimeTh_, 20);
	private_nh_.param<double>("overlapingTh", overlapingTh_, 0.7);
	private_nh_.param<bool>("debugging", debugging_, false);

	// Vector of subscriber for the local input maps
	local_maps_sub_.resize(num_robots_);
	for (int r=0; r< num_robots_; r++){
		char maptopicstr[100];
		sprintf(maptopicstr,"robot_%d/local_map_slam",r);
		char robot_frame_str[100];
		sprintf(robot_frame_str,"robot_%d/%s",r,robot_frame_.c_str());

		// register local map subscribers to local map callback
		local_maps_sub_[r] = nh_.subscribe (maptopicstr, 1, &graph_optimizer_slam::local_map_callback, this);
	}
	
	// global map publisher and server
	map_server_ = nh2_.advertiseService("fused_map", &graph_optimizer_slam::map_callback, this );
	map_pub_ = nh2_.advertise<nav_msgs::OccupancyGrid>("fused_map", 3);

	transform_timer_ = nh2_.createTimer(ros::Duration(0.05), boost::bind(&graph_optimizer_slam::tranform_publish_callback, this, _1) );
}
// Callback called with a fixed timer to update the fused map
void graph_optimizer_slam::tranform_publish_callback(const ros::TimerEvent& t){
	// TODO: it should broadcast multiple transforms to each robot odometry
	if(map_initialized_){
		//ROS_ERROR("tranform published");
		tf_broadcaster_.sendTransform(tf::StampedTransform(map_transform_, ros::Time::now(), ref_frame_, map_frame_));
	}
}
// fused map callback, just returns the last fused gridmap
bool graph_optimizer_slam::map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response){
	boost::mutex::scoped_lock(map_mutex_);
	if(map_initialized_){
		response.map = fused_map_;
    		return true;
  	}
  	else  return false;
}


// Callback for new local maps received
void graph_optimizer_slam::local_map_callback(const nav_msgs::OccupancyGrid& lmap){
	
	//ROS_INFO("map received %dx%d", lmap.info.width, lmap.info.height);

	if(map_initialized_){
		// continue if keyframe
		bool iskeyMap = checkForKeyMap(lmap); // this checks the distance to last keymap 
		if (!iskeyMap) {
			// TODO: we should do localization here???
			//ROS_INFO("NO KEYMAP");
			return;
		}
	}
	//ROS_INFO("KEYMAP RECEIVED!!!");

	ref_frame_ = lmap.header.frame_id;


	// to cvmat
	cv::Mat mapasimg, grey_scale_map, feat_map;
	OccupancyGrid2CvMat(lmap, mapasimg,255,100,0);
	OccupancyGrid2CvMat(lmap, feat_map,255,0,255);
	OccupancyGrid2CvMat(lmap, grey_scale_map,127,0,255);
	char node_str[100];
	sprintf(node_str,"/home/mjulia/node_%03d.jpg",graph.getNodes().size());

	// find features and descriptors
	//ROS_INFO("Feature extraction...");
	std::vector<cv::KeyPoint> features;
	cv::Mat descriptors;
	find_features(feat_map, feat_map, features, descriptors, gridmap_description::SURF_DESC, true);
	ROS_INFO("*********** keymap received with %d features ***********", features.size());
	
	if (features.size()<5){
		ROS_INFO("not enough features for key map");
		return;
	}

	cv::Mat img_to_disc,img_dist;

	//cv::imwrite(node_str, grey_scale_map);

	if (debugging_){

	cv::Mat distance1(feat_map.rows, feat_map.cols, CV_32FC1);
	cv::Mat distanceNorm1(feat_map.rows, feat_map.cols, CV_8UC1);
	cv::distanceTransform(feat_map, distance1, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::normalize(distance1, distanceNorm1, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

		// save keypoints images to disc in order to debug,
		img_to_disc = cv::Mat(grey_scale_map.rows, grey_scale_map.cols, CV_8UC3);
		cv::drawKeypoints( grey_scale_map, features,  img_to_disc, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
		img_dist = cv::Mat(grey_scale_map.rows, grey_scale_map.cols, CV_8UC3);
		cv::drawKeypoints( distanceNorm1, features,  img_dist, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
		//cv::imwrite(node_str, img_to_disc);
	}

	// create the new node
	graph_node* node= new graph_node();
	node->set_id(graph.getNodes().size());
	node->set_gridmap(mapasimg);
	node->set_node_stamp(lmap.header.stamp);
	node->set_features(features);
	node->set_descriptors(descriptors);
	if (debugging_)	node->set_disc_map(img_to_disc);
	if (debugging_)	node->set_dist_map(img_dist);
	tf::Transform origin;	
	tf::poseMsgToTF(lmap.info.origin, origin);
	tf::Transform origin_to_center;
	ROS_INFO("Keymap origin in the msg: %f, %f", origin.getOrigin().getX(), origin.getOrigin().getY());
	origin_to_center.setOrigin(tf::Point(lmap.info.width * lmap.info.resolution / 2.0,lmap.info.height * lmap.info.resolution / 2.0, 0 )); 
	origin_to_center.setRotation (tf::createQuaternionFromYaw(0)); 
	node->set_initial_origin(origin*origin_to_center);
	ROS_INFO("Keymap initial pos: %f, %f", node->get_initial_origin().getOrigin().getX(), node->get_initial_origin().getOrigin().getY());
	bool loopfound = false;	
	bool localizing = false;
	if(map_initialized_){
		// Match with last keymap
		std::vector<cv::DMatch> matches;
		// SURF: good distance threshold = 0.2 and 2 neighbours
		// SIFT: good distance threshold = 200 and 3 neighbours
		dataAssociation(graph.get_last_node()->get_descriptors(), descriptors, matches, 300.0, 3);
		ROS_INFO("MATCHES FOUND %d", matches.size());

		// find transform with last keymap
		tf::Transform last_to_new_cells, last_to_new;
		cv::Mat covariance(3,3,CV_64FC1);
		std::vector<cv::DMatch> finalmatches;
		int res = align_maps(graph.get_last_node()->get_features(),features,matches,last_to_new_cells, covariance, 5, finalmatches);
		ROS_INFO("Next Node with %d inliers",res);

//		if (debugging_){
//			// this saves data association images to disc in order to debug,
//			cv::Mat drawImg;
//			drawMatches( graph.get_last_node()->get_disc_map(), graph.get_last_node()->get_features(), img_to_disc, features, matches, drawImg, CV_RGB(255, 0, 255), CV_RGB(255, 0, 0) );
//			char match_img_str[100];
//			sprintf(match_img_str,"/home/mjulia/match_%03d-%03d.jpg",graph.getNodes().size()-1,graph.getNodes().size());
//			cv::imwrite(match_img_str, drawImg);
//			if (res > 0) drawMatches( graph.get_last_node()->get_disc_map(), graph.get_last_node()->get_features(), img_to_disc, features, finalmatches, drawImg, CV_RGB(0, 255, 0), CV_RGB(255, 0, 0) );
//			else  drawMatches( graph.get_last_node()->get_disc_map(), graph.get_last_node()->get_features(), img_to_disc, features, finalmatches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0) );
//			sprintf(match_img_str,"/home/mjulia/match_%03d-%03dFound.jpg",graph.getNodes().size()-1,graph.getNodes().size());
//			cv::imwrite(match_img_str, drawImg);
//		}
		
		if (res>0){
			ROS_INFO("Map alignment returned transformation: x: %fcells, y: %fcells, theta: %frad", last_to_new_cells.getOrigin().getX(), 
														last_to_new_cells.getOrigin().getY(),
														tf::getYaw( last_to_new_cells.getRotation()));
			// scale the transform and covariance with the resolution of the gridmap			
			last_to_new.getOrigin().setX(last_to_new_cells.getOrigin().getX()*lmap.info.resolution);	
			last_to_new.getOrigin().setY(last_to_new_cells.getOrigin().getY()*lmap.info.resolution);	
			last_to_new.getOrigin().setZ(0);
			last_to_new.setRotation(last_to_new_cells.getRotation());

			covariance.at<double>(0,0)*=lmap.info.resolution*lmap.info.resolution;
			covariance.at<double>(0,1)*=lmap.info.resolution*lmap.info.resolution;
			covariance.at<double>(0,2)*=lmap.info.resolution;
			covariance.at<double>(1,1)*=lmap.info.resolution*lmap.info.resolution;
			covariance.at<double>(1,2)*=lmap.info.resolution;
			covariance.at<double>(1,0)=covariance.at<double>(0,1);
			covariance.at<double>(2,0)=covariance.at<double>(0,2);
			covariance.at<double>(2,1)=covariance.at<double>(1,2);

//			ROS_INFO("COV_rec: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//			covariance.at<double>(0,0), covariance.at<double>(0,1),covariance.at<double>(0,2),
//			covariance.at<double>(1,0), covariance.at<double>(1,1),covariance.at<double>(1,2),
//			covariance.at<double>(2,0), covariance.at<double>(2,1),covariance.at<double>(2,2));	

			// apply the traslation from the optimized pose of last node to get the initial pose of new node before optimization
			node->set_optimized_origin( graph.get_last_node()->get_optimized_origin() * last_to_new);

			// create the edge  
			cv::Mat information = 0.05*(covariance.inv());

//			ROS_INFO("INF: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//			information.at<double>(0,0), information.at<double>(0,1),information.at<double>(0,2),
//			information.at<double>(1,0), information.at<double>(1,1),information.at<double>(1,2),
//			information.at<double>(2,0), information.at<double>(2,1),information.at<double>(2,2));

			tf::Transform new_to_last = last_to_new.inverse(); 
			graph_edge edge(new_to_last, information, graph.get_last_node());
			node->add_edge(edge);
		}
		else{
			ROS_ERROR("No transform found");
			// origin is the odom of current key map
			last_to_new= node->get_initial_origin() * graph.get_last_node()->get_initial_origin().inverse();
			node->set_optimized_origin( graph.get_last_node()->get_optimized_origin() * last_to_new);
			tf::Transform new_to_last = last_to_new.inverse(); 
			covariance.at<double>(0,0)=1;
			covariance.at<double>(0,1)=0;
			covariance.at<double>(0,2)=0;
			covariance.at<double>(1,0)=0;
			covariance.at<double>(1,1)=1;
			covariance.at<double>(1,2)=0;
			covariance.at<double>(2,0)=0;
			covariance.at<double>(2,1)=0;
			covariance.at<double>(2,2)=1;
	
			graph_edge edge(new_to_last, covariance, graph.get_last_node());
			node->add_edge(edge);
		}
	
	
		// Look for Loop closures
	
		double a1 = tf::getYaw( node->get_optimized_origin().getRotation());
		double c1x = node->get_optimized_origin().getOrigin().getX() + ( mapasimg.cols*cos(a1)+mapasimg.rows*sin(a1))*lmap.info.resolution;
		double c1y = node->get_optimized_origin().getOrigin().getY() + (-mapasimg.cols*sin(a1)+mapasimg.rows*cos(a1))*lmap.info.resolution;;
		double radius = sqrt(mapasimg.cols*mapasimg.cols+mapasimg.rows*mapasimg.rows)*lmap.info.resolution/2.0; // radius
		bool loopfound = false;	
		int loopperiod = 12;
		int lastloopid = -loopperiod;	
		if (graph.getNodes().size() > loopperiod){
			for (uint m = 0; m<graph.getNodes().size()-loopperiod; m++){

				if (loopfound && (m-lastloopid) < loopperiod) continue;
				//if (loopfound) break;
				// check the percentage of predicted overlaping between maps
				// only check loop closures if is higher than a threshold

				// we are going to approximate the two grid rectangles by two cirles. There is a nice formula for the intersection of two circles with the same radio:
				// http://jwilson.coe.uga.edu/EMAT6680Su12/Carreras/EMAT6690/Essay2/essay2.html
				double a2 = tf::getYaw( graph.getNodes()[m]->get_optimized_origin().getRotation());
				double c2x = graph.getNodes()[m]->get_optimized_origin().getOrigin().getX() + ( mapasimg.cols*cos(a2)+mapasimg.rows*sin(a2))*lmap.info.resolution;
				double c2y = graph.getNodes()[m]->get_optimized_origin().getOrigin().getY() + (-mapasimg.cols*sin(a2)+mapasimg.rows*cos(a2))*lmap.info.resolution;;
				double dist = sqrt(pow(c1x-c2x,2)+pow(c1y-c2y,2)); // distance of the centres of the two circles
				double overlap_area = 2*radius*radius*acos(dist/(2*radius)) - dist/2*sqrt(4*radius*radius-dist*dist);
				double overlap = overlap_area / (M_PI*(radius*radius)); // we get the overlaping percentage
				//ROS_INFO("Checking overlap with node %d (dist %f, rad: %f), overlap is %f percent", m, dist, radius, overlap);			
				if (overlap > overlapingTh_){
					ROS_INFO("Checking loop closure with node %d", m);			

					// Match with
					std::vector<cv::DMatch> matches;
					// SURF: good distance threshold = 0.2 and 2 neighbours
					// SIFT: good distance threshold = 200 and 3 neighbours
					dataAssociation(descriptors, graph.getNodes()[m]->get_descriptors(), matches, 300.0, 3);
					ROS_INFO("MATCHES FOUND %d", matches.size());

					// find transform 
					tf::Transform new_to_past_cells, new_to_past;
					cv::Mat covariance(3,3,CV_64FC1);
					std::vector<cv::DMatch> finalmatches;
					int res = align_maps(features,graph.getNodes()[m]->get_features(),matches,new_to_past_cells, covariance, 6, finalmatches);
					if (res>0) ROS_INFO("Loop found with %d inliers",res);
					if (res>0){ 
						if (debugging_){
							// save data association images to disc in order to debug, 
							cv::Mat drawImg;
							char match_img_str[100];

//							drawMatches( img_to_disc, features, graph.getNodes()[m]->get_disc_map(),graph.getNodes()[m]->get_features(), matches, drawImg, CV_RGB(255, 0, 255), CV_RGB(255, 0, 0) );
//							sprintf(match_img_str,"/home/mjulia/loop_%03d-%03d.jpg",graph.getNodes().size(),m);
//							cv::imwrite(match_img_str, drawImg);

							if (res>0) drawMatches( img_to_disc, features, graph.getNodes()[m]->get_disc_map(),graph.getNodes()[m]->get_features(), finalmatches, drawImg, CV_RGB(255, 0, 255), CV_RGB(255, 0, 0) );		
							else drawMatches( img_to_disc, features, graph.getNodes()[m]->get_disc_map(),graph.getNodes()[m]->get_features(), finalmatches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0) );	
							sprintf(match_img_str,"/home/mjulia/loop_%03d-%03dFound.jpg",graph.getNodes().size(),m);
							cv::imwrite(match_img_str, drawImg);


							if (res>0) drawMatches( img_dist, features, graph.getNodes()[m]->get_dist_map(),graph.getNodes()[m]->get_features(), finalmatches, drawImg, CV_RGB(255, 0, 255), CV_RGB(255, 0, 0) );		
							else drawMatches( img_dist, features, graph.getNodes()[m]->get_dist_map(),graph.getNodes()[m]->get_features(), finalmatches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0) );	
							sprintf(match_img_str,"/home/mjulia/loop_%03d-%03dDist.jpg",graph.getNodes().size(),m);
							cv::imwrite(match_img_str, drawImg);

						}
					}

					if (res>0){
						
						//ROS_INFO("Loop closure found with keymap %d",m);					
						// scale the transform and covariance with the resolution of the gridmap			
						new_to_past.getOrigin().setX(new_to_past_cells.getOrigin().getX()*lmap.info.resolution);	
						new_to_past.getOrigin().setY(new_to_past_cells.getOrigin().getY()*lmap.info.resolution);	
						new_to_past.getOrigin().setZ(0);
						new_to_past.setRotation(new_to_past_cells.getRotation());

						covariance.at<double>(0,0)*=lmap.info.resolution*lmap.info.resolution;
						covariance.at<double>(0,1)*=lmap.info.resolution*lmap.info.resolution;
						covariance.at<double>(0,2)*=lmap.info.resolution;
						covariance.at<double>(1,1)*=lmap.info.resolution*lmap.info.resolution;
						covariance.at<double>(1,2)*=lmap.info.resolution;
						covariance.at<double>(1,0)=covariance.at<double>(0,1);
						covariance.at<double>(2,0)=covariance.at<double>(0,2);
						covariance.at<double>(2,1)=covariance.at<double>(1,2);
	
//						ROS_INFO("COV_rec: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//						covariance.at<double>(0,0), covariance.at<double>(0,1),covariance.at<double>(0,2),
//						covariance.at<double>(1,0), covariance.at<double>(1,1),covariance.at<double>(1,2),
//						covariance.at<double>(2,0), covariance.at<double>(2,1),covariance.at<double>(2,2));
	
						// create the edge 
						cv::Mat information = 0.05*(covariance.inv());

//						ROS_INFO("INF: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//						information.at<double>(0,0), information.at<double>(0,1),information.at<double>(0,2),
//						information.at<double>(1,0), information.at<double>(1,1),information.at<double>(1,2),
//						information.at<double>(2,0), information.at<double>(2,1),information.at<double>(2,2));

						double dist = sqrt(pow(new_to_past.getOrigin().getX(),2)+pow(new_to_past.getOrigin().getY(),2));
//						if (dist<keyMapDistanceTh_){
//							localizing = true;
//							ROS_ERROR("TOO CLOSE TO PAST NODE, NO NEW NODES WILL BE ADDED");
//						}
						if (dist < 10*keyMapDistanceTh_){
							ROS_INFO("Map alignment returned transformation: x: %fcells, y: %fcells, theta: %frad", new_to_past_cells.getOrigin().getX(), 
																	new_to_past_cells.getOrigin().getY(),
																	tf::getYaw( new_to_past_cells.getRotation()));
							graph_edge edge(new_to_past, information, graph.getNodes()[m]);
							node->add_edge(edge);
							loopfound = true;
							lastloopid = m;
						}
					}
					else{
						//ROS_INFO("No loop closure found to node %d", m);
					}

				}
			
			}
		}
	}
	if (!localizing){
		// save data into database 
		graph.add_node(node);
	}
	ROS_INFO("OPTIMIZE GRAPH");
	// TODO: implement g2o or sgd in global_graph.cpp!!!
	if (graph.getNodes().size() > 3) graph.optimize();
	ROS_INFO("FUSE MAPS");
	if (!map_initialized_){
		fused_map_.info.resolution = lmap.info.resolution;
		fused_map_.header.frame_id = map_frame_;  
	}

	ROS_INFO("SET MAP TRANSFORM"); // This should be the distance between new map optimized origin and initial origin
	map_transform_ = node->get_initial_origin() * (node->get_optimized_origin().inverse());	
	global_map_fusion_DT();
	//global_map_fusion();
	

	ROS_INFO("PUBLISH GLOBAL MAP");
	fused_map_.header.stamp = lmap.header.stamp;
	fused_map_.header.seq++;
	map_pub_.publish(fused_map_);

}

bool graph_optimizer_slam::checkForKeyMap(const nav_msgs::OccupancyGrid& newmap){
	// we compare newkeymap and lastkeymap origins
	tf::Transform origin_to_center;
	origin_to_center.setOrigin(tf::Point(newmap.info.width * newmap.info.resolution / 2.0,newmap.info.height * newmap.info.resolution / 2.0, 0 )); 
	origin_to_center.setRotation (tf::createQuaternionFromYaw(0)); 
	tf::Transform lastKeyMapPos = graph.get_last_node()->get_initial_origin()*origin_to_center.inverse();
	float distance =	sqrt(pow(newmap.info.origin.position.x - lastKeyMapPos.getOrigin().getX(), 2) +
	                             pow(newmap.info.origin.position.y - lastKeyMapPos.getOrigin().getY(), 2) );

	// check the distance now
	if (distance > keyMapDistanceTh_)
		return true;
	else{ // we check time stamps
		if (keyMapTimeTh_ >0){ // only if time  threshold is positive
			ros::Duration ellapsed = newmap.header.stamp - graph.get_last_node()->get_node_stamp();
			if (ellapsed.toSec() > keyMapTimeTh_) return true;
			else return false;
		}
		else return false;
	}
}

void graph_optimizer_slam::global_map_fusion(){
	
	// resize and clear the fused map
	float minx, miny, maxx, maxy;
	bool firsttime = true;
	for (uint m = 0; m<graph.getNodes().size(); m++){
		// check the four corners
		// this is the rotation
		float th = tf::getYaw( graph.getNodes()[m]->get_optimized_origin().getRotation());
		float sinth = sin(th);	float costh = cos(th);
		// first corner
		float x1 = graph.getNodes()[m]->get_optimized_origin().getOrigin().getX();
		float y1 = graph.getNodes()[m]->get_optimized_origin().getOrigin().getY();
		if (firsttime) {minx = x1; maxx=x1; miny=y1; maxy=y1; firsttime = false;}
		if (x1 < minx) minx = x1;
		if (x1 > maxx) maxx = x1;
		if (y1 < miny) miny = y1;
		if (y1 > maxy) maxy = y1;
		// second corner
		float x2 = x1 + graph.getNodes()[m]->get_gridmap().cols*costh*fused_map_.info.resolution;
		float y2 = y1 + graph.getNodes()[m]->get_gridmap().cols*sinth*fused_map_.info.resolution;
		if (x2 < minx) minx = x2;
		if (x2 > maxx) maxx = x2;
		if (y2 < miny) miny = y2;
		if (y2 > maxy) maxy = y2;
		// third corner
		float x3 = x1 - graph.getNodes()[m]->get_gridmap().rows*sinth*fused_map_.info.resolution;
		float y3 = y1 + graph.getNodes()[m]->get_gridmap().rows*costh*fused_map_.info.resolution;
		if (x3 < minx) minx = x3;
		if (x3 > maxx) maxx = x3;
		if (y3 < miny) miny = y3;
		if (y3 > maxy) maxy = y3;
		// fourth corner
		float x4 = x3 +graph.getNodes()[m]->get_gridmap().cols*costh*fused_map_.info.resolution;
		float y4 = y3 + graph.getNodes()[m]->get_gridmap().cols*sinth*fused_map_.info.resolution;
		if (x4 < minx) minx = x4;
		if (x4 > maxx) maxx = x4;
		if (y4 < miny) miny = y4;
		if (y4 > maxy) maxy = y4; 
	//	ROS_INFO("map size %d x %d (resolution %f) th: %f", graph.getNodes()[m]->get_gridmap().rows, graph.getNodes()[m]->get_gridmap().cols, fused_map_.info.resolution, th );
	//	ROS_INFO("map corners (%f,%f), (%f,%f), (%f,%f), (%f,%f)", x1,y1, x2,y2, x3,y3, x4,y4);
	}
	fused_map_.info.width  = (ceil(maxx)-floor(minx))/fused_map_.info.resolution+1;
	fused_map_.info.height = (ceil(maxy)-floor(miny))/fused_map_.info.resolution+1;
	//ROS_INFO("fused map X: [%f - %f], Y: [%f - %f]", minx,maxx, minx,maxy);
	//ROS_INFO("fused map size = %d x %d, origin %f, %f", fused_map_.info.width, fused_map_.info.height, minx,miny);
	fused_map_.info.origin.position.x=floor(minx)-(graph.getNodes()[0]->get_gridmap().cols*fused_map_.info.resolution)/2;
	fused_map_.info.origin.position.y=floor(miny)-(graph.getNodes()[0]->get_gridmap().rows*fused_map_.info.resolution)/2;
	fused_map_.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
	fused_map_.data.resize(fused_map_.info.width*fused_map_.info.height);
	for(uint i=0; i< fused_map_.data.size(); i++) fused_map_.data[i] = 255;

	for (uint m = 0; m<graph.getNodes().size(); m++){
		fusemap(graph.getNodes()[m]->get_gridmap(), graph.getNodes()[m]->get_optimized_origin(), fused_map_);
	}

	map_initialized_=true;
}

// fuses a map into another
void graph_optimizer_slam::fusemap(const cv::Mat map, const tf::Transform& map_origin, nav_msgs::OccupancyGrid& outmap ){
	//ROS_INFO("adding keymap to the fusedmap");
	float tx = (map_origin.getOrigin().getX() / outmap.info.resolution);
	float ty = (map_origin.getOrigin().getY() / outmap.info.resolution);
	float th = tf::getYaw(map_origin.getRotation());
	float sinth = sin(th);
	float costh = cos(th);
	float hx = (outmap.info.origin.position.x / outmap.info.resolution);
	float hy = (outmap.info.origin.position.y / outmap.info.resolution);
	float incx = tx - hx;
	float incy = ty - hy;

	for (uint x=0; x < map.cols; x++){
		for (uint y=0; y < map.rows; y++){
			int minind = x+y*map.cols; // input map index

			int jx = floor( x*costh-y*sinth+incx+0.5)-(map.cols)/2 ;
			int jy = floor( x*sinth+y*costh+incy+0.5)-(map.rows)/2 ;

			int moutind = jx+jy*outmap.info.width;		// objective map index

			//ROS_INFO("mapping point %d,%d => %d,%d (%d->%d)", x,y, jx,jy, minind, moutind );			
			// fuse the values
			if (jx>=0 && jx<((int)(outmap.info.width)) && jy>=0 && jy<((int)(outmap.info.height))){				

				if (outmap.data[moutind]==255)
					outmap.data[moutind] = map.data[minind];
				else if (map.data[minind] !=255)
					//outmap.data[moutind] = (outmap.data[moutind] > map.data[minind])? outmap.data[moutind] : map.data[minind] ; // we take the highest occupancy probability
					outmap.data[moutind] = map.data[minind] ; // we take the value of the most recent map
			}
		}
	}
	//ROS_INFO("keymap added to the fusedmap");
}



static const uchar colors[][3] =
{
	{255,0,0},
	{255,128,0},
	{255,255,0},
	{0,255,0},
	{0,128,255},
	{0,255,255},
	{0,0,255},
	{255,0,255},
	{128,0,255},
	{128,255,0},
	{128,128,255},
	{128,255,128},
	{128,0,0},
	{128,255,255},
	{0,255,128},
	{255,0,128},
	{128,128,128},
	{255,128,128},
	{0,0,128},
	{255,255,128}

};


void graph_optimizer_slam::global_map_fusion_DT(){
	ROS_INFO("SIZING THE FUSED MAP");
	// resize and clear the fused map
	float minx, miny, maxx, maxy;
	bool firsttime = true;
	for (uint m = 0; m<graph.getNodes().size(); m++){
		// check the four corners
		// this is the rotation
		float th = tf::getYaw( graph.getNodes()[m]->get_optimized_origin().getRotation());
		float sinth = sin(th);	float costh = cos(th);
		// first corner
		float x1 = graph.getNodes()[m]->get_optimized_origin().getOrigin().getX();
		float y1 = graph.getNodes()[m]->get_optimized_origin().getOrigin().getY();
		if (firsttime) {minx = x1; maxx=x1; miny=y1; maxy=y1; firsttime = false;}
		if (x1 < minx) minx = x1;
		if (x1 > maxx) maxx = x1;
		if (y1 < miny) miny = y1;
		if (y1 > maxy) maxy = y1;
		// second corner
		float x2 = x1 + graph.getNodes()[m]->get_gridmap().cols*costh*fused_map_.info.resolution;
		float y2 = y1 + graph.getNodes()[m]->get_gridmap().cols*sinth*fused_map_.info.resolution;
		if (x2 < minx) minx = x2;
		if (x2 > maxx) maxx = x2;
		if (y2 < miny) miny = y2;
		if (y2 > maxy) maxy = y2;
		// third corner
		float x3 = x1 - graph.getNodes()[m]->get_gridmap().rows*sinth*fused_map_.info.resolution;
		float y3 = y1 + graph.getNodes()[m]->get_gridmap().rows*costh*fused_map_.info.resolution;
		if (x3 < minx) minx = x3;
		if (x3 > maxx) maxx = x3;
		if (y3 < miny) miny = y3;
		if (y3 > maxy) maxy = y3;
		// fourth corner
		float x4 = x3 +graph.getNodes()[m]->get_gridmap().cols*costh*fused_map_.info.resolution;
		float y4 = y3 + graph.getNodes()[m]->get_gridmap().cols*sinth*fused_map_.info.resolution;
		if (x4 < minx) minx = x4;
		if (x4 > maxx) maxx = x4;
		if (y4 < miny) miny = y4;
		if (y4 > maxy) maxy = y4; 
	//	ROS_INFO("map size %d x %d (resolution %f) th: %f", graph.getNodes()[m]->get_gridmap().rows, graph.getNodes()[m]->get_gridmap().cols, fused_map_.info.resolution, th );
	//	ROS_INFO("map corners (%f,%f), (%f,%f), (%f,%f), (%f,%f)", x1,y1, x2,y2, x3,y3, x4,y4);
	}
	ROS_INFO("INITIALIZING THE FUSED MAP");

	fused_map_.info.width  = (ceil(maxx)-floor(minx))/fused_map_.info.resolution+1;
	fused_map_.info.height = (ceil(maxy)-floor(miny))/fused_map_.info.resolution+1;
	//ROS_INFO("fused map X: [%f - %f], Y: [%f - %f]", minx,maxx, minx,maxy);
	//ROS_INFO("fused map size = %d x %d, origin %f, %f", fused_map_.info.width, fused_map_.info.height, minx,miny);
	fused_map_.info.origin.position.x=floor(minx)-(graph.getNodes()[0]->get_gridmap().cols*fused_map_.info.resolution)/2;
	fused_map_.info.origin.position.y=floor(miny)-(graph.getNodes()[0]->get_gridmap().rows*fused_map_.info.resolution)/2;
	fused_map_.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
	fused_map_.data.resize(fused_map_.info.width*fused_map_.info.height);
	for(uint i=0; i< fused_map_.data.size(); i++) fused_map_.data[i] = 255;
	ROS_INFO("INSERTING SEEDS");

	cv::Mat DT,labels;
	DT = cv::Mat(fused_map_.info.height, fused_map_.info.width, CV_8UC1);
	for(uint i=0; i< DT.cols*DT.rows; i++) DT.data[i] = 255;
	labels = cv::Mat(fused_map_.info.height, fused_map_.info.width, CV_32SC1);
	for (uint m = 0; m<graph.getNodes().size(); m++){
		int x = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getX()-fused_map_.info.origin.position.x)/fused_map_.info.resolution;
		int y = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getY()-fused_map_.info.origin.position.y)/fused_map_.info.resolution;
		DT.at<char>(y,x) = 0;
		//ROS_INFO("node %d, in pos (%f, %f) goes to cell (%d, %d)",m,graph.getNodes()[m]->get_optimized_origin().getOrigin().getX(),graph.getNodes()[m]->get_optimized_origin().getOrigin().getY(),x,y);
	}
	ROS_INFO("DISTANCE TRANSFORM"); 

	cv::Mat distance(DT.rows, DT.cols, CV_32FC1);
	cv::Mat distanceNorm(DT.rows, DT.cols, CV_8UC1);
	cv::distanceTransform(DT, distance, labels, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::normalize(distance, distanceNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	ROS_INFO("CLASSIFYING");
	int classmap[graph.getNodes().size()];
	for (uint m = 0; m<graph.getNodes().size(); m++) classmap[m]=0;
	for (uint m = 0; m<graph.getNodes().size(); m++){
		int x = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getX()-fused_map_.info.origin.position.x)/fused_map_.info.resolution;
		int y = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getY()-fused_map_.info.origin.position.y)/fused_map_.info.resolution;
		classmap[labels.at<int>(y,x)] = m;
		//ROS_INFO("node %d in pos (%d,%d) is label %d", m, x,y,labels.at<int>(y,x));
	}
	ROS_INFO("MERGING");
	for (uint i=0; i < fused_map_.info.width; i++){
		for (uint j=0; j < fused_map_.info.height; j++){

			int map_index = classmap[labels.at<int>(j,i)];

			float th = tf::getYaw(graph.getNodes()[map_index]->get_optimized_origin().getRotation());
			float sinth = sin(th);
			float costh = cos(th);
			float tx = (fused_map_.info.origin.position.x + ((float)i)*fused_map_.info.resolution - graph.getNodes()[map_index]->get_optimized_origin().getOrigin().getX() );
			float ty = (fused_map_.info.origin.position.y + ((float)j)*fused_map_.info.resolution - graph.getNodes()[map_index]->get_optimized_origin().getOrigin().getY() );
			int x = floor((tx*costh-ty*sinth)/fused_map_.info.resolution) + (graph.getNodes()[map_index]->get_gridmap().cols)/2 ;
			int y = floor((tx*sinth+ty*costh)/fused_map_.info.resolution) + (graph.getNodes()[map_index]->get_gridmap().rows)/2 ;

			long int from_index = x+y*graph.getNodes()[map_index]->get_gridmap().cols;
			long int to_index = i+j*fused_map_.info.width;

//			ROS_INFO("From (%d,%d) in map %d to cell (%d,%d) in fused map",x,y,map_index,i,j);

			if (x < graph.getNodes()[map_index]->get_gridmap().cols && y < graph.getNodes()[map_index]->get_gridmap().rows && x>=0 && y>=0)
				if (graph.getNodes()[map_index]->get_gridmap().data[from_index] !=255)
					fused_map_.data[to_index] =  graph.getNodes()[map_index]->get_gridmap().data[from_index];
			//else ROS_ERROR("from map %ld (%d,%d)", from_index,x,y,graph.getNodes()[map_index]->get_gridmap().cols);

//			fused_map_.data[to_index] =  labels.at<int>(j,i);
		}
	}

	ROS_INFO("SAVE IMAGES TO DISK");
	//cv::Mat labelsNorm(DT.rows, DT.cols, CV_8UC1);
	//cv::normalize(labels, labelsNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	cv::Mat LabelsColor(DT.rows, DT.cols, CV_8UC3);
	cv::Mat DistColor(DT.rows, DT.cols, CV_8UC3);

	//cv::cvtColor(labelsNorm, LabelsColor, CV_GRAY2RGB);
	cv::cvtColor(distanceNorm, DistColor, CV_GRAY2RGB);

	for (uint i=0; i < labels.rows; i++){
		for (uint j=0; j < labels.cols; j++){
			int idx = labels.at<int>(i,j)%20;
			LabelsColor.at<Vec3b>(i,j)[0] = colors[idx][0];
			LabelsColor.at<Vec3b>(i,j)[1] = colors[idx][1];
			LabelsColor.at<Vec3b>(i,j)[2] = colors[idx][2];
		}
	}
	ROS_INFO("coloured");

	for (uint m = 0; m<graph.getNodes().size(); m++){
		int x = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getX()-fused_map_.info.origin.position.x)/fused_map_.info.resolution;
		int y = (graph.getNodes()[m]->get_optimized_origin().getOrigin().getY()-fused_map_.info.origin.position.y)/fused_map_.info.resolution;
		LabelsColor.at<Vec3b>(y,x) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y+1,x) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y+2,x) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y,x+1) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y,x+2) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y-1,x) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y-2,x) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y,x-1) = cv::Vec3b(0,0,0);
		LabelsColor.at<Vec3b>(y,x-2) = cv::Vec3b(0,0,0);

		DistColor.at<Vec3b>(y,x) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y+1,x) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y+2,x) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y,x+1) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y,x+2) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y-1,x) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y-2,x) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y,x-1) = cv::Vec3b(0,255,255);
		DistColor.at<Vec3b>(y,x-2) = cv::Vec3b(0,255,255);
	}
	char match_img_str[100];
	sprintf(match_img_str,"/home/mjulia/labels%d.jpg",graph.getNodes().size());
	cv::imwrite(match_img_str, LabelsColor);

	sprintf(match_img_str,"/home/mjulia/dist%d.jpg",graph.getNodes().size());
	cv::imwrite(match_img_str, DistColor);

	map_initialized_=true;
	ROS_INFO("we are done");
}



















