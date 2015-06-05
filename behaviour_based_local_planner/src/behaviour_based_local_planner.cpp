//**********************************************************************************
//**                                                                              **
//**   Description:     behaviour_based_local_planner                             **
//**                                                                              **
//**       Project:                                                               **
//**                                                                              **
//**       Created:     14 November 2012                                          **
//**        Author:     Miguel Juliá                                              **
//**       Contact:     mjulia@umh.es                                             **
//**										  **
//**********************************************************************************

#include <behaviour_based_local_planner/behaviour_based_local_planner.h>
#include <laser_geometry/laser_geometry.h>
#include <matFuns.h>
#include <opencv/highgui.h>
#include <nav_msgs/GetMap.h>
#include <btBulletDynamicsCommon.h>

/// constructor
local_planner::local_planner():
	nh_(),
	nh2_(),
	myspinner_(1, &queue_),
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(2.0)),
	scan_filter_(0)
{
	nh2_.setCallbackQueue(&queue_);
	myspinner_.start();

	// reading parameters
	ROS_INFO("Initiating behaviour based reactive controller, reading parameters");
	private_nh_.param<int>("actionradius", actionradius_, 140);
	private_nh_.param<int>("dilationradius", dilationradius_, 7);
	private_nh_.param<string>("robot_frame", robot_frame_, "base_link");
	private_nh_.param<string>("ref_frame", ref_frame_, "odom");
	double meterswidth, metersheight;
	private_nh_.param<double>("width", meterswidth, 17);
	private_nh_.param<double>("height", metersheight, 17);
	private_nh_.param<double>("resolution", resolution_, 0.05);
	width_ = meterswidth / resolution_ + 1;
	height_ = metersheight / resolution_ + 1;
	private_nh_.param<bool>("publish_map", publish_map_, false);
	private_nh_.param<int>("timeHorizon", timeHorizon_, 33);
	private_nh_.param<double>("markingRange", markingRange_, 6);
	private_nh_.param<int>("numMarkingCells", numMarkingCells_, 2);
	private_nh_.param<int>("lzwidth", lzwidth_, 15);
	private_nh_.param<int>("lzheight", lzheight_, 15);
	private_nh_.param<double>("control_loop_freq", control_loop_freq_, 10);
	private_nh_.param<int>("initialState", state_, 0);
	setState(state_);
	private_nh_.param<double>("avoidObstaclesWeight", avoidObstaclesWeight_, 100.0);
	private_nh_.param<double>("goToFrontierWeight", goToFrontierWeight_, 50.0);
	private_nh_.param<double>("goToUnexploredZonesWeight", goToUnexploredZonesWeight_, 5.0);
	private_nh_.param<double>("goToGatewaysWeight", goToUnexploredZonesWeight_, 50.0);
	private_nh_.param<double>("avoidOtherRobotsWeight", avoidOtherRobotsWeight_, 0.0);
	private_nh_.param<double>("goToPrecisePosesWeight", goToPrecisePosesWeight_, 0.0);
	private_nh_.param<double>("goToGoalWeight", goToGoalWeight_, 100.0);
	double avoidObstaclesWidth, goToFrontierWidth, goToUnexploredZonesWidth, goToGatewaysWidth, avoidOtherRobotsWidth, goToPrecisePosesWidth, goToGoalWidth;
	private_nh_.param<double>("avoidObstaclesWidth", avoidObstaclesWidth, 0.12);
	private_nh_.param<double>("goToFrontierWidth", goToFrontierWidth, 7.0);
	private_nh_.param<double>("goToUnexploredZonesWidth", goToUnexploredZonesWidth, 5.0);
	private_nh_.param<double>("goToGatewaysWidth", goToGatewaysWidth, 7.0);
	private_nh_.param<double>("avoidOtherRobotsWidth", avoidOtherRobotsWidth, 2.0);
	private_nh_.param<double>("goToPrecisePosesWidth", goToPrecisePosesWidth, 2.0);
	private_nh_.param<double>("goToGoalWidth", goToGoalWidth, 15.0);
	avoidObstaclesWidth_ = avoidObstaclesWidth/resolution_;
	goToFrontierWidth_ = goToFrontierWidth/resolution_;
	goToUnexploredZonesWidth_ = goToUnexploredZonesWidth/resolution_;
	goToGatewaysWidth_ = goToGatewaysWidth/resolution_;
	avoidOtherRobotsWidth_ = avoidOtherRobotsWidth/resolution_;
	goToPrecisePosesWidth_ = goToPrecisePosesWidth/resolution_;
	goToGoalWidth_ = goToGoalWidth/resolution_;
	private_nh_.param<double>("vmax", vmax_, 0.3);
	private_nh_.param<double>("wmax", wmax_, 0.5);
	private_nh_.param<double>("k1", k1_, 0.95);
	private_nh_.param<double>("k2", k2_, 3.0);

	private_nh_.param<string>("other_robots_frames", other_robots_frames_, "");
	std::stringstream ss(other_robots_frames_);
	std::string framer;
        while(ss >> framer)
		robotr_frame_.push_back(framer);		
	numrobots_ = robotr_frame_.size()+1;

	// advertising map and speed publishers
	ROS_INFO("advertising topics");	
	speed_pub_ = nh_.advertise < geometry_msgs::Twist > ("cmd_vel", 0);
	map_pub_   = nh_.advertise < nav_msgs::OccupancyGrid > ("local_map", 0);
	slam_map_pub_   = nh_.advertise < nav_msgs::OccupancyGrid > ("local_map_slam", 0);
	map_meta_data_pub_ = nh_.advertise<nav_msgs::MapMetaData>("local_map_metadata", 1, true);
	slam_map_meta_data_pub_ = nh_.advertise<nav_msgs::MapMetaData>("local_map_slam_metadata", 1, true);

	//initialize the local occupancygrid
	ROS_INFO("initializing local map");
	// get the robot and laser poses
	tf::StampedTransform robot_to_ref;
	try{
		ros::Time now = ros::Time::now();
		tf_listener_.waitForTransform(ref_frame_, robot_frame_, now, ros::Duration(2.0));		
		tf_listener_.lookupTransform (ref_frame_, robot_frame_, now, robot_to_ref); 
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("transform exception!!! %s", ex.what());
	}
	// configure Local map for navigation
	map_.info.width = width_;
	map_.info.height = height_;
	map_.info.origin.position.x = robot_to_ref.getOrigin().x()-meterswidth/2;
	map_.info.origin.position.y = robot_to_ref.getOrigin().y()-metersheight/2;
	map_.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);;
	map_.info.resolution = resolution_;
	map_.data.resize(width_ * height_);
	map_.header.frame_id=ref_frame_;
	occCount_.resize(width_ * height_);
	totalCount_.resize(width_ * height_);
	clear();
	omap_.initialize(map_);
	// configure Local map for SLAM
	SLAMmap_.info = map_.info;
	SLAMmap_.data.resize(width_ * height_);
	SLAMmap_.header.frame_id=ref_frame_;
	SLAMoccCount_.resize(width_ * height_);
	SLAMtotalCount_.resize(width_ * height_);
	clearSLAMmap();
	SLAMomap_.initialize(map_);
	ROS_INFO("map %d x %d initialized", width_, height_);

	// initialize speed message
	speedCommand_.linear.x = 0;
	speedCommand_.linear.y = 0;
	speedCommand_.linear.z = 0;
	speedCommand_.angular.x = 0;
	speedCommand_.angular.y = 0;
	speedCommand_.angular.z = 0;
	
	// initialize the potential field
	globalField_.initialize(lzwidth_,lzheight_); 
	localMinimaCounter_ = 0;

	// subscribing to topics: laser	
	global_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("dynamic_map");
	
	ROS_INFO("subscribing to topics");
	scan_filter_sub_.subscribe(nh_, "base_scan", 10);
	scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_filter_sub_, tf_listener_, ref_frame_, 1);
	scan_filter_->registerCallback(boost::bind(&local_planner::laser_callback, this, _1));	

	// running the control loop timer
	ROS_INFO("intiating control loop");
	timer_ = nh2_.createTimer(ros::Duration(1/control_loop_freq_), boost::bind(&local_planner::timer_callback, this, _1));

	// initializing services
	ROS_INFO("initializing services");	
	state_server_ = nh_.advertiseService("SetState", &local_planner::state_callback, this);
	gateway_server_ = nh_.advertiseService("SetGateway", &local_planner::gateway_callback, this);

	ROS_INFO("Behaviour based reactive controller is working");
}

// destructor
local_planner::~local_planner(){
	if (scan_filter_)	delete scan_filter_;
}

/// laser callback
// we update the local map when the laser scan is received
void local_planner::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	ROS_INFO("laser received, stamp %f, locking the mutex",scan->header.stamp.toSec());
	
	// get the robot and laser poses
	tf::StampedTransform robot_to_ref;
	tf::StampedTransform laser_to_ref;
	try{
		tf_listener_.waitForTransform(ref_frame_, robot_frame_, scan->header.stamp, ros::Duration(1.0));	
		tf_listener_.lookupTransform (ref_frame_, robot_frame_, scan->header.stamp, robot_to_ref); 
		tf_listener_.lookupTransform (ref_frame_, scan->header.frame_id, scan->header.stamp, laser_to_ref); 
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("transform exception!!! %s", ex.what());
		return;
	}
	
	// clear footprint
	point rcell;
	rcell.x = (int)floor((robot_to_ref.getOrigin().x() - map_.info.origin.position.x)/resolution_);		// cell of the laser in the local map
	rcell.y = (int)floor((robot_to_ref.getOrigin().y() - map_.info.origin.position.y)/resolution_);
	clearFootprint(rcell);
	
	// clearing by raytracing
	ROS_INFO("clearing");
	clearing(*scan, laser_to_ref);

	// marking obstacles
	ROS_INFO("marking");
	marking(*scan, laser_to_ref);
	
	// roll the window if necessary
	//ROS_INFO("roll window");
	int abscellx = (int)floor(robot_to_ref.getOrigin().x()/resolution_);			//cell of the robot in global coordinates
	int abscelly = (int)floor(robot_to_ref.getOrigin().y()/resolution_);
	int currentx = (int)floor(map_.info.origin.position.x/resolution_) +  width_/2;  	//cell of the window center in global coordinates
	int currenty = (int)floor(map_.info.origin.position.y/resolution_) + height_/2;
	//ROS_INFO("To roll: %d, %d => %d, %d", abscellx, abscelly, currentx, currenty);
	//ROS_INFO("map ori: %f, %f width: %d, height: %d ",map_.info.origin.position.x, map_.info.origin.position.y, width_, height_);
	
	bool rolled = false;
	if( abs(abscellx-currentx) >= width_/15 || abs(abscelly-currenty) >= height_/15){
		ROS_INFO("Requesting map");
		nav_msgs::GetMap srv;
		bool res=false;
		ros::Duration delay(0.2);
		//do{
			ROS_INFO("requesting dynamic_map...");
			res = global_map_client_.call(srv); 	
			if (res){
				ROS_INFO("map received %dx%d",srv.response.map.info.width,srv.response.map.info.height);
				map_mutex.lock(); 
				last_global_map_ =  srv.response.map;
				map_mutex.unlock();
		  	}
			else{ 	ROS_ERROR("Failed to call service map");
				delay.sleep();
				ROS_INFO("trying again...");
			}
		//} while (!res); 
		// get the transform from map to odom
		tf::StampedTransform ref_to_map;
		if (last_global_map_.data.size()>0){
			try{
				ros::Time lastsync; std::string errstr;
				tf_listener_.getLatestCommonTime(last_global_map_.header.frame_id, ref_frame_, lastsync, &errstr);		
				tf_listener_.lookupTransform (last_global_map_.header.frame_id, ref_frame_, lastsync, ref_to_map); 
			}
			catch(tf::TransformException& ex){
				ROS_ERROR("transform exception!!! %s", ex.what());
				return;
			}
		}
		map_mutex.lock();
		
		if( abs(abscellx-currentx) >= width_/15){
			rollwindowx(abscellx-currentx, last_global_map_, ref_to_map);
			rolled = true;
		}
		if( abs(abscelly-currenty) >= height_/15) { 
			rollwindowy(abscelly-currenty, last_global_map_, ref_to_map);
			rolled = true;
		}
		map_mutex.unlock();
	}

	// publish local map?
	ROS_INFO("publish map");
	if (publish_map_){
		map_.header.stamp =  scan->header.stamp;
		map_.header.seq++;
		map_meta_data_pub_.publish(map_.info);
		map_pub_.publish(map_);
		SLAMmap_.header.stamp =  scan->header.stamp;
		SLAMmap_.header.seq++;
		slam_map_meta_data_pub_.publish(SLAMmap_.info);
		slam_map_pub_.publish(SLAMmap_);
	}

	//if (rolled) clearSLAMmap();

}

void local_planner::setState(int state){
	state_ = state;

	switch(state){
		case local_planner::REACTIVE_EXPLORATION_STATE:
			gateway_mutex_.lock();
			gateway_.clear();
			gateway_mutex_.unlock();
			avoidObstaclesEnabled_ = true;
			goToFrontierEnabled_ = true;
			goToUnexploredZonesEnabled_ = true;
			goToGatewaysEnabled_ = true;
			avoidOtherRobotsEnabled_ = true;
			goToPrecisePosesEnabled_ = false;
			goToGoalEnabled_ = false;
			break;
		case local_planner::CHANGE_ZONE_STATE:
			avoidObstaclesEnabled_ = true;
			goToFrontierEnabled_ = false;
			goToUnexploredZonesEnabled_ = false;
			goToGatewaysEnabled_ = false ;
			avoidOtherRobotsEnabled_ = false;
			goToPrecisePosesEnabled_ = false;
			goToGoalEnabled_ = true;
			break;
		case local_planner::ACTIVE_LOCALIZATION_STATE:
			gateway_mutex_.lock();
			gateway_.clear();
			gateway_mutex_.unlock();
			avoidObstaclesEnabled_ = true;
			goToFrontierEnabled_ = false;
			goToUnexploredZonesEnabled_ = false;
			goToGatewaysEnabled_ = false ;
			avoidOtherRobotsEnabled_ = false;
			goToPrecisePosesEnabled_ = true;
			goToGoalEnabled_ = false;
			break;
		case 10:
			gateway_mutex_.lock();
			gateway_.clear();
			gateway_mutex_.unlock();
			avoidObstaclesEnabled_ = false;
			goToFrontierEnabled_ = false;
			goToUnexploredZonesEnabled_ = false;
			goToGatewaysEnabled_ = false ;
			avoidOtherRobotsEnabled_ = false;
			goToPrecisePosesEnabled_ = false;
			goToGoalEnabled_ = false;
			break;
	}
}

bool local_planner::gateway_callback(behaviour_based_local_planner::SetGateway::Request& request, behaviour_based_local_planner::SetGateway::Response& response){
	tf::StampedTransform gateway_to_ref;
	try{
		//ros::Time lastsync; std::string errstr;
		//tf_listener_.getLatestCommonTime(ref_frame_, request.header.frame_id, lastsync, &errstr);
		//tf_listener_.lookupTransform (ref_frame_, request.header.frame_id, lastsync, gateway_to_ref); 
		tf_listener_.waitForTransform(ref_frame_, request.header.frame_id, request.header.stamp, ros::Duration(2.0));				
		tf_listener_.lookupTransform (ref_frame_, request.header.frame_id, request.header.stamp, gateway_to_ref); 
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("transform exception!!! %s", ex.what());
		return false;
	}

	map_mutex.lock();
	float posx = map_.info.origin.position.x;
	float posy = map_.info.origin.position.y;
	gateway_mutex_.lock();
	gateway_.clear();
	for (uint i = 0; i < request.gateway.size(); i++){
		// We move them to the local frame of reference (ref_frame_)
		tf::Vector3 gateway_from_ref = gateway_to_ref * tf::Vector3(request.gateway[i].x,
								request.gateway[i].y,
								request.gateway[i].z);
		pointf gate;
		gate.x = (int) floor((gateway_from_ref.getX() - posx) /resolution_);
		gate.y = (int) floor((gateway_from_ref.getY() - posy) /resolution_);
		gateway_.push_back(gate);
	}
	gateway_mutex_.unlock();
	map_mutex.unlock();

	return true;
}

bool local_planner::state_callback(behaviour_based_local_planner::SetState::Request& request, behaviour_based_local_planner::SetState::Response& response){
	setState(request.state);
	return true;
}

void local_planner::clearFootprint(const point& rcell){
	for  (int i = rcell.x - 6; i <= rcell.x + 6; i++){
		for  (int j = rcell.y - 6; j <= rcell.y + 6; j++){
			uint idx = i+map_.info.width*j;
			if (idx >= map_.data.size()) continue;
			if ( totalCount_[idx] < (uint)timeHorizon_) 
				totalCount_[idx] += 1.0f;
			else if (occCount_[idx] > 1) 
				occCount_[idx] -= 1.0f;
			map_.data[idx] = 100.0f* occCount_[idx]/totalCount_[idx];
			if ( SLAMtotalCount_[idx] < (uint)timeHorizon_) 
				SLAMtotalCount_[idx] += 1.0f;
			else if (SLAMoccCount_[idx] > 1) 
				SLAMoccCount_[idx] -= 1.0f;
			SLAMmap_.data[idx] = 100.0f* SLAMoccCount_[idx]/SLAMtotalCount_[idx];
		}
	}
}

void local_planner::clearing(const sensor_msgs::LaserScan& scan, const tf::StampedTransform& laser_pose){
	point rcell; //,pcell;
	uint size;

	rcell.x = (int)floor((laser_pose.getOrigin().x() - map_.info.origin.position.x)/resolution_);		// cell of the laser in the local map
	rcell.y = (int)floor((laser_pose.getOrigin().y() - map_.info.origin.position.y)/resolution_);
	double laseryaw = tf::getYaw(laser_pose.getRotation()); 
	
	ROS_INFO("laser transform %f, %f",laser_pose.getOrigin().x(),laser_pose.getOrigin().y());
	ROS_INFO("processing scan with laser in position (%d, %d, %f)...", rcell.x, rcell.y, laseryaw);

	for(uint s=0; s < scan.ranges.size(); s++){
		// cell of the sensed point
		float th = laseryaw + scan.angle_min + s*scan.angle_increment;
		float range = (markingRange_ < scan.ranges[s] )? markingRange_: scan.ranges[s] ;
		int pcellx = (int)floor(( laser_pose.getOrigin().x() + range*cos(th) - map_.info.origin.position.x)/resolution_);
		int pcelly = (int)floor(( laser_pose.getOrigin().y() + range*sin(th) - map_.info.origin.position.y)/resolution_);
		point* line = getLine(rcell.x,rcell.y,pcellx,pcelly,size);
//		if (size > range/resolution_+2){
//			ROS_ERROR ("ERROR clearing: input range: %f", scan.ranges[s] );
//			ROS_INFO ("rcell: [%d, %d], pcell: [%d, %d], size: %d",rcell.x,rcell.y,pcell.x,pcell.y,size);
//			ROS_INFO ("laser origin: [%f, %f], map origin: [%f, %f]",laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), map_.info.origin.position.x, map_.info.origin.position.y);
//			ROS_INFO("Th: %f, (%f, %f, %f)", th, laseryaw, scan.angle_min, s*scan.angle_increment);
//			ROS_INFO ("pcelly %d",(int)floor(( laser_pose.getOrigin().y() + range*sin(th) - map_.info.origin.position.y)/resolution_) );
//			ROS_INFO ("pcelly not int %f",floor(( laser_pose.getOrigin().y() + range*sin(th) - map_.info.origin.position.y)/resolution_) );
//			ROS_INFO ("pcelly no floor %f", (( laser_pose.getOrigin().y() + range*sin(th) - map_.info.origin.position.y)/resolution_) );
//			ROS_INFO ("pcelly meters %f",  laser_pose.getOrigin().y() + range*sin(th) - map_.info.origin.position.y);
//			ROS_INFO ("range*sinth %f", range*sin(th));
//			ROS_INFO ("sinth %f", sin(th));
//		}

		// clearing
		for (int i = 0; i < (int)size - numMarkingCells_; i++){
			//if (idx >= map_.data.size()) continue;
			if ( line[i].x<0 || line[i].y <0 || line[i].x >= map_.info.width || line[i].y >=map_.info.height) continue;			
			else{
				uint idx = line[i].x+map_.info.width*line[i].y;
				if ( totalCount_[idx] < (uint)timeHorizon_) 
					totalCount_[idx] += 1.0f;
				else if (occCount_[idx] > 1) 
					occCount_[idx] -= 1.0f;
				map_.data[idx] = 100.0f* occCount_[idx]/totalCount_[idx];
				if (scan.ranges[s] < scan.range_max){
					if ( SLAMtotalCount_[idx] < (uint)timeHorizon_) 
						SLAMtotalCount_[idx] += 1.0f;
					else if (SLAMoccCount_[idx] > 1) 
						SLAMoccCount_[idx] -= 1.0f;
					SLAMmap_.data[idx] = 100.0f* SLAMoccCount_[idx]/SLAMtotalCount_[idx];
				}
			}
		}
		delete line;
	}
}
 
void local_planner::marking(const sensor_msgs::LaserScan& scan, const tf::StampedTransform& laser_pose){
	point rcell;//,pcell;
	uint size;
	
	rcell.x = (int)floor((laser_pose.getOrigin().x() - map_.info.origin.position.x)/resolution_);		// cell of the laser in the local map
	rcell.y = (int)floor((laser_pose.getOrigin().y() - map_.info.origin.position.y)/resolution_);
	double laseryaw = tf::getYaw(laser_pose.getRotation()); 

	for(uint s=0; s < scan.ranges.size(); s++){
		// cell of the sensed point
		
		if (scan.ranges[s] < markingRange_){
			float th = laseryaw + scan.angle_min + s*scan.angle_increment;
			int pcellx = (int)floor(( laser_pose.getOrigin().x() + scan.ranges[s]*cos(th) - map_.info.origin.position.x)/resolution_);
			int pcelly = (int)floor(( laser_pose.getOrigin().y() + scan.ranges[s]*sin(th) - map_.info.origin.position.y)/resolution_);
			point* line = getLine(rcell.x,rcell.y,pcellx,pcelly,size);
			
			// clearing
			for (uint i = size-numMarkingCells_ ; i < size; i++){
				//if (idx >= map_.data.size()) continue;
				if ( line[i].x<0 || line[i].y <0 || line[i].x >= map_.info.width || line[i].y >=map_.info.height) continue;			
				else{
					uint idx = line[i].x+map_.info.width*line[i].y; 
					if ( totalCount_[idx] < (uint)timeHorizon_){
						totalCount_[idx] += 1.0f;
						occCount_[idx] += 1.0f;
					}
					else if (occCount_[idx] < (uint)timeHorizon_) {
						occCount_[idx] += 1.0f;
					}
					map_.data[idx] = 100.0f* occCount_[idx]/totalCount_[idx];
					if ( SLAMtotalCount_[idx] < (uint)timeHorizon_){
						SLAMtotalCount_[idx] += 1.0f;
						SLAMoccCount_[idx] += 1.0f;
					}
					else if (SLAMoccCount_[idx] < (uint)timeHorizon_) {
						SLAMoccCount_[idx] += 1.0f;
					}
					SLAMmap_.data[idx] = 100.0f* SLAMoccCount_[idx]/SLAMtotalCount_[idx];
				}
			}
			delete line;
		}
	}
}

void local_planner::clear(){ 
	for(uint i=0; i < map_.data.size(); i++){
		map_.data[i]=-1;
		occCount_[i]=0;
		totalCount_[i]=0;
	}
}

void local_planner::clearSLAMmap(){ 
	for(uint i=0; i < SLAMmap_.data.size(); i++){
		SLAMmap_.data[i]=-1;
		SLAMoccCount_[i]=0;
		SLAMtotalCount_[i]=0;
	}
}

void local_planner::rollwindowx(int diff, const nav_msgs::OccupancyGrid& globalMap, const tf::StampedTransform& ref_to_map){
	ROS_INFO("roll in x direction %d cells", diff);
	if (diff > 0){
		for(uint i=0; i< map_.info.width; i++){
			for(uint j=0; j<map_.info.height ; j++){
				uint targetidx = i+j*map_.info.width;
				uint originidx = i+diff+j*map_.info.width;
				if (i+diff<map_.info.width){
					occCount_[targetidx]=occCount_[originidx];
					totalCount_[targetidx]=totalCount_[originidx];		
					map_.data[targetidx]=map_.data[originidx];
					SLAMoccCount_[targetidx]=SLAMoccCount_[originidx];
					SLAMtotalCount_[targetidx]=SLAMtotalCount_[originidx];		
					SLAMmap_.data[targetidx]=SLAMmap_.data[originidx];
				}
				else{
					tf::Vector3 cellfromref;					
					cellfromref.setX(map_.info.origin.position.x + diff*resolution_ + (i+0.5) * resolution_);
					cellfromref.setY(map_.info.origin.position.y + (j+0.5) * resolution_);
					cellfromref.setZ(0);
					tf::Vector3 cellfrommap = ref_to_map * cellfromref;
					int iglobal = (int) floor((cellfrommap.getX()-globalMap.info.origin.position.x ) / resolution_);
					int jglobal = (int) floor((cellfrommap.getY()-globalMap.info.origin.position.y ) / resolution_);
					uint globalidx = iglobal + globalMap.info.width* jglobal;
					if (iglobal<0 || jglobal<0 || iglobal >= globalMap.info.width || jglobal >= globalMap.info.height){
						occCount_[targetidx]=0;
						totalCount_[targetidx]=0;
						map_.data[targetidx]=-1;
					}
					else{
						int globalValue = globalMap.data[globalidx];
						if (globalValue == -1){
							occCount_[targetidx]=0;
							totalCount_[targetidx]=0;
							map_.data[targetidx]=-1;
						}
						else { //globalValue <= 100
							occCount_[targetidx]=globalValue/100*timeHorizon_;
							totalCount_[targetidx]=timeHorizon_;
							map_.data[targetidx]=globalValue;						
						}
					}
					SLAMoccCount_[targetidx]=0;
					SLAMtotalCount_[targetidx]=0;
					SLAMmap_.data[targetidx]=-1;
				}
			}
		}
	}
	else if(diff < 0){
		for(int i=map_.info.width-1; i>=0; i--){
			for(uint j=0; j<map_.info.height ; j++){
				uint targetidx = i+j*map_.info.width;
				uint originidx = i+diff+j*map_.info.width;
				if (i+diff>0){
					occCount_[targetidx]=occCount_[originidx];
					totalCount_[targetidx]=totalCount_[originidx];		
					map_.data[targetidx]=map_.data[originidx];
					SLAMoccCount_[targetidx]=SLAMoccCount_[originidx];
					SLAMtotalCount_[targetidx]=SLAMtotalCount_[originidx];		
					SLAMmap_.data[targetidx]=SLAMmap_.data[originidx];
				}
				else{
					tf::Vector3 cellfromref;					
					cellfromref.setX(map_.info.origin.position.x + diff*resolution_ + (i+0.5) * resolution_);
					cellfromref.setY(map_.info.origin.position.y + (j+0.5) * resolution_);
					cellfromref.setZ(0);
					tf::Vector3 cellfrommap = ref_to_map * cellfromref;
					int iglobal = (int) floor((cellfrommap.getX()-globalMap.info.origin.position.x ) / resolution_);
					int jglobal = (int) floor((cellfrommap.getY()-globalMap.info.origin.position.y ) / resolution_);
					uint globalidx = iglobal + globalMap.info.width* jglobal;
					if (iglobal<0 || jglobal<0 || iglobal >= globalMap.info.width || jglobal >= globalMap.info.height){
						occCount_[targetidx]=0;
						totalCount_[targetidx]=0;
						map_.data[targetidx]=-1;
					}
					else{
						int globalValue = globalMap.data[globalidx];
						if (globalValue == -1){
							occCount_[targetidx]=0;
							totalCount_[targetidx]=0;
							map_.data[targetidx]=-1;
						}
						else { //globalValue <= 100
							occCount_[targetidx]=globalValue/100*timeHorizon_;
							totalCount_[targetidx]=timeHorizon_;
							map_.data[targetidx]=globalValue;						
						}
					}
					SLAMoccCount_[targetidx]=0;
					SLAMtotalCount_[targetidx]=0;
					SLAMmap_.data[targetidx]=-1;
				}
			}
		}
	}
	else{
		ROS_DEBUG("Requested to roll the window 0 cells. Nothing done");
		return;
	}
	ROS_INFO("cells rolled");
	map_.info.origin.position.x += diff*resolution_;
	omap_.setXOrigin(omap_.getXOrigin() + diff*resolution_);
	SLAMmap_.info.origin.position.x += diff*resolution_;
	SLAMomap_.setXOrigin(SLAMomap_.getXOrigin() + diff*resolution_);
	ROS_INFO("origin updated");
	gateway_mutex_.lock();
	for (uint i = 0; i < gateway_.size(); i++)  gateway_[i].x -= diff;
	gateway_mutex_.unlock();
	ROS_INFO("gateways moved");
	return;

}

void local_planner::rollwindowy(int diff, const nav_msgs::OccupancyGrid& globalMap, const tf::StampedTransform& ref_to_map){
	ROS_INFO("roll in y direction %d cells", diff);
	if (diff > 0){
		for(uint j=0; j<map_.info.height ; j++){
			for(uint i=0; i< map_.info.width; i++){
				uint targetidx = i+j*map_.info.width;
				uint originidx = i+(j+diff)*map_.info.width;
				if (j+diff<map_.info.width){
					occCount_[targetidx]=occCount_[originidx];
					totalCount_[targetidx]=totalCount_[originidx];		
					map_.data[targetidx]=map_.data[originidx];
					SLAMoccCount_[targetidx]=SLAMoccCount_[originidx];
					SLAMtotalCount_[targetidx]=SLAMtotalCount_[originidx];		
					SLAMmap_.data[targetidx]=SLAMmap_.data[originidx];
				}
				else{
					tf::Vector3 cellfromref;					
					cellfromref.setX(map_.info.origin.position.x + (i+0.5) * resolution_);
					cellfromref.setY(map_.info.origin.position.y + diff*resolution_ + (j+0.5) * resolution_);
					cellfromref.setZ(0);
					tf::Vector3 cellfrommap = ref_to_map * cellfromref;
					int iglobal = (int) floor((cellfrommap.getX()-globalMap.info.origin.position.x ) / resolution_);
					int jglobal = (int) floor((cellfrommap.getY()-globalMap.info.origin.position.y ) / resolution_);
					uint globalidx = iglobal + globalMap.info.width* jglobal;
					if (iglobal<0 || jglobal<0 || iglobal >= globalMap.info.width || jglobal >= globalMap.info.height){
						occCount_[targetidx]=0;
						totalCount_[targetidx]=0;
						map_.data[targetidx]=-1;
					}
					else{
						int globalValue = globalMap.data[globalidx];
						if (globalValue == -1){
							occCount_[targetidx]=0;
							totalCount_[targetidx]=0;
							map_.data[targetidx]=-1;
						}
						else { //globalValue <= 100
							occCount_[targetidx]=globalValue/100*timeHorizon_;
							totalCount_[targetidx]=timeHorizon_;
							map_.data[targetidx]=globalValue;						
						}
					}
					SLAMoccCount_[targetidx]=0;
					SLAMtotalCount_[targetidx]=0;
					SLAMmap_.data[targetidx]=-1;
				}
			}
		}
	}
	else if(diff < 0){
		for(int j=map_.info.height-1; j>=0 ; j--){
			for(uint i=0; i< map_.info.width; i++){
				uint targetidx = i+j*map_.info.width;
				uint originidx = i+(j+diff)*map_.info.width;
				if (j+diff>0){
					occCount_[targetidx]=occCount_[originidx];
					totalCount_[targetidx]=totalCount_[originidx];		
					map_.data[targetidx]=map_.data[originidx];
					SLAMoccCount_[targetidx]=SLAMoccCount_[originidx];
					SLAMtotalCount_[targetidx]=SLAMtotalCount_[originidx];		
					SLAMmap_.data[targetidx]=SLAMmap_.data[originidx];
				}
				else{
					tf::Vector3 cellfromref;					
					cellfromref.setX(map_.info.origin.position.x + (i+0.5) * resolution_);
					cellfromref.setY(map_.info.origin.position.y + diff*resolution_ + (j+0.5) * resolution_);
					cellfromref.setZ(0);
					tf::Vector3 cellfrommap = ref_to_map * cellfromref;
					int iglobal = (int) floor((cellfrommap.getX()-globalMap.info.origin.position.x ) / resolution_);
					int jglobal = (int) floor((cellfrommap.getY()-globalMap.info.origin.position.y ) / resolution_);
					uint globalidx = iglobal + globalMap.info.width* jglobal;
					if (iglobal<0 || jglobal<0 || iglobal >= globalMap.info.width || jglobal >= globalMap.info.height){
						occCount_[targetidx]=0;
						totalCount_[targetidx]=0;
						map_.data[targetidx]=-1;
					}
					else{
						int globalValue = globalMap.data[globalidx];
						if(globalValue == -1){
							occCount_[targetidx]=0;
							totalCount_[targetidx]=0;
							map_.data[targetidx]=-1;
						}
						else { //globalValue <= 100
							occCount_[targetidx]=globalValue/100*timeHorizon_;
							totalCount_[targetidx]=timeHorizon_;
							map_.data[targetidx]=globalValue;						
						}
					}
					SLAMoccCount_[targetidx]=0;
					SLAMtotalCount_[targetidx]=0;
					SLAMmap_.data[targetidx]=-1;
				}
			}
		}
	}
	else{
		ROS_DEBUG("Requested to roll the window 0 cells. Nothing done");
		return;
	}
	ROS_INFO("cells rolled");
	map_.info.origin.position.y += diff*resolution_;
	omap_.setYOrigin(omap_.getYOrigin() + diff*resolution_);
	SLAMmap_.info.origin.position.y += diff*resolution_;
	SLAMomap_.setYOrigin(SLAMomap_.getYOrigin() + diff*resolution_);
	ROS_INFO("origin updated");
	gateway_mutex_.lock();
	for (uint i = 0; i < gateway_.size(); i++)  gateway_[i].y -= diff;
	gateway_mutex_.unlock();
	ROS_INFO("gateways moved");
	return;

}


void local_planner::timer_callback(const ros::TimerEvent& t){
	ROS_INFO("control loop timer event");
	// get the robot pose and cell
	ROS_INFO("get the robot transform");
	tf::StampedTransform robot_to_ref;
	ros::Time now = ros::Time::now();
	try{
		tf_listener_.waitForTransform(ref_frame_, robot_frame_, now, ros::Duration(2.0));		
		tf_listener_.lookupTransform (ref_frame_, robot_frame_, now, robot_to_ref);
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("transform exception!!! %s", ex.what());
		return;
	}

	std::vector<point> robotcells(robotr_frame_.size());
	std::vector<tf::StampedTransform> robotr_to_ref(robotr_frame_.size());
	for(uint r=0; r<robotr_frame_.size(); r++){
		try{
			tf_listener_.waitForTransform(ref_frame_, robotr_frame_[r], now, ros::Duration(1.0));		
			tf_listener_.lookupTransform (ref_frame_, robotr_frame_[r], now, robotr_to_ref[r]);
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("transform exception!!! %s", ex.what());
			return;
		}
	}	

	map_mutex.lock();

	for(uint r=0; r<robotr_frame_.size(); r++){
		robotcells[r].x = (int)floor((robotr_to_ref[r].getOrigin().x() - map_.info.origin.position.x)/resolution_);		// cell of the robot in the local map
		robotcells[r].y = (int)floor((robotr_to_ref[r].getOrigin().y() - map_.info.origin.position.y)/resolution_);
	}

	point rcell;
	rcell.x = (int)floor((robot_to_ref.getOrigin().x() - map_.info.origin.position.x)/resolution_);		// cell of the robot in the local map
	rcell.y = (int)floor((robot_to_ref.getOrigin().y() - map_.info.origin.position.y)/resolution_);

	// update local potential field
	//omap_.showMap("omap");
	//cv::waitKey(5);
	ROS_INFO("update the potential field, robot cell: (%d, %d)", rcell.x, rcell.y);
	pose pos;
	updateField(rcell,robotcells);
	pos.x = (robot_to_ref.getOrigin().x() - map_.info.origin.position.x)/resolution_-rcell.x+lzwidth_/2+0.5;
	pos.y = (robot_to_ref.getOrigin().y() - map_.info.origin.position.y)/resolution_-rcell.y+lzheight_/2+0.5;

	map_mutex.unlock();	

	double currentHeading = tf::getYaw(robot_to_ref.getRotation()); 

	// speed controller
	ROS_INFO("speed controller...");
	pointf f = getResponse(globalField_, pos);
	ROS_INFO("direction vector (%f,%f)", f.x,f.y);
	speed vel = regulator(currentHeading,f);
//	if ((f.x*f.x + f.y*f.y) < 1.5){ // local minimum
//		ROS_DEBUG("rotate recovery applied");
//		vel.v = 0;
//		vel.w = wmax_/2.0; // this is a rotation recovery behaviour
//	}
	//globalField_.savePotentialAsImage("/home/mjulia/global.jpg");

	//publish speed command
	speedCommand_.linear.x = vel.v;
	speedCommand_.angular.z = vel.w;
	speed_pub_.publish(speedCommand_);
	ROS_INFO("speed command: %f, %f", vel.v, vel.w);
}


void local_planner::updateField (const point& poscell, const std::vector<point> robotcells){
	
	globalField_.reset();

	binMap esz;
	ROS_INFO("FIND ESZ");
	omap_.secview(poscell.x, poscell.y, esz, dilationradius_,actionradius_);
	ROS_INFO("ESZ DETERMINED");

	localPotentialField avObs(lzwidth_,lzheight_); 
	localPotentialField goFro(lzwidth_,lzheight_); 
	localPotentialField goUZ(lzwidth_,lzheight_); 
	localPotentialField gates(lzwidth_,lzheight_); 
	
	processOMap(poscell, omap_, esz, avObs, goFro, goUZ, gates);
	
	globalField_ += avObs * avoidObstaclesWeight_;
	globalField_ += goFro * goToFrontierWeight_;
	globalField_ += goUZ  * goToUnexploredZonesWeight_;
	globalField_ += gates * goToGatewaysWeight_;
	
	//avObs.savePotentialAsImage("/home/mjulia/avObs.jpg");
	//goFro.savePotentialAsImage("/home/mjulia/goFro.jpg");
	//goUZ.savePotentialAsImage("/home/mjulia/goUZ.jpg");

	
	localPotentialField avRob(lzwidth_,lzheight_); 
	processAvRob(poscell, robotcells, esz, avRob);
	globalField_ += avRob * avoidOtherRobotsWeight_;
	
//	localPotentialField goPre(lzwidth_,lzheight_); 
//	processGoPre(poscell, *ppmap, esz, goPre);
//	global += goPre * goToPrecisePosesWeight_;

	localPotentialField goGoal(lzwidth_,lzheight_);
	processGoGoal(poscell, esz, goGoal);
	globalField_ += goGoal * goToGoalWeight_;	

	//goGoal.savePotentialAsImage("/home/mjulia/goGoal.jpg");
	//globalField_.savePotentialAsImage("/home/mjulia/potential.jpg");

//	delete ppmap;
//	delete ipmap;

}

void local_planner::processOMap(const point& pos, const gridMapInterface& omap, const binMap& esz, localPotentialField& avObs,localPotentialField& goFro,localPotentialField& goUZ,localPotentialField& gates){
	
	if(goToUnexploredZonesEnabled_ || goToFrontierEnabled_ || avoidObstaclesEnabled_ || goToGatewaysEnabled_){
		ROS_INFO ("processing omap roi: %d, %d, %d, %d, omap size %d x %d", esz.getRoi().x, esz.getRoi().y, esz.getRoi().width, esz.getRoi().height, omap.getWidth(), omap.getHeight() );
		uint i,j;
		int ci,cj,x,y;
		int wws = (lzwidth_-1)/2;
		int whs = (lzheight_-1)/2;
		binMap gateways;
		omap.gateways(esz,gateways);
		int ngateways=0;
//		for (i = esz.getRoi().x-1; i <= esz.getRoi().x + esz.getRoi().width ; i++){
//			for (j = esz.getRoi().y-1; j <= esz.getRoi().y + esz.getRoi().height ; j++){	
		bool frontiers = false;		
		for (i = 0; i < esz.getWidth() ; i++){
			for (j = 0; j < esz.getHeight() ; j++){			

				if(omap.isunknown(i,j) && goToUnexploredZonesEnabled_) {					// unexplored
					if (esz.get(i,j)){ // if esz

						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				

								goUZ.set(x,y, goUZ.get(x,y)-gauss(ci,cj, (float)i,(float)j, goToUnexploredZonesWidth_));
							}
						}
					}
				}
				else if(omap.isfrontier(i,j) && goToFrontierEnabled_) {				// frontier
					if (esz.get(i,j)){ // if esz
						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
								goFro.set(x,y, goFro.get(x,y)-gauss(ci,cj,(float) i,(float)j, goToFrontierWidth_));
							}
						}
						frontiers = true;
					}
				}
				else if (omap.isoccupied(i,j) && avoidObstaclesEnabled_) {										// occupied
					if (esz.isOver(i,j, (uint)dilationradius_)){ // if next to esz
						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
								avObs.set(x,y, avObs.get(x,y)+ gauss(ci,cj,(float) i,(float)j, avoidObstaclesWidth_));
							}
						}
					}
				}
				else if (gateways.get(i,j) && goToGatewaysEnabled_ ) {										// gateway
					ngateways++;
					if (!frontiers){ // if next to esz
						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
								gates.set(x,y, gates.get(x,y)- gauss(ci,cj,(float) i,(float)j, goToGatewaysWidth_));
							}
						}
						
					}					
				}
			}
		}
		//ROS_ERROR("Num gateways: %d", ngateways);
		//gates.savePotentialAsImage("/home/mjulia/gates.jpg");
		if (frontiers && goToGatewaysEnabled_){
		 	gates.reset();
		}
		else if (goToGatewaysEnabled_) ROS_ERROR("No frontiers in reactive mode, activating subsumed behaviour GO TO GATEWAYS");
	}

	avObs.normalize();
	//gates.normalize();
	//goFro.normalize();
	//goUZ.normalize();
}

void local_planner::processGoPre(const point& pos, const binMap& ppmap, const binMap&  esz, localPotentialField& goPre){

	if(goToPrecisePosesEnabled_){
//		int i,j,ci,cj,x,y;
//		int wws = (lzwidth_-1)/2;
//		int whs = (lzheight_-1)/2;	

//		for (i = esz.getRoi().x ; i < esz.getRoi().x + esz.getRoi().width-1 ; i++){
//			for (j = esz.getRoi().y ; j < esz.getRoi().y + esz.getRoi().height-1 ; j++){			

//				if(ppmap.get(i,j) && esz.get(i,j)) {	
//					for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
//						for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
//							goPre.set(x,y, goPre.get(x,y)-gauss(ci,cj, (float)i,(float)j, goToPrecisePosesWidth_));
//						}
//					}
//				}
//			}
//		}
	}
	goPre.normalize();
}

void local_planner::processGoGoal(const point& pos, const binMap& esz, localPotentialField& goGoal){
	if(goToGoalEnabled_){
		ROS_INFO ("processing go to goal");

		gateway_mutex_.lock();
		int goalindex = gateway_.size()-1;		
		if (goalindex > 1){ // more than one points
			if(gateway_[goalindex-1].x > 0 && 
			   gateway_[goalindex-1].y > 0 && 
			   gateway_[goalindex-1].x < esz.getWidth() &&
			   gateway_[goalindex-1].y < esz.getHeight() &&
			   esz.get((int)gateway_[goalindex-1].x,(int)gateway_[goalindex-1].y)){ // if the next is visible we remove the current
				ROS_ERROR("POP, NEXT IS VISIBLE");
				gateway_.pop_back();
				goalindex--;
			}
		}
		pointf goal = gateway_[goalindex];

		if  (sqrt(pow(goal.x-pos.x,2)+pow(goal.y-pos.y,2)) < 2){
			ROS_ERROR("POP, ARRIVED");
			gateway_.pop_back();
		}
		gateway_mutex_.unlock();

		if (gateway_.size() <= 1 && state_!=10){ // the last one is the frontier
			ROS_ERROR("LIST EMPTY, GO REACTIVE");
			setState(REACTIVE_EXPLORATION_STATE);
			return;
		}

		uint ci,cj,x,y;
		uint wws = (lzwidth_-1)/2;
		uint whs = (lzheight_-1)/2;
		
		pointf projectedGoal;
		float ang = atan2(goal.y-pos.y,goal.x-pos.x);
		projectedGoal.x = pos.x + (wws*cos(ang));
		projectedGoal.y = pos.y + (wws*sin(ang));
		ROS_INFO("goal: (%f,%f), pose: (%d,%d), ang: %f, pgoal=(%f,%f)", goal.x, goal.y, pos.x, pos.y, ang, projectedGoal.x, projectedGoal.y);
		for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
			for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
				goGoal.set(x,y, goGoal.get(x,y)-gauss(ci,cj,  projectedGoal.x, projectedGoal.y, goToGoalWidth_));
			}
		}
	}
	//goGoal.normalize();
}

void local_planner::processAvRob(const point& poscell, const std::vector<point> robotcells, const binMap& esz, localPotentialField& avRob){

	if(avoidOtherRobotsEnabled_){
		if (robotcells.size()>=1){
		ROS_INFO ("processing avoid robots roi: %d, %d, %d, %d",esz.getRoi().x,esz.getRoi().y,esz.getRoi().width,esz.getRoi().height);
			uint r;
			int ci,cj, x,y; 
			int wws = (lzwidth_-1)/2;
			int whs = (lzheight_-1)/2;
			for (r = 0; r<robotcells.size(); r++){  // for each other robot r2
				if (esz.get(robotcells[r].x, robotcells[r].y)){	
					for (x = 0, ci = poscell.x - wws ; ci <= poscell.x + wws ; ci++, x++){
						for ( y = 0, cj = poscell.y - whs ; cj <= poscell.y + whs ; cj++, y++){   // for each local cell				
							avRob.set(x,y, avRob.get(x,y)+gauss(ci,cj, robotcells[r].x, robotcells[r].y, avoidOtherRobotsWidth_));
						}
					} // local cell j
				}
			} // other robot
		}
	}
	//avRob.normalize();
}


speed local_planner::regulator(float currentHeading, pointf& f){
	speed velo;

	if (f.x!=0 || f.y!=0){
		float ang = (atan2(f.y, f.x) - currentHeading );
		if (ang>=PI) ang += (float)(-2*PI);
		else if (ang<-PI) ang += (float)(2*PI);

		velo.w = k1_*ang;
		if (velo.w>=wmax_) velo.w = wmax_;
		else if (velo.w<-wmax_)	velo.w = -wmax_;

		velo.v = vmax_/(k2_*fabs(ang)+1);
	}
	else{
		velo.v=-0.05;
		velo.w=0.02;
	}
	return velo;
}

// currentPos is the pose of the robot relative to the local potential frame in cells units, but with float precision
pointf local_planner::getResponse(const localPotentialField& lpf, const pose& currentPos){
	
	pointf ctrlAction; // the output

	// find the minimum
	float min=lpf.get(0,0);
	float max=lpf.get(0,0);
	int i,j, mini=0, minj=0;
	for(i=0;i< lpf.getWidth(); i++){
		for(j=0;j< lpf.getHeight(); j++){
			if (min > lpf.get(i,j)){
				min = lpf.get(i,j);
				mini=i;
				minj=j;
			}
			if (max < lpf.get(i,j))	max = lpf.get(i,j);
		}
	}

	// subpixel improvement
	int imin,imax,jmin,jmax;
	imin = (mini-1>=0)? mini-1:0;
	jmin = (minj-1>=0)? minj-1:0;
	imax = (mini+1<lpf.getWidth())? mini+1 : lpf.getWidth()-1;
	jmax = (minj+1<lpf.getHeight())? minj+1 : lpf.getHeight()-1;
	float vali=0;
	float valj=0;
	float sum=0;
	for(i=imin;i<=imax; i++){
		for(j=jmin;j<=jmax; j++){
			float aux = (lpf.get(i,j)-min);
			vali += aux*i;
			valj += aux*j;
			sum += aux;
		}
	}
	// getting the direction
	if (sum==0){
		ctrlAction.x = 0;
		ctrlAction.y = 0;
	}
	else{
		vali /= sum;
		valj /= sum;
		ROS_INFO("val %f,%f, bot %f,%f", vali,valj, currentPos.x,currentPos.y);
		ctrlAction.x = (vali-currentPos.x);
		ctrlAction.y = (valj-currentPos.y);
	}
//	ROS_ERROR("ctrl action %f, %f", ctrlAction.x, ctrlAction.y);

	// detect local minima
	if ((ctrlAction.x*ctrlAction.x + ctrlAction.y*ctrlAction.y) < 1.5){
		//ROS_ERROR("LOCAL MINIMUM!!!");
		localMinimaCounter_++;
		if (localMinimaCounter_>100 && state_!=10){
			ROS_ERROR("LOCAL MINIMUM!!!, switching to reactive navigation");
			setState(REACTIVE_EXPLORATION_STATE);
		}
		if (localMinimaCounter_>250 && state_!=10){
			ROS_ERROR ("Activating recovery Behaviour, Map cleared");
			clear();
		}
	}
	else{
		localMinimaCounter_=0;
	}


	return ctrlAction;
}

