//**********************************************************************************
//**                                                                              **
//**   Description:     PEOPLE_FINDER_MAPPER                                      **
//**                                                                              **
//**       Project:     IURO Project                                              **
//**                                                                              **
//**       Created:     27 June 2011                                              **
//**        Author:     Miguel Juliá                                              **
//**       Contact:     mjulia@umh.es                                             **
//**										  **
//**********************************************************************************

#include "iuro_people_finder_mapper/people_finder_mapper.h"
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>

// All this is to run a thread with a high stacksize in order to be able to use the recursive function
// It doesn't seem to be possible to change the stacksize of the ros threads.

typedef struct threadArgs{
	people_finder_mapper* pfm;
	const point* p;
	int numpoints;
	binMap* accessible;
	const nav_msgs::OccupancyGrid* omap;
	float res;
}threadArgs;

void* people_finder_mapper::workingThread(void* arguments){
	//printf("the thread is running\n");
	threadArgs* args = (threadArgs*) arguments;
	//printf("numpoints = %d\n", args->numpoints);
	args->res = args->pfm->countAccessible(args->p, args->numpoints, *(args->accessible), *(args->omap)); 
	return 0;
}

float people_finder_mapper::newThread(const point* p, int numpoints, binMap& accessible, const nav_msgs::OccupancyGrid& omap, people_finder_mapper* pfm){
	threadArgs args;
	args.pfm = pfm;
	args.p = p;
	args.numpoints = numpoints;
	args.accessible = &accessible;
	args.omap = &omap;

	pthread_t thread_id;
	pthread_attr_t tattr;
	int ret;
	sched_param param;
	sigset_t mask, oldmask;
	sigemptyset (&mask);
	sigaddset (&mask, SIGUSR1);
	pthread_sigmask(SIG_BLOCK, &mask, &oldmask);
	ret = pthread_attr_init (&tattr);
	ret = pthread_attr_getschedparam (&tattr, &param);
	ret = pthread_attr_setschedparam (&tattr, &param);
	size_t stacksize = 200000000;
	if (stacksize) pthread_attr_setstacksize (&tattr, stacksize);	
	//printf("creating the thread...\n");
	ret = pthread_create(&thread_id, &tattr, &people_finder_mapper::workingThread, &args);
	//printf("thread created %d\n", ret);
	//printf("wating...\n");
	pthread_join(thread_id, NULL); // wait for the result
	//ros::Duration(3).sleep();
	//printf("thread finished\n");
	return args.res;
}

// ************************************* end

people_finder_mapper::people_finder_mapper(): 
	nh_(), 
	private_nh_("~"), 
	tf_listener_(nh_,ros::Duration(1.0)),
	reset_as_(private_nh_, "reset", boost::bind(&people_finder_mapper::resetCallback, this, _1), false),
	pfmap_(0),
	initialized_(false)
{

	// read parameters
	private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
	private_nh_.param<double>("person_speed", person_speed_, 0.5);
	private_nh_.param<int>("occupied_threshold", occupied_threshold_, 66);
	private_nh_.param<bool>("use_unknown", use_unknown_, true);

	// advertise topics
	map_pub_ 	   = nh_.advertise<iuro_people_finder_mapper::people_finder_map>("people_finder_map", 1, true);
	map_meta_data_pub_ = nh_.advertise<nav_msgs::MapMetaData>("people_finder_map_metadata", 1, true);
	map_as_image_pub_  = nh_.advertise<sensor_msgs::Image>("people_finder_map_as_image", 1, true);
	//cam_info_pub_ 	   = private_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);

	// advertise services
	map_serv_ 	   = nh_.advertiseService("get_people_finder_map", &people_finder_mapper::mapCallback, this);

	// subscribe topics
	//omap_sub_ 	   = nh_.subscribe("map", 1, &people_finder_mapper::omap_callback, this);
	omap_filter_sub_.subscribe(nh_, "map", 1);
	omap_filter_ = new tf::MessageFilter<nav_msgs::OccupancyGrid>(omap_filter_sub_, tf_listener_, robot_frame_, 1);
	omap_filter_->registerCallback(boost::bind(&people_finder_mapper::omap_callback, this, _1));

	//sensed_area_sub_   = nh_.subscribe("sensed_area", 1, &people_finder_mapper::sensed_area_callback, this);
	sensed_area_filter_sub_.subscribe(nh_, "sensed_area", 10);
	sensed_area_filter_ = new tf::MessageFilter<nav_msgs::GridCells>(sensed_area_filter_sub_, tf_listener_, "odom", 1);
	sensed_area_filter_->registerCallback(boost::bind(&people_finder_mapper::sensed_area_callback, this, _1));
	ROS_INFO("people finder mapper created");

	reset_as_.start();
}

people_finder_mapper::~people_finder_mapper(){
	if (pfmap_) delete pfmap_;
}

/*******************************************************   callbacks **********************************************************/

bool people_finder_mapper::mapCallback(iuro_people_finder_mapper::get_people_finder_map::Request &req, iuro_people_finder_mapper::get_people_finder_map::Response &res){
	boost::mutex::scoped_lock(map_mutex_);
	if(initialized_){
		res.map = *pfmap_;
    		return true;
  	}
  	else  return false;
}

void people_finder_mapper::omap_callback(const nav_msgs::OccupancyGrid::ConstPtr& omap){
	// get robot position (this is in ref frame)
	tf::StampedTransform ref_to_robot;
	//ROS_DEBUG("hey, i'm here, size = %d x %d", omap->info.width, omap->info.height);
	ROS_INFO("map received, the time stamp is %f", omap->header.stamp.toSec());
	try{
		tf_listener_.waitForTransform (omap->header.frame_id, robot_frame_, omap->header.stamp, ros::Duration(1)); 
		tf_listener_.lookupTransform  (omap->header.frame_id, robot_frame_, omap->header.stamp, ref_to_robot); 
	}
	catch(tf::TransformException&){
		ROS_ERROR("could not find tranform");
		return;
	}
	//ROS_DEBUG("and i could arrive til here!, great!");

	ROS_DEBUG("ref to robot = (%f,%f)", ref_to_robot.getOrigin().getX(), ref_to_robot.getOrigin().getY());

	// get map frame (this is in ref frame)
	tf::Pose ref_to_map_origin;
	tf::poseMsgToTF (omap->info.origin, ref_to_map_origin);

	ROS_DEBUG("ref to map = (%f,%f)m ", ref_to_map_origin.getOrigin().getX(), ref_to_map_origin.getOrigin().getY());

	// discretize the robot position to cells
	point rpos;
	rpos.x = (int)((ref_to_robot.getOrigin().getX()-ref_to_map_origin.getOrigin().getX())/omap->info.resolution-0.5);
	rpos.y = (int)((ref_to_robot.getOrigin().getY()-ref_to_map_origin.getOrigin().getY())/omap->info.resolution-0.5);

	ROS_DEBUG("map to robot = (%f,%f)",ref_to_robot.getOrigin().getX()-ref_to_map_origin.getOrigin().getX(), ref_to_robot.getOrigin().getY()-ref_to_map_origin.getOrigin().getY());

	ROS_DEBUG("robot cell = (%d,%d)", rpos.x, rpos.y);

	// initialize if necessary
	if (!initialized_){
		initialize(*omap,rpos);
		lastTimeExpanded_= omap->header.stamp;
	}
	else{
		omap_ = *omap;

		// roll the map if the occupancy grid has been rolled
		if (omap->info.origin.position.x != pfmap_->info.origin.position.x || omap->info.origin.position.y != pfmap_->info.origin.position.y){
			int diffx = (omap->info.origin.position.x - pfmap_->info.origin.position.x)/pfmap_->info.resolution;
			int diffy = (omap->info.origin.position.y - pfmap_->info.origin.position.y)/pfmap_->info.resolution;
			if (diffx != 0) rollwindowx(diffx);
			if (diffy != 0) rollwindowy(diffy);
			normalizemap(); // always after a rolling of an expand
		}
		clearOccupiedArea(*omap,rpos);
	}
	//checksum();
	boost::mutex::scoped_lock(map_mutex_);
	// publish the map
	pfmap_->header.stamp = omap->header.stamp;
	map_meta_data_pub_ .publish(pfmap_->info);
	map_pub_.publish(*pfmap_);
	pfmap_->header.seq++;

	// publish the map as image so it can be seen in rviz, since there is no way now to display the pfmap message
	img_.header.seq = pfmap_->header.seq;
	img_.header.stamp = pfmap_->header.stamp;
	img_.header.frame_id = pfmap_->header.frame_id; // this is not correct
	img_.width = pfmap_->info.width;
	img_.height = pfmap_->info.height;
	img_.step = pfmap_->info.width;
	img_.encoding = "mono8";
	uint totalbytes = pfmap_->info.width*pfmap_->info.height;
	img_.data.resize(totalbytes);
	
	// get the max
	double max = 0.0;
	for (uint i = 0; i< totalbytes; i++)
		if (max<pfmap_->data[i]) max= pfmap_->data[i];
	ROS_DEBUG("max value = %f", max);

	// normalize and copy
	for (uint i = 0; i< totalbytes; i++)
		img_.data[i] = (int)floor(255.0f*pfmap_->data[i]/max);
	
	ROS_DEBUG("publish image");
	map_as_image_pub_.publish(img_);

	//ci_.K[0]=1;ci_.K[4]=1;ci_.K[8]=1; ci_.K[2]=img_.width/2;ci_.K[5]=img_.height/2;
	//ci_.P[0]=1;ci_.P[5]=1;ci_.P[10]=1; ci_.P[2]=img_.width/2;ci_.P[6]=img_.height/2;

	//ci_.header.seq = img_.header.seq;
	//ci_.header.stamp = img_.header.stamp;
	//ci_.header.frame_id = img_.header.frame_id;
	//ci_.width = img_.width;
	//ci_.height = img_.height;
	//cam_info_pub_.publish(ci_);
}

void people_finder_mapper::sensed_area_callback(const nav_msgs::GridCells::ConstPtr& sensed_area){
	ROS_INFO("sensed area received");
	if (initialized_){
		offsetx_ = pfmap_->info.origin.position.x/pfmap_->info.resolution;
		offsety_ = pfmap_->info.origin.position.y/pfmap_->info.resolution;
		ROS_DEBUG("offset = (%d,%d)m",offsetx_, offsety_); 
		// insert the sensed_area cells in the binMap
		binMap sensedCells(pfmap_->info.width,pfmap_->info.height);
		for (uint i = 0; i < sensed_area->cells.size(); i++){
			sensedCells.set((int)(sensed_area->cells[i].x/pfmap_->info.resolution-offsetx_), (int)(sensed_area->cells[i].y/pfmap_->info.resolution-offsety_), true);
			//printf("%d,%d\n",(int)(sensed_area->cells[i].x/pfmap_->info.resolution-offsetx_), (int)(sensed_area->cells[i].y/pfmap_->info.resolution-offsety_));
		}
		//sensedCells.showMap("cells");
		clearSensedArea(sensedCells);
		//checksum();

		// expand uncertainty if neccessary
		double expandTimes = (sensed_area->header.stamp - lastTimeExpanded_).toSec()*person_speed_ / pfmap_->info.resolution;
		ROS_INFO("expand times = %f (%f)", expandTimes, (sensed_area->header.stamp - lastTimeExpanded_).toSec());
		if (expandTimes>1){
			for (int i=0; i<(int)floor(expandTimes); i++){
				expandUncertainty(omap_);
				//checksum();
			}
			normalizemap(); // always after a rolling or a expand
			lastTimeExpanded_= sensed_area->header.stamp;
		}
		
		boost::mutex::scoped_lock(map_mutex_);
		// publish the map
		pfmap_->header.stamp = sensed_area->header.stamp;
		map_meta_data_pub_ .publish(pfmap_->info);
		map_pub_.publish(*pfmap_);
		pfmap_->header.seq++;

		// publish the map as image so it can be seen in rviz, since there is no way now to display the pfmap message
		img_.header.seq = pfmap_->header.seq;
		img_.header.stamp = pfmap_->header.stamp;
		img_.header.frame_id = pfmap_->header.frame_id; // this is not correct
		img_.width = pfmap_->info.width;
		img_.height = pfmap_->info.height;
		img_.step = pfmap_->info.width;
		img_.encoding = "mono8";
		uint totalbytes = pfmap_->info.width*pfmap_->info.height;
		img_.data.resize(totalbytes);
	
		// get the max
		double max = 0.0; int maxi = 0;
		//double min = 100.0; int mini = 0;
		for (uint i = 0; i< totalbytes; i++){
			if (max<pfmap_->data[i]){ max= pfmap_->data[i];maxi=i;}
		//	if (min>pfmap_->data[i]){ min= pfmap_->data[i];mini=i;}
		}
		ROS_INFO("max value = %f (i=%d)", max,maxi);
		//ROS_INFO("min value = %f (i=%d)", min,mini);

		// normalize and copy
		for (uint i = 0; i< totalbytes; i++)
			img_.data[i] = (int)floor(255.0f*pfmap_->data[i]/max);
	
		ROS_DEBUG("publish image");
		map_as_image_pub_.publish(img_);

		//cv_bridge::CvImagePtr imgcv = cv_bridge::toCvCopy(img_);
		//cv::imwrite("/home/mjulia/mapimage.png", imgcv->image);
	}
	
}

void people_finder_mapper::resetCallback(const iuro_people_finder_mapper::resetGoalConstPtr& msg){
	
	// get robot position (this is in ref frame)
	tf::StampedTransform ref_to_robot;
	ros::Time now = ros::Time::now();
	try{
		tf_listener_.waitForTransform (omap_.header.frame_id, robot_frame_, now, ros::Duration(1)); 
		tf_listener_.lookupTransform  (omap_.header.frame_id, robot_frame_, now, ref_to_robot); 
	}
	catch(tf::TransformException&){
		ROS_ERROR("could not find tranform");
		return;
	}
	ROS_DEBUG("ref to robot = (%f,%f)", ref_to_robot.getOrigin().getX(), ref_to_robot.getOrigin().getY());

	// get map frame (this is in ref frame)
	tf::Pose ref_to_map_origin;
	tf::poseMsgToTF (omap_.info.origin, ref_to_map_origin);
	ROS_DEBUG("ref to map = (%f,%f)m ", ref_to_map_origin.getOrigin().getX(), ref_to_map_origin.getOrigin().getY());

	// discretize the robot position to cells
	point rpos;
	rpos.x = (int)((ref_to_robot.getOrigin().getX()-ref_to_map_origin.getOrigin().getX())/omap_.info.resolution-0.5);
	rpos.y = (int)((ref_to_robot.getOrigin().getY()-ref_to_map_origin.getOrigin().getY())/omap_.info.resolution-0.5);
	ROS_DEBUG("robot cell = (%d,%d)", rpos.x, rpos.y);

	// initialize if necessary
	initialize(omap_,rpos);

	iuro_people_finder_mapper::resetResult result;
	result.done=true;
	reset_as_.setSucceeded(result);

}


/*****************************************************************************************************************************/

void people_finder_mapper::initialize(const nav_msgs::OccupancyGrid& omap, const point& rpos){
	ROS_DEBUG("Initializing people_finder_mapper...\n");
	boost::mutex::scoped_lock(map_mutex_);
	//map_mutex_.lock();
	
	omap_ = omap;

	// prepare the map
	if (pfmap_) delete pfmap_; 
	pfmap_ = new iuro_people_finder_mapper::people_finder_map();
	pfmap_->info.resolution = omap.info.resolution;
	pfmap_->info.width = omap.info.width;
	pfmap_->info.height = omap.info.height;
	pfmap_->data.resize(omap.info.width*omap.info.height);
	pfmap_->info.origin = omap.info.origin;
	pfmap_->header.seq = 0;
	pfmap_->header.frame_id = omap.header.frame_id;	
	
	ROS_DEBUG("[TARGETPROBMAPPER] looking for accesible cells...");
	double nac;
	binMap accessible;
	int numpoints = 1;
	nac = newThread(&rpos,numpoints,accessible,omap,this);	
	ROS_DEBUG("[TARGETPROBMAPPER] accessible cells in the map: %f\n", nac);
	
	double val = 1/nac; // initial probability
	def_prob_ = val;
	ROS_DEBUG("[TARGETPROBMAPPER] initializing all cells to: %e\n",val);
	for (uint i = 0; i< pfmap_->info.width; i++)
		for(uint j=0; j < pfmap_->info.height; j++)
			if (accessible.get(i,j)) pfmap_->data[j*pfmap_->info.width+i] = val;
			else pfmap_->data[j*pfmap_->info.width+i] = 0.0f;

	//map_mutex_.unlock();
	//int expandTimes = pfmap_->info.width;
	//for (int i=0; i<expandTimes; i++){
	//	expandUncertainty(omap_);
	//}
	//normalizemap(); // always after a rolling or a expand

	ROS_DEBUG("[TARGETPROBMAPPER] people_finder_mapper Initialized\n");
	initialized_ = true;
}

void people_finder_mapper::clearOccupiedArea(const nav_msgs::OccupancyGrid& omap, const point& rpos){
	boost::mutex::scoped_lock(map_mutex_);
	
	// get the non accessible area
	binMap area;
	int numpoints = 1;
	newThread(&rpos, numpoints, area, omap,this);
	area.invert();
	
	// sum the probability of the non accesible area
	double X = countProb(area);
	double div = (1-X);
	def_prob_/=div;

	// put each cell in the non accesible area to 0
	// and for each cell in the accesible area reevaluate its probability
	for (uint i = 0; i< pfmap_->info.width; i++){
		for(uint j=0; j < pfmap_->info.height; j++){
			if(area.get(i,j))	pfmap_->data[j*pfmap_->info.width+i]  = 0.0f;
			else			pfmap_->data[j*pfmap_->info.width+i] /= div;
		}
	}
}

// modelo del sensor, lo obtengo con una esz 
void people_finder_mapper::clearSensedArea(const binMap& area){
	boost::mutex::scoped_lock(map_mutex_);
	ROS_DEBUG("sensed area received");
	double X = countProb(area);
	double div = (1-X);
	def_prob_/=div;
	
	// put each cell in the sensed area to 0
	// and for each cell not in the sensed area reevaluate its probability
	for (uint i = 0; i< pfmap_->info.width; i++){
		for(uint j=0; j < pfmap_->info.height; j++){
			if(area.get(i,j))	pfmap_->data[j*pfmap_->info.width+i]  = 0.0f;
			else			pfmap_->data[j*pfmap_->info.width+i] /= div;
		}
	}
}

void people_finder_mapper::expandUncertainty(const nav_msgs::OccupancyGrid& omap){
	boost::mutex::scoped_lock(map_mutex_);
	
	// define destination map 
	iuro_people_finder_mapper::people_finder_map* oldmap = pfmap_;
	pfmap_ = new iuro_people_finder_mapper::people_finder_map();
	pfmap_->header = oldmap->header;
	pfmap_->info = oldmap->info;
	pfmap_->data.resize(oldmap->info.width*oldmap->info.height);
	for (uint i = 0; i< pfmap_->data.size(); i++) pfmap_->data[i] = 0.0;
	//double iniprob = 1/oldmap->info.width*oldmap->info.height;

	// For each cell
	for (int i = -1; i<= (int)pfmap_->info.width; i++){
		for(int j=-1; j <= (int)pfmap_->info.height; j++){
			double val=0.0;
			int N=0; // N = Count the number of accesible neighbour cells 

			if( i<0 || i>= (int)pfmap_->info.width || j<0 || j >= (int)pfmap_->info.height){ // cell is outside the window
				for (int x = i-1; x <= i+1; x++){ // check the 8 neighbours n...
					for (int y = j-1; y <= j+1; y++){
						unsigned int ind = y*pfmap_->info.width+x;
						if( x<0 || x>= (int)pfmap_->info.width || y<0 || y >= (int)pfmap_->info.height) N++; // cells out of the window always count
						else {
							if (omap.data[ind] < occupied_threshold_ && (omap.data[ind]!=-1 || use_unknown_)) N++;
						}
					}
				}
				val = def_prob_;
			}
			else{ // cell is inside the window
				int ind = j*pfmap_->info.width+i;
				if (omap.data[ind] > occupied_threshold_) continue;
				for (int x = i-1; x <= i+1; x++){ // check the 8 neighbours n...
					for (int y = j-1; y <= j+1; y++){
						int ind2 = y*pfmap_->info.width+x;
						if( x<0 || x>= (int)pfmap_->info.width || y<0 || y >= (int)pfmap_->info.height) N++; // cells out of the window always count
						else {
							if (omap.data[ind2] < occupied_threshold_ && (omap.data[ind2]!=-1 || use_unknown_)) N++;
						}
					}
				}
				val = oldmap->data[ind];
			}
			
			double inc = 0;
			if (N>0) inc = val/N; // it moves to other cells or stay with probability...
			
			// expand the values in the new map
			for (int x = i-1; x <= i+1; x++){
				if( x<0 || x>= (int)pfmap_->info.width) continue; // don't update out of the window
				for (int y = j-1; y <= j+1; y++){
					if(y<0 || y >= (int)pfmap_->info.height) continue; // don't update out of the window
					int ind2 = y*pfmap_->info.width+x;
					if (omap.data[ind2] < occupied_threshold_ && (omap.data[ind2]!=-1 || use_unknown_)) pfmap_->data[ind2] += inc;
				}
			}
		}
	}
	delete oldmap;
}

float people_finder_mapper::countProb(const binMap& area) const{
	// sum P(c) para cada c en area
	double sum=0;
	for (int i = area.getRoi().x ; i < area.getRoi().x + area.getRoi().width ; i++)
		for (int j = area.getRoi().y ; j < area.getRoi().y + area.getRoi().height ; j++)   // for each cell
			if(area.get(i,j))
				sum+= pfmap_->data[j*pfmap_->info.width+i];
	return sum;
}

float people_finder_mapper::countAccessible(const point* p, int numpoints, binMap& accessible, const nav_msgs::OccupancyGrid& omap) const{
	// N = some floodfill algorithm counting
	ROS_DEBUG("[countAccessible] npoints :%d, map size (%d, %d)\n", numpoints, pfmap_->info.width, pfmap_->info.height);
	accessible.initialize(pfmap_->info.width, pfmap_->info.height);
	float N=0;
	for (int i=0; i< numpoints; i++){
		//ROS_DEBUG("[countAccessible] %d\n",i);
		accessible.set(p[i].x,p[i].y,true);
		N+=1;
		N+= floodfill(p[i].x,p[i].y,accessible, omap);
	}
	ROS_DEBUG("[countAccessible] result %f\n",N);
	return N;
}

float people_finder_mapper::floodfill(const int& nx, const int& ny, binMap& processed, const nav_msgs::OccupancyGrid& omap) const{
	float N=0;
//	ROS_DEBUG("[floodfill]  para el punto (%d, %d), acumulado: %f\n", nx,ny,N);
//	ROS_DEBUG("[floodfill]  size (%d x %d)\n", processed.getWidth(), processed.getHeight());
	for (int x = nx-1; x <= nx+1; x++){
//		ROS_DEBUG("[floodfill]  x=%d\n",x);
		if(x < 0 ||  x>= (int)pfmap_->info.width) continue;
		for (int y = ny-1; y <= ny+1; y++){
//			ROS_DEBUG("[floodfill]  y=%d\n",y);
			if(y < 0 || y >= (int)pfmap_->info.height) continue;
//			ROS_DEBUG("[floodfill]  considering point (%d, %d) processed= %d, is occupied= %d\n", x,y, processed.get(x,y), (omap.data[y*omap.info.width+x]> occupied_threshold_));
			if(!processed.get(x,y)){
				unsigned int ind= y*omap.info.width+x;
				if( ((omap.data[ind]<occupied_threshold_)) && (omap.data[ind]!=-1 || use_unknown_) ){
					processed.set(x,y,true);
//					omap.showMap("proc");
//					ROS_DEBUG("[floodfill]  iteration ... point (%d, %d) => (%d, %d)\n", nx, ny, x, y);	
//					ros::Duration(0.1).sleep();
					N+=1;
//					printf("N=%f\n",N);
//					ROS_DEBUG("[floodfill]  N=%d\n",N);
					N += floodfill(x, y, processed,omap);
				}
			}
		}
	}
	return N;
}

void people_finder_mapper::rollwindowx(int diff){
	double sum=0.0;
	for(uint i=0; i< pfmap_->info.width; i++)
		for(uint j=0; j < pfmap_->info.height; j++)
			if (i+diff<pfmap_->info.width && (i+diff>0) ) // common part
				sum += pfmap_->data[j*pfmap_->info.width+i];
	// sum is P(M2|M1)
	// P(M1|M2)
	//double sum2 = sum / (sum+def_prob_*diff*pfmap_->info.height);
	double coeff = 1;//sum/sum2;
	//printf("x: diff = %d, sum = %f, sum2 = %f, coeff  = %f, def_prob_ = %f\n",diff, sum, sum2, coeff, def_prob_);
	//def_prob_ *= coeff; //1.0/(pfmap_->info.height*pfmap_->info.width);

	if (diff > 0){
		for(uint i=0; i< pfmap_->info.width; i++){
			for(uint j=0; j<pfmap_->info.height ; j++){
				uint targetidx = i+j*pfmap_->info.width;
				uint originidx = i+diff+j*pfmap_->info.width;
				if (i+diff<pfmap_->info.width){		
					pfmap_->data[targetidx]=pfmap_->data[originidx] * coeff;
				}
				else{
					pfmap_->data[targetidx]=def_prob_;
				}
			}
		}
	}
	else if(diff < 0){
		for(int i=pfmap_->info.width-1; i>=0; i--){
			for(uint j=0; j<pfmap_->info.height ; j++){
				uint targetidx = i+j*pfmap_->info.width;
				uint originidx = i+diff+j*pfmap_->info.width;
				if (i+diff>0){	
					pfmap_->data[targetidx]=pfmap_->data[originidx] * coeff;
				}
				else{
					pfmap_->data[targetidx]=def_prob_;
				}
			}
		}
	}
	else{
		ROS_DEBUG("Requested to roll the window 0 cells. Nothing done");
		return;
	}
	pfmap_->info.origin.position.x += diff*pfmap_->info.resolution;
	return;

}

void people_finder_mapper::rollwindowy(int diff){

	double sum=0.0;
	for(uint i=0; i< pfmap_->info.width; i++)
		for(uint j=0; j < pfmap_->info.height; j++)
			if (j+diff<pfmap_->info.height && (j+diff>0) ) // common part
				sum += pfmap_->data[j*pfmap_->info.width+i];
	// sum is P(M2|M1)
	// P(M1|M2)
	//double sum2 = sum / (sum+def_prob_*diff*pfmap_->info.width); 
	double coeff = 1;//sum/sum2;
	//printf("y: diff = %d, sum = %f, sum2 = %f, coeff  = %f, def_prob_ = %f\n",diff, sum, sum2, coeff, def_prob_);

	//def_prob_ *= coeff;  //1.0/(pfmap_->info.height*pfmap_->info.width);

	if (diff > 0){
		for(uint j=0; j<pfmap_->info.height ; j++){
			for(uint i=0; i< pfmap_->info.width; i++){
				uint targetidx = i+j*pfmap_->info.width;
				uint originidx = i+(j+diff)*pfmap_->info.width;
				if (j+diff<pfmap_->info.height){	
					pfmap_->data[targetidx]=pfmap_->data[originidx] * coeff;
				}
				else{
					pfmap_->data[targetidx]=def_prob_; 
				}
			}
		}
	}
	else if(diff < 0){
		for(int j=pfmap_->info.height-1; j>=0 ; j--){
			for(uint i=0; i< pfmap_->info.width; i++){
				uint targetidx = i+j*pfmap_->info.width;
				uint originidx = i+(j+diff)*pfmap_->info.width;
				if (j+diff>0){
					pfmap_->data[targetidx]=pfmap_->data[originidx] * coeff;
				}
				else{
					pfmap_->data[targetidx]=def_prob_;
				}
			}
		}
	}
	else{
		ROS_DEBUG("Requested to roll the window 0 cells. Nothing done");
		return;
	}
	pfmap_->info.origin.position.y += diff*pfmap_->info.resolution;
	return;
}

void people_finder_mapper::normalizemap(){
	boost::mutex::scoped_lock(map_mutex_);
	double sum=0.0;
	for(uint j=0; j < pfmap_->info.height; j++)
		for(uint i=0; i< pfmap_->info.width; i++)
			sum += pfmap_->data[j*pfmap_->info.width+i];

	for(uint j=0; j < pfmap_->info.height; j++)
		for(uint i=0; i< pfmap_->info.width; i++)
			 pfmap_->data[j*pfmap_->info.width+i]  /= sum;

	def_prob_/=sum;
}

void people_finder_mapper::checksum(){
	boost::mutex::scoped_lock(map_mutex_);
	double sum=0.0;
	for(uint i=0; i< pfmap_->info.width; i++)
		for(uint j=0; j < pfmap_->info.height; j++)
				sum += pfmap_->data[j*pfmap_->info.width+i];
	ROS_DEBUG("sum = %e\n",sum);
}

