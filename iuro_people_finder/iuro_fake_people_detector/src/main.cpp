//*  fake_people_detector                                  
/**                                                        
*   Author: Miguel Julia <mjulia@umh.es>                                                                                                               
*                                                          
*   Description: This class is an amazing fake, basically  
*                it looks for a set of tf frame names in   
*                the area scanned by the laser.            
*                This is supposed to be only for           
*                simulated environments where you know     
*                the position of the simulated objects to  
*                be found.                                 
*                                                          
*   Subscribes to: 
*            - sensor_msgs/LaserScan base_scan           	                   
*                                                          
*   Publishes: 
*	     - nav_msgs/GridCells sensed_area: this is the set of scanned cells   
*	     - iuro_fake_people_detector/tracked_people people: the simulated people found           
*
*   Params:  
*            - string robot_tf_list: list of frame names to find  around the scanned area      
*            - double resolution                                  
*            - string ref_frame                                   
*            - string robot_frame                                 
*            - int8 radius: size of the area (in cells) to look around for each tf frame                       
*/ 

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <binMap.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iuro_fake_people_detector/tracked_people.h>

class fake_people_detector{

public:
	fake_people_detector():
		nh_(),
		tf_listener_(nh_,ros::Duration(1.0))
	{
		ros::NodeHandle private_nh("~");
		private_nh.param<std::string>("ref_frame", ref_frame_, "odom");
		private_nh.param<std::string>("robot_frame", robot_frame_, "base_link");
		private_nh.param<double>("resolution", resolution_, 0.1);
		private_nh.param<std::string>("robot_tf_list", robot_tf_list_, "");
		private_nh.param<int>("radius", radius_, 3);

		people_pub_ = nh_.advertise<iuro_fake_people_detector::tracked_people>("people", 1, true);
		sensed_area_pub_ = nh_.advertise<nav_msgs::GridCells>("sensed_area", 1, true);

		scan_filter_sub_.subscribe(nh_, "base_scan", 10);
		scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_filter_sub_, tf_listener_, ref_frame_, 10);
		scan_filter_->registerCallback(boost::bind(&fake_people_detector::laser_callback, this, _1));

		sensed_area_cells_.header.seq=0;
		sensed_area_cells_.header.frame_id= ref_frame_;
		sensed_area_cells_.cell_width = resolution_;
		sensed_area_cells_.cell_height = resolution_;

		ppl_tracked_msg_.header.seq=0;
		ppl_tracked_msg_.header.frame_id = ref_frame_;
		int numpersons = CountWords(robot_tf_list_.c_str());
		ROS_INFO("searching for %d people", numpersons);		
		tracking_list_.resize(numpersons);
		for (int i = 0; i < numpersons; i++){
			tracking_list_[i].id=i+1;
			tracking_list_[i].tracked_times=0; 
		}
	}

private:
	// ros node, listener and publishers
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_filter_sub_;	// laser topic subscriber
	tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;		// filters laser until transforms are available
	ros::Publisher sensed_area_pub_;
	ros::Publisher people_pub_;

	// data
	nav_msgs::GridCells sensed_area_cells_;
	iuro_fake_people_detector::tracked_people ppl_tracked_msg_;
	std::vector<iuro_fake_people_detector::person_detected> tracking_list_;

	// params
	double resolution_;
	std::string ref_frame_;
	std::string robot_frame_;
	std::string robot_tf_list_;
	int radius_;
	
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
		sensor_msgs::LaserScan myscan = *scan;
//		ROS_INFO("laser received, stamp %f",scan->header.stamp.toSec());

		sensor_msgs::PointCloud pcloud_ref;
		laser_geometry::LaserProjection projector;
//		ROS_INFO("get the points in global coordinates");
		myscan.time_increment = 0.0;

		double maxrange = myscan.range_max;
		myscan.range_max = 50.0;
		int mapwidth = (int)(maxrange*2/resolution_);
		

		tf::StampedTransform ref_to_robot;
		try{		
			projector.transformLaserScanToPointCloud (ref_frame_, myscan,  pcloud_ref, tf_listener_, laser_geometry::channel_option::Distance);
			tf_listener_.lookupTransform (ref_frame_, robot_frame_, scan->header.stamp, ref_to_robot); 
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("transform exception!!!");
		}

		// fill the data using raytracing!!
		sensed_area_cells_.cells.clear();
		geometry_msgs::Point p0;		// starting point
		//ROS_INFO("robot position: %f, %f",ref_to_robot.getOrigin().getX(),ref_to_robot.getOrigin().getY());
		p0.x = (int)(ref_to_robot.getOrigin().getX()/resolution_-0.5);
		p0.y = (int)(ref_to_robot.getOrigin().getY()/resolution_-0.5);
		//ROS_INFO("robot cell: %f, %f",p0.x,p0.y);

		for (uint i = 0; i < pcloud_ref.points.size(); i++){
			geometry_msgs::Point p1;		// ending point
//			p1.x = p0.x; p1.y=p0.y+1;
			p1.x = (int)(pcloud_ref.points[i].x /resolution_-0.5);
			p1.y = (int)(pcloud_ref.points[i].y /resolution_-0.5);

			// this is to debug
			//p0.x = p0.x*resolution_;
			//p0.y = p0.y*resolution_;
			//sensed_area_cells_.cells.push_back(p0);

			raytraceLine(p0.x, p0.y, p1.x, p1.y, sensed_area_cells_.cells, (int)fabs(maxrange/resolution_-0.5) );
			//ROS_INFO("main: cells size %d", sensed_area_cells_.cells.size());
		}

		double offsetx = p0.x+mapwidth/2;
		double offsety = p0.y-mapwidth/2;
		// insert the sensed_area cells in the binMap
		binMap sensedCells(mapwidth,mapwidth);
		for (uint i = 0; i < sensed_area_cells_.cells.size(); i++){
			sensedCells.set((int)floor(sensed_area_cells_.cells[i].x/resolution_-offsetx+0.5), (int)floor(sensed_area_cells_.cells[i].y/resolution_-offsety+0.5), true);
		}
		// TODO: Filter the repeated cells using the bin map and fill again the message.

		// debug
		//cvNamedWindow("img",0);
		//sensedCells.showMap("img");
		//cvWaitKey(10);

		std::istringstream iss(robot_tf_list_);
		int id=1;	
        	std::string frame_name;

		do {
			iss >> frame_name;	
			if (!frame_name.empty()){				
				tf::StampedTransform ref_to_target;
				try{	
					tf_listener_.waitForTransform (ref_frame_, frame_name, scan->header.stamp, ros::Duration(0.5)); 	
					tf_listener_.lookupTransform (ref_frame_, frame_name, scan->header.stamp, ref_to_target); 
				}
				catch(tf::TransformException& ex){
					ROS_ERROR("transform exception!!!");
					continue;
				}
				geometry_msgs::Point ptarget;
				ptarget.x = (int)(ref_to_target.getOrigin().getX()/resolution_-0.5);
				ptarget.y = (int)(ref_to_target.getOrigin().getY()/resolution_-0.5);

				geometry_msgs::Point pdiff;
				pdiff.x = ptarget.x-p0.x;
				pdiff.y = ptarget.y-p0.y;
				int midmap = mapwidth/2;
				if (abs(pdiff.x) < midmap && abs(pdiff.y) < midmap && check_area(sensedCells, (int)floor(pdiff.x+midmap), (int)floor(pdiff.y+midmap), radius_)){
					ROS_INFO("ROBOT %d FOUND: %s", id, frame_name.c_str());
					tracking_list_[id-1].tracked_times++;
					tracking_list_[id-1].pose.position.x=ref_to_target.getOrigin().getX();				
					tracking_list_[id-1].pose.position.y=ref_to_target.getOrigin().getY();		
					tracking_list_[id-1].pose.position.z=0;				
				}
				else{
					tracking_list_[id-1].tracked_times = 0;
				}
				id++;
			}
		}while(!iss.eof());

		ROS_INFO("main end of the loop: cells size %d", (int)sensed_area_cells_.cells.size());

		//  publish
		sensed_area_cells_.header.stamp = scan->header.stamp;
		sensed_area_cells_.header.seq++;
		sensed_area_pub_.publish(sensed_area_cells_);

		ppl_tracked_msg_.header.stamp = scan->header.stamp;		
		ppl_tracked_msg_.header.seq++;
		ppl_tracked_msg_.list.clear();
		for(uint i = 0; i<tracking_list_.size(); i++) 
			if(tracking_list_[i].tracked_times>0) 
				ppl_tracked_msg_.list.push_back(tracking_list_[i]);
		people_pub_.publish(ppl_tracked_msg_);
	}
	
        inline void raytraceLine(int x0, int y0, int x1, int y1, std::vector<geometry_msgs::Point>& cells, unsigned int max_length){
	  
		int dx = x1 - x0;			// x distance
        	int dy = y1 - y0;			// y distance

        	unsigned int abs_dx = abs(dx);		// abs x distance
        	unsigned int abs_dy = abs(dy);		// abs y distance

        	int offset_dx = sign(dx);		// direction in x ax			
        	int offset_dy = sign(dy);		// direction in y ax

        	double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1)); // euclidean dist p0 to p1

        	//we need to chose how much to scale our dominant dimension, based on the maximum length of the line
        	double scale = std::min(1.0,  max_length / dist); 

        	//if x is dominant
        	if(abs_dx >= abs_dy){
        		int error_y = abs_dx / 2; //???
        		bresenham2D(abs_dx, abs_dy, 1, error_y, x0, y0, offset_dx, offset_dy, (unsigned int)(scale * abs_dx), cells);
          	}
		else{ //otherwise y is dominant
          		int error_x = abs_dy / 2;
          		bresenham2D(abs_dy, abs_dx, 0, error_x, y0, x0, offset_dy, offset_dx, (unsigned int)(scale * abs_dy), cells);
		}
		//ROS_INFO("raytrace: cells size %d", cells.size());

        }

        inline void bresenham2D(unsigned int abs_da, unsigned int abs_db, int dir, int error_b, int p_x, int p_y, int offset_a, int offset_b, unsigned int max_length, 				std::vector<geometry_msgs::Point>& cells){
        	int end = std::min(max_length, abs_da);
		//ROS_INFO("laser cell: %d, %d",p_x,p_y);
		for(int i = 0; i < end; ++i){
			geometry_msgs::Point p;
			if (dir){	p.x=p_x*resolution_; p.y=p_y*resolution_;}
			else	{	p.x=p_y*resolution_; p.y=p_x*resolution_;}
        		cells.push_back(p);
        		p_x += offset_a;
        		error_b += abs_db;
        		if(error_b >= abs_da){
        			p_y += offset_b;
        			error_b -= abs_da;
        		}
        	}
		geometry_msgs::Point p;
		if (dir){	p.x=p_x*resolution_; p.y=p_y*resolution_;}
		else	{	p.x=p_y*resolution_; p.y=p_x*resolution_;}
      		cells.push_back(p);
		//ROS_INFO("bresenham: cells size %d", cells.size());
        }

	inline int sign(int x){
        	return x > 0 ? 1.0 : -1.0;
      	}

	bool check_area(binMap& area, int x, int y, int r){
		bool itsok = false;
		for (int i = x - r; i < x + r; i++){
			for (int j = y - r; j < y + r; j++){
				if (area.get(i,j)){
					itsok = true;
					break;
				}
			}
		}
		return (itsok);
	}

	int CountWords(const char* str){
		if (str == NULL) return 0; 

		bool inSpaces = true;
		int numWords = 0;

		while (*str != NULL){
			if (std::isspace(*str)){
				inSpaces = true;
			}
			else if (inSpaces){
				numWords++;
				inSpaces = false;
			}
			++str;
		}
		return numWords;
	}

};

int main(int argc, char *argv[]) {
	
	ros::init(argc, argv, "fake_people_detector");
	fake_people_detector fpd;
	ros::spin();
	return 0;
	
}



