
#include <map_fusion/map_fusion.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <nav_msgs/GetMap.h>
#include <sstream>
#include <string>
//#include <dynamicvoronoi.h>
#include <exception>

using namespace cv;
using namespace std;

// constructor
map_fusion::map_fusion():
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(1.0)),
	num_robots_(2),
	delay_(8), // seconds
	map_initialized_(false)
{

	// Client for the input maps
	for (int r=0; r< num_robots_; r++){
		char maptopicstr[100];
		sprintf(maptopicstr,"robot_%d/dynamic_map",r);
		map_clients_.push_back(nh_.serviceClient<nav_msgs::GetMap>(maptopicstr));
	}

	// TODO: debug windows controlled by parameter
	// open windows to display
	 cv::namedWindow("matches",0);
	//cv::namedWindow("keypoints",0);
	// TODO: num_robots and delay should be read from parameters

	// map publisher and server 
	map_server_ = nh_.advertiseService("fused_map", &map_fusion::map_callback, this );
	map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("fused_map", 3);

	// control loop timer
	ROS_INFO("intiating map fusion timer");
	timer_ = nh_.createTimer(ros::Duration(delay_), boost::bind(&map_fusion::timer_callback, this, _1) );

	transform_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&map_fusion::tranform_publish_callback, this, _1) );

}

// fused map callback, just returns the last fused map
bool map_fusion::map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response){
	boost::mutex::scoped_lock(map_mutex_);
	if(map_initialized_){
		response.map = fused_map_;
    		return true;
  	}
  	else  return false;
}
// Callback called with a fixed timer to update the fused map
void map_fusion::tranform_publish_callback(const ros::TimerEvent& t){
//	boost::mutex::scoped_lock(map_mutex_);
	if(map_initialized_){
		//ROS_ERROR("tranform published");
		tf_broadcaster_.sendTransform(tf::StampedTransform(maps_transform_, ros::Time::now(), frame_id_1_, frame_id_2_));
	}
}
// Callback called with a fixed timer to update the fused map
void map_fusion::timer_callback(const ros::TimerEvent& t){

	if(ros::ok()){ 

		std::vector<nav_msgs::GetMap> requests(num_robots_);
		std::vector<std::vector<cv::KeyPoint> > keypoints(num_robots_);
		std::vector<cv::Mat> descriptors(num_robots_);
		std::vector<cv::Mat> mapasimg(num_robots_);
		std::vector<cv::Mat> mask(num_robots_);
		// 1. Get the last maps
		ROS_INFO("TIMER CALLBACK");

		// lets go parallel!!!
//		ROS_INFO("GOING PARALLEL!!!");
		boost::thread workerThread(boost::bind(&map_fusion::process_map_thread, this, 0, &requests[0], &keypoints[0], &descriptors[0], &mapasimg[0], &mask[0]));
//		process_map_thread(0, &requests[0], &keypoints[0], &descriptors[0], &mapasimg[0],&mask[0]);
		process_map_thread(1, &requests[1], &keypoints[1], &descriptors[1], &mapasimg[1],&mask[1]);
//		ROS_INFO("waiting for the aux thread");
		workerThread.join();
//		ROS_INFO("thread finished");

//		for (int r=0; r< num_robots_; r++){
//			do{
//				ROS_INFO("requesting dynamic_map...");
//				res = map_clients_[r].call(requests[r]); 	
//				if (res){
//					ROS_INFO("map received %dx%d",requests[r].response.map.info.width,requests[r].response.map.info.height);
//					// to cvmat
//					OccupancyGrid2CvMat(requests[r].response.map, mapasimg[r]);
//					// find features and descriptors
//					ROS_INFO("Feature extraction...");
//					find_features(mapasimg[r], keypoints[r], descriptors[r]);
//			  	}
//				else{ 	ROS_ERROR("Failed to call service map");
//					delay.sleep();
//					ROS_INFO("trying again...");
//				}
//			} while (!res); 
//		}

		// Match features
// 		ROS_INFO("matching %d features with %d features", descriptors[0].rows, descriptors[1].rows);
		Ptr<DescriptorMatcher> matcher = new cv::FlannBasedMatcher();
		std::list<cv::DMatch> matches;
		crossCheckMatching(matcher, descriptors[0], descriptors[1], matches, 3, 200.0f);
		matcher.release();
		ROS_INFO("MATCHES FOUND %d", matches.size());

		// only the top 10
		matches.sort();
		std::vector< DMatch > good_matches;
		uint i = 0;
		std::list<DMatch>::iterator lastit=matches.begin();
		for (std::list<DMatch>::iterator it=matches.begin(); it != matches.end() ; it++){
			bool alreadydone = false;
			for(uint j=0; j<good_matches.size(); j++){
				if (keypoints[0][it->queryIdx].pt.x == keypoints[0][good_matches[j].queryIdx].pt.x &&
			 	    keypoints[0][it->queryIdx].pt.y == keypoints[0][good_matches[j].queryIdx].pt.y &&				
				    keypoints[1][it->trainIdx].pt.x == keypoints[1][good_matches[j].trainIdx].pt.x &&
			 	    keypoints[1][it->trainIdx].pt.y == keypoints[1][good_matches[j].trainIdx].pt.y) alreadydone=true;				
			}
			if (alreadydone) continue;
			good_matches.push_back(*it); 
			lastit=it;
		//	ROS_INFO("GOOD  MATCH (%d) (%f, %f) ==> (%f, %f)",it->queryIdx, keypoints[0][it->queryIdx].pt.x, keypoints[0][it->queryIdx].pt.y, 
		//						      keypoints[1][it->trainIdx].pt.x, keypoints[1][it->trainIdx].pt.y );
			i++;
			if (i >= 50) break;
		}

//		// draw matches
//		ROS_INFO("DRAW MATCHES");
//		if (good_matches.size()>0){
//			cv::Mat drawImg;
//			drawMatches( mapasimg[0], keypoints[0], mapasimg[1], keypoints[1], good_matches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0) );
//			cv::imshow("matches", drawImg);
//			cv::waitKey(25);
//		}


//		cv::Mat transform = cv::estimateRigidTransform(mapasimg[0], mapasimg[1],false);
//		ROS_INFO("transform: %d x %d (D%d;%dC)", transform.rows, transform.cols, transform.depth(), transform.channels());
//		ROS_INFO("transform matrix: R=[%f, %f ,%f ; %f, %f, %f]",    transform.at<double>(0,0),transform.at<double>(0,1),transform.at<double>(0,2),
//									transform.at<double>(1,0),transform.at<double>(1,1),transform.at<double>(1,2));
//		ROS_INFO("TRANSFORM x = %f, y = %f, th = %f", transform.at<double>(0,2), transform.at<double>(1,2), atan2(transform.at<double>(1,0),transform.at<double>(0,0)));


		ROS_INFO("ALIGN MAPS");
		align_maps(keypoints[0], keypoints[1], good_matches, map1_to_map2_, mapasimg[0], mapasimg[1]);

		boost::mutex::scoped_lock(map_mutex_);

		ROS_INFO("PUBLISH TRANSFORM");
		int tx = ceil(map1_to_map2_.getOrigin().getX());
		int ty = ceil(map1_to_map2_.getOrigin().getY());
		float th = -tf::getYaw(map1_to_map2_.getRotation());
		
		maps_transform_.setOrigin(tf::Point( -requests[1].response.map.info.origin.position.x*cos(-th) +
						    -requests[1].response.map.info.origin.position.y*sin(-th) +
						    tx*requests[1].response.map.info.resolution + requests[0].response.map.info.origin.position.x,
						    +requests[1].response.map.info.origin.position.x*sin(-th) +
						    -requests[1].response.map.info.origin.position.y*cos(-th) +
						    ty*requests[1].response.map.info.resolution + requests[0].response.map.info.origin.position.y,
						    0));
		maps_transform_.setRotation(tf::createQuaternionFromYaw(th));
		frame_id_1_=requests[0].response.map.header.frame_id;
		frame_id_2_=requests[1].response.map.header.frame_id;

		ROS_INFO("FUSE MAPS");
		fusemaps(requests[0].response.map, requests[1].response.map, map1_to_map2_, fused_map_);
	
		ROS_INFO("PUBLISH MAPS");
		map_pub_.publish(fused_map_);
	}
}

void map_fusion::process_map_thread(int n, nav_msgs::GetMap* request, std::vector<cv::KeyPoint>* keypoints, cv::Mat* descriptors, cv::Mat* mapasimg, cv::Mat* mask){
	ROS_INFO("requesting dynamic_map [%d]...",n);
	bool res=false;
	ros::Duration delay(0.2);
	do{
		res = map_clients_[n].call(*request); 	
		if (res){
//			ROS_INFO("map %d received %dx%d", n, request->response.map.info.width, request->response.map.info.height);
			// to cvmat
			OccupancyGrid2CvMat(request->response.map, *mapasimg, *mask);
			// find features and descriptors
			find_features(*mapasimg, *keypoints, *descriptors,n, *mask);
	  	}
		else{ 	ROS_ERROR("Failed to call service map %d",n);
			delay.sleep();
			ROS_INFO("trying again...");
		}
	} while (!res); 
}

void map_fusion::find_features(cv::Mat& map, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,int indexrobot,const cv::Mat& mask){

	// Distance Transform
	cv::Mat distance(map.rows, map.cols, CV_32FC1);
	cv::Mat distanceNorm(map.rows, map.cols, CV_8UC1);

//	ROS_INFO("distance trasform...");
	cv::distanceTransform(map, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
//	ROS_INFO("normalize...");
//	cv::normalize(distance, distance, 0.0, 1.0, cv::NORM_MINMAX);
	cv::normalize(distance, distanceNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);
	distance.release();

	// DETECT FEATURES
	//cv::FastFeatureDetector detector;					// FAST
	cv::SiftFeatureDetector detector;//(0.01,10,2,2);			// SIFT
	//cv::SurfFeatureDetector detector;					// SURF
	//cv::GoodFeaturesToTrackDetector detector(500,0.8,20,7,true);		// HARRIS
	
	ROS_INFO("detect features...");
	detector.detect( distanceNorm, keypoints);
	ROS_INFO("FEATURES DETECTED: %d", keypoints.size());
//	std::vector<cv::KeyPoint> keypoints2;
//	detector.detect( map, keypoints2 );
//	detector.detect( map, keypoints );

	// Extract features
	cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SiftDescriptorExtractor();				// SIFT
	//cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor();			// SURF

	ROS_INFO("evaluating descriptors...");
	extractor->compute(distanceNorm, keypoints, descriptors);
//	extractor->compute(map, keypoints, descriptors);
	ROS_INFO("DESCRIPTORS EVALUATED size %d for %d features", descriptors.cols, descriptors.rows);

	extractor.release();

	map = distanceNorm;

	// display keypoints
//	cv::Mat img_keypoints(map.rows, map.cols, CV_8UC3);
//	cv::drawKeypoints( distanceNorm, keypoints, img_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
//	cv::Mat img_keypoints2(map.rows, map.cols, CV_8UC3);
//	cv::drawKeypoints( map, keypoints2, img_keypoints2, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );

//	if (indexrobot==0){
//		cv::imwrite("/home/mjulia/distance0.png", img_keypoints);
//		cv::imwrite("/home/mjulia/grid0.png", img_keypoints2);
//	}
//	else{
//		cv::imwrite("/home/mjulia/distance1.png", img_keypoints);
//		cv::imwrite("/home/mjulia/grid1.png", img_keypoints2);

//	}

	//cv::imshow("keypoints", img_keypoints);
	//cv::waitKey(25);
}

void map_fusion::align_maps(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch> matches, tf::Transform& map1_to_map2, const cv::Mat& map1, const cv::Mat& map2 ){
	
	if (matches.size()<2){
		map1_to_map2.setOrigin(tf::Point(-2000,0,0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(0));
		ROS_ERROR("COULDN'T FIND MAP ALIGNMENT");
		return;
	}

	// THIS IS RANSAC
	double edth = 15; 	// euclidean distance threshold between maps for the 2 points selected (in pixels) 
	double inlierth = 15;	// euclidean distance between the proyected point with the test tranform and the matched point (in pixels) 
	int inliers=0;
	double txx=0, tyy=0, thh=0;
	std::vector<uint> bestgoodmatches;
	std::vector<DMatch> finalmatches;
	KeyPoint p1m1, p1m2, p2m1, p2m2;
	float dA, dB, dC, dD, dAA, dBB, dCC, dDD;
	float th,tx,ty;
	for (uint iter=0; iter < 5*matches.size(); iter++){

		int p1i, p2i;
	 	p1i = rand() % matches.size();
		do{
			p2i = rand() % matches.size();
		}while (p1i==p2i);

		p1m1 = keypoints1[matches[p1i].queryIdx]; 
		p1m2 = keypoints2[matches[p1i].trainIdx];
		p2m1 = keypoints1[matches[p2i].queryIdx]; 
		p2m2 = keypoints2[matches[p2i].trainIdx];

		dA = p1m2.pt.x - p2m2.pt.x;
		dB = p1m2.pt.y - p2m2.pt.y;
		dC = p1m1.pt.x - p2m1.pt.x;
		dD = p1m1.pt.y - p2m1.pt.y;

		if (sqrt(dA*dA+dB*dB)-sqrt(dC*dC-dD*dD)>edth) continue;

		th = atan2(dB*dC - dA*dD, dA*dC + dB*dD);
		tx = p1m1.pt.x - p1m2.pt.x * cos(th) - p1m2.pt.y * sin(th); 
		ty = p1m1.pt.y - p1m2.pt.y * cos(th) + p1m2.pt.x * sin(th);

		int goodpoints = 0;
		std::vector<uint> goodmatches;
		for (uint i=0; i<matches.size(); i++){
			double x = tx + cos(th)*keypoints2[matches[i].trainIdx].pt.x + sin(th)*keypoints2[matches[i].trainIdx].pt.y ;
			double y = ty - sin(th)*keypoints2[matches[i].trainIdx].pt.x + cos(th)*keypoints2[matches[i].trainIdx].pt.y ;
			if ( sqrt( pow(keypoints1[matches[i].queryIdx].pt.x-x,2) + pow(keypoints1[matches[i].queryIdx].pt.y-y,2) ) < inlierth ){
//				dAA = p1m2.pt.x - keypoints2[matches[i].trainIdx].pt.x;
//				dBB = p1m2.pt.y - keypoints2[matches[i].trainIdx].pt.y;
//				dCC = p1m1.pt.x - keypoints1[matches[i].queryIdx].pt.x;
//				dDD = p1m1.pt.y - keypoints1[matches[i].queryIdx].pt.y;
//				if (sqrt(dAA*dAA+dBB*dBB)-sqrt(dCC*dCC-dDD*dDD)<edth){
					goodpoints++;
					goodmatches.push_back(i);
//				}
			}
		}
//		ROS_INFO("TRANSFORM IS x = %f, y = %f, th = %f (inliers: %d)", tx, ty, th, goodpoints);

		if (goodpoints>inliers){
			inliers = goodpoints;
			txx = tx;
			tyy = ty;
			thh = th;
			bestgoodmatches = goodmatches;
		}
	}

//	THIS IS THE 2 POINTS TRANSFORM
	ROS_INFO("2P TRANSFORM IS x = %f, y = %f, th = %f (inliers: %d)", txx, tyy, thh, inliers);
//	map1_to_map2.setOrigin(tf::Point(txx,tyy,0));
//	map1_to_map2.setRotation(tf::createQuaternionFromYaw(thh));

	// Now we do Least Squares with all the inliers
	if (inliers >= 5){
		cv::Mat ptsm1(2*bestgoodmatches.size(), 1, CV_64FC1);
		cv::Mat ptsm2(2*bestgoodmatches.size(), 4, CV_64FC1);
		for (uint i=0; i<bestgoodmatches.size(); i++){
			finalmatches.push_back(matches[bestgoodmatches[i]]);
			ptsm1.at<double>(2*i,0) = keypoints1[matches[bestgoodmatches[i]].queryIdx].pt.x;
			ptsm1.at<double>(2*i+1,0) = keypoints1[matches[bestgoodmatches[i]].queryIdx].pt.y;

			ptsm2.at<double>(2*i,0) = keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.x;
			ptsm2.at<double>(2*i,1) = keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.y;
			ptsm2.at<double>(2*i,2) = 1;
			ptsm2.at<double>(2*i,3) = 0;
			ptsm2.at<double>(2*i+1,0) = keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.y;
			ptsm2.at<double>(2*i+1,1) = -keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.x;
			ptsm2.at<double>(2*i+1,2) = 0;
			ptsm2.at<double>(2*i+1,3) = 1;

			ROS_INFO("inlier %d, dist=%f", i, matches[bestgoodmatches[i]].distance);

		}
		cv::Mat trans = ptsm2.inv(cv::DECOMP_SVD)*ptsm1;
		ROS_INFO("LS TRANSFORM IS x = %f, y = %f, th = %f (inliers: %d)", trans.at<double>(2,0), trans.at<double>(3,0), atan2(trans.at<double>(1,0),trans.at<double>(0,0)), inliers);

		map1_to_map2.setOrigin(tf::Point(trans.at<double>(2,0), trans.at<double>(3,0),0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(atan2(trans.at<double>(1,0),trans.at<double>(0,0))));
	}
	else{
		map1_to_map2.setOrigin(tf::Point(-2000,0,0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(0));
		ROS_ERROR("COULDN'T FIND MAP ALIGNMENT");
	}

	if (finalmatches.size()>0){
		cv::Mat drawImg;
		drawMatches( map1, keypoints1, map2, keypoints2, finalmatches, drawImg, CV_RGB(255, 255, 0), CV_RGB(255, 0, 0) );
		cv::imshow("matches", drawImg);
		cv::waitKey(25);
	}


}

void map_fusion::fusemaps(const nav_msgs::OccupancyGrid& map1, const nav_msgs::OccupancyGrid& map2, const tf::Transform& map1_to_map2, nav_msgs::OccupancyGrid& outmap ){
	
	int tx = ceil(map1_to_map2.getOrigin().getX());
	int ty = ceil(map1_to_map2.getOrigin().getY());
	float th = tf::getYaw(map1_to_map2.getRotation());
	//ROS_INFO("transform (%d, %d, %f)", tx, ty, th);

	int aux = ceil((float)map2.info.height*sin(-th)-tx);
	int disx = (aux>0)? aux: 0;
	int disy = (ty<0)? -ty: 0;
//	ROS_INFO("DIS %d, %d", disx, disy);

	int aux2 = ceil((float)map2.info.width*cos(-th)+tx);
	int expx = disx - map1.info.width + ((aux2>(int)map1.info.width)? aux2: (int)map1.info.width);
	int aux3 = ceil((float)map2.info.width*sin(-th)+(float)map2.info.height*cos(-th) +ty);
	int expy = disy - map1.info.height + ((aux3>(int)map1.info.height)? aux3: (int)map1.info.height);

//	expx = map1.info.width;
//	expy = map1.info.height;
//	disx = map1.info.width/2.0;
//	disy = map1.info.height/2.0;

//	ROS_INFO("EXP %d, %d", expx, expy);
	outmap.header.frame_id = map1.header.frame_id;
	outmap.info.resolution = map1.info.resolution;
	outmap.info.width = map1.info.width+expx;
	outmap.info.height = map1.info.height+expy;
	outmap.info.origin.position.x = map1.info.origin.position.x-disx*outmap.info.resolution;
	outmap.info.origin.position.y = map1.info.origin.position.y-disy*outmap.info.resolution;
	outmap.info.origin.orientation = map1.info.origin.orientation;
//	ROS_INFO("fused map size %d x %d", outmap.info.width,outmap.info.height);
	outmap.data.resize(outmap.info.width*outmap.info.height);

	float sinth = sin(th);
	float costh = cos(th);
	for (uint x=0; x < outmap.info.width; x++){
		int ix = x-disx;
		for (uint y=0; y < outmap.info.height; y++){
			int iy = y-disy;
			int moutind = x+y*outmap.info.width;
			// map1
			outmap.data[moutind] = ( ix>=0 && ix<((int)(map1.info.width)) && iy>=0 && iy<((int)(map1.info.height)))?  map1.data[ix+iy*map1.info.width] : -1;
		
			int jx = floor((ix-tx)*costh-(iy-ty)*sinth+0.5) ;
			int jy = floor((ix-tx)*sinth+(iy-ty)*costh+0.5) ;
			if (jx>=0 && jx<((int)(map2.info.width)) && jy>=0 && jy<((int)(map2.info.height))){
				int m2ind = jx+jy*map2.info.width;
				if (outmap.data[moutind]==-1)
					outmap.data[moutind] = map2.data[m2ind];
				else if (map2.data[m2ind] !=-1)
					outmap.data[moutind] = (outmap.data[moutind] > map2.data[m2ind])? outmap.data[moutind] : map2.data[m2ind] ;
			}
		}
	}

	map_initialized_=true;
}
// converts from occupancygrid message to opencv matrix
void map_fusion::OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, cv::Mat& mask){
	// convert map to image
	mapasimg.create(map.info.height, map.info.width, CV_8UC1);
	mask.create(map.info.width, map.info.height, CV_8UC1);
	memcpy (mapasimg.data, &map.data[0], map.info.width*map.info.height*sizeof(uchar));

	bool init=false;
	uint minx=0,miny=0,maxx=0,maxy=0;
	// put a free value to unknown cells
	for (uint i=0; i<map.info.width*map.info.height; i++){
		if(mapasimg.data[i]==255) mapasimg.data[i]=0;
		else{
			div_t res = div((int)i,(int)map.info.width);
			if (!init){ init=true; minx=res.quot; maxx=res.quot; miny=res.rem; maxy=res.rem;}
			else{
				if (res.quot < minx) minx=res.quot;
				else if (res.quot > maxx) maxx=res.quot;
				if (res.rem < miny) miny=res.rem;
				else if (res.rem > maxy) maxy=res.rem;
			}
//			mask.data[i]=1;
			if(mapasimg.data[i]<=50) mapasimg.data[i]=255;	
			else mapasimg.data[i]=0;
		}
	}
//	mapasimg.adjustROI(miny,maxy,minx,maxx);
	for (uint x=minx; x<=maxx; x++)
		for (uint y=miny; y<=maxy; y++)
			mask.data[x+y*map.info.width]=1;
}

void map_fusion::crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         list<DMatch>& filteredMatches12, int knn, float disth)
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
    DMatch forward, backward;
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx)
                {
		    if(forward.distance < disth){
	                    filteredMatches12.push_back(forward);
	                    findCrossCheck = true;
	                    break;
		    }
	 	   // else ROS_ERROR("rejected match with distance %f", forward.distance);
                }
            }
            if( findCrossCheck ) break;
        }
    }
}
