
#include <convex_segmentation/convex_segmentation.h>
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
convex_segmentation::convex_segmentation():
	private_nh_("~"),
	tf_listener_(nh_,ros::Duration(1.0)),
	delay_(3), // seconds
	map_initialized_(false)
{

	// Client for the input maps
	map_client_ = nh_.serviceClient<nav_msgs::GetMap>("dynamic_map");

	// TODO: debug windows controlled by parameter
	// open windows to display
	 cv::namedWindow("voronoi",0);
//	 cv::namedWindow("distance1",0);
//	 cv::namedWindow("distance2",0);
//	 cv::namedWindow("distance3",0);

	// TODO: num_robots and delay should be read from parameters

	// map publisher and server 
	map_server_ = nh_.advertiseService("voronoi_map_server", &convex_segmentation::map_callback, this );
	map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("voronoi_map", 3);

	// control loop timer
	ROS_INFO("initiating segmentation timer");
	timer_ = nh_.createTimer(ros::Duration(delay_), boost::bind(&convex_segmentation::timer_callback, this, _1) );
}

// voronoi map service callback, just returns the last segmented map
bool convex_segmentation::map_callback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response){
	boost::mutex::scoped_lock(map_mutex_);
	if(map_initialized_){
		response.map = fused_map_;
    		return true;
  	}
  	else  return false;
}

// Callback called with a fixed timer to update the segmented map
void convex_segmentation::timer_callback(const ros::TimerEvent& t){

	if(ros::ok()){ 

		nav_msgs::GetMap request;
		cv::Mat mapasimg;
		cv::Mat mask;
		// 1. Get the last maps
		ROS_INFO("TIMER CALLBACK");

		process_map(&request, &mapasimg, &mask);

		ROS_INFO("PUBLISH MAPS");
	}
}

void convex_segmentation::process_map(nav_msgs::GetMap* request, cv::Mat* mapasimg, cv::Mat* mask){
	ROS_INFO("requesting dynamic_map ...");
	bool res=false;
	ros::Duration delay(0.2);
	do{
		res = map_client_.call(*request); 
		if (res){
//			ROS_INFO("map %d received %dx%d", n, request->response.map.info.width, request->response.map.info.height);
			// to cvmat
			ROS_INFO("map received, converting to opencv...");
			OccupancyGrid2CvMat(request->response.map, *mapasimg, *mask);
			// find features and descriptors
			ROS_INFO("converted, voronoi tranform now ...");
			cv::Mat segmentedMap;
			segment(*mapasimg, segmentedMap);
			ROS_INFO("voronoi computed");
	  	}
		else{ 	ROS_ERROR("Failed to call service map");
			delay.sleep();
			ROS_INFO("trying again...");
		}
	} while (!res); 
}
static const uchar colors[][3] =
{
	{0,0,0},
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

void convex_segmentation::segment(const cv::Mat& map, cv::Mat& segmented){
	//cv::Mat map = inmap(cv::Rect(100,100,100,100));
	// Distance Transform
	cv::Mat distance(map.rows, map.cols, CV_32FC1);
	cv::Mat distanceNorm(map.rows, map.cols, CV_8UC1);
	cv::Mat distanceNeg(map.rows, map.cols, CV_32FC1);
	cv::Mat distanceNegNorm(map.rows, map.cols, CV_8UC1);
	cv::Mat distanceNegColor(map.rows, map.cols, CV_32FC3);

	ROS_INFO("distance trasform...");
	cv::Mat labels(map.rows, map.cols, CV_32FC1);
	cv::distanceTransform(map, distance, labels, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
	ROS_INFO("normalize...");
	cv::normalize(distance, distanceNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	float max = *std::max_element(distance.begin<float>(),distance.end<float>());
	ROS_INFO("maximum %f", max);

	ROS_INFO("negative");
	// create a matrix with all elements equal to 255 for subtraction
	cv::Mat sub_mat = Mat::ones(distance.size(), CV_32FC1)*max; 
	//subtract the original matrix by sub_mat to give the negative output new_image
	cv::subtract(sub_mat, distance, distanceNeg);
	cv::normalize(distanceNeg, distanceNegNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	//ROS_INFO("covert to 3 channels...");
	//cv::Mat in[] = {distanceNeg, distanceNeg, distanceNeg};
	//cv::merge(in, 3, distanceNegColor);
	//cv::normalize(distanceNegColor, distanceNegColor2, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC3);

	// watershed 
	ROS_INFO("watershed...");
	cv::Mat newlabels(map.rows, map.cols, CV_32SC1);
	flood_fill(distanceNeg, newlabels);
	// normalize
	cv::Mat newlabels2(map.rows, map.cols, CV_8UC1);
	cv::normalize(newlabels, newlabels2, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC3);

	// colour voronoi labels
	ROS_INFO("colour voronoi img...");
	cv::Mat voronoi(map.rows, map.cols, CV_8UC3);
 	int i, j;
        for( i = 0; i < newlabels.rows; i++ )
        {
            int* ll = newlabels.ptr<int>(i);       //before--> (int*)(labels->imageData + i*labels->widthStep)
            float* dd = distance.ptr<float>(i);     //before--> (float*)(dist->imageData + i*dist->widthStep)
            uchar* d = voronoi.ptr<uchar>(i);    //before--> (uchar*)(dist8u->imageData + i*dist8u->widthStep)

            for( j = 0; j < newlabels.cols; j++ )
            {
                int idx = ll[j] == 0 || dd[j] == 0 ? 0 : (ll[j]-1)%20 + 1;
                int b = cvRound(colors[idx][0]);        // if there is an option to cvRound in the new OpenCV C++ API, tell me please
                int g = cvRound(colors[idx][1]);
                int r = cvRound(colors[idx][2]);
                d[j*3] = cv::saturate_cast<uchar>(b);       //before--> (uchar)b; 
                d[j*3+1] = cv::saturate_cast<uchar>(g);     //before--> (uchar)g; 
                d[j*3+2] = cv::saturate_cast<uchar>(r);     //before--> (uchar)r;
            }
        }

	segmented = newlabels;
	ROS_INFO("display...");

//	cv::imshow("distance1", distanceNorm);
//	cv::imshow("distance2", distanceNegNorm);
//	cv::imshow("distance3", newlabels2);
	cv::imshow("voronoi", voronoi);
	cv::waitKey(100);
}

void convex_segmentation::flood_fill(const cv::Mat& distance, cv::Mat& labels){

	labels.setTo(cv::Scalar(0));
	cv::Mat seeds(distance.rows, distance.cols, CV_8UC1);
	seeds.setTo(cv::Scalar(0));

	float max = *std::max_element(distance.begin<float>(),distance.end<float>());
	cv::Mat distanceNorm(distance.rows, distance.cols, CV_8UC1);
	cv::normalize(distance, distanceNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	cv::Mat voronoi(distance.rows, distance.cols, CV_8UC3);
	cv::Mat in[] = {distanceNorm, distanceNorm, distanceNorm};
	cv::merge(in, 3, voronoi); 
	cv::imshow("voronoi", voronoi);
	cv::waitKey(0);
	ROS_INFO("GO");
	int numlabels = 0;
	for (int i = 0; i < (int)distance.cols; i++){
		for (int j = 0; j < (int)distance.rows; j++){
//			ROS_INFO("point %d,%d: value=%f",i,j,distance.at<float>(i,j));
			if (distance.at<float>(i,j)==max){ 
//				ROS_INFO("outside");
				continue;
			}

//			ROS_INFO("recursive flood fill");
			rec_flood_fill(distance, labels, seeds, i, j, numlabels);
//			ROS_INFO("numlabels: %d", numlabels);

//			// colour voronoi labels
//			//ROS_INFO("colour voronoi img...");
//		 	int i, j;
//			for( i = 0; i < labels.rows; i++ )
//			{
//			    int* ll = labels.ptr<int>(i);       //before--> (int*)(labels->imageData + i*labels->widthStep)
//			   // const float* dd = distance.ptr<float>(i);     //before--> (float*)(dist->imageData + i*dist->widthStep)
//			    uchar* d = voronoi.ptr<uchar>(i);    //before--> (uchar*)(dist8u->imageData + i*dist8u->widthStep)

//			    for( j = 0; j < labels.cols; j++ )
//			    {
//				if (ll[j] >0){
//					int idx = (ll[j] == 0) ? 0 : (ll[j]-1)%8 + 1;
//					int b = cvRound(colors[idx][0]);        // if there is an option to cvRound in the new OpenCV C++ API, tell me please
//					int g = cvRound(colors[idx][1]);
//					int r = cvRound(colors[idx][2]);
//					d[j*3] = cv::saturate_cast<uchar>(b);       //before--> (uchar)b; 
//					d[j*3+1] = cv::saturate_cast<uchar>(g);     //before--> (uchar)g; 
//					d[j*3+2] = cv::saturate_cast<uchar>(r);     //before--> (uchar)r;
//				}
//			    }
//			}

//			//cv::imshow("distance1", distanceNorm);
//			cv::imshow("voronoi", voronoi);
//			cv::waitKey(5);
		}
	}
	// colour voronoi labels
	//ROS_INFO("colour voronoi img...");
 	int i, j;
	for( i = 0; i < labels.rows; i++ )
	{
	    int* ll = labels.ptr<int>(i);       //before--> (int*)(labels->imageData + i*labels->widthStep)
	   // const float* dd = distance.ptr<float>(i);     //before--> (float*)(dist->imageData + i*dist->widthStep)
	    uchar* d = voronoi.ptr<uchar>(i);    //before--> (uchar*)(dist8u->imageData + i*dist8u->widthStep)

	    for( j = 0; j < labels.cols; j++ )
	    {
		if (ll[j] >0){
			int idx = (ll[j] == 0) ? 0 : (ll[j]-1)%20 + 1;
			int b = cvRound(colors[idx][0]);        // if there is an option to cvRound in the new OpenCV C++ API, tell me please
			int g = cvRound(colors[idx][1]);
			int r = cvRound(colors[idx][2]);
			d[j*3] = cv::saturate_cast<uchar>(b);       //before--> (uchar)b; 
			d[j*3+1] = cv::saturate_cast<uchar>(g);     //before--> (uchar)g; 
			d[j*3+2] = cv::saturate_cast<uchar>(r);     //before--> (uchar)r;
		}
	    }
	}
	cv::imshow("voronoi", voronoi);
	cv::waitKey(0);

	ROS_INFO("segmented into %d regions", numlabels);
}

void convex_segmentation::rec_flood_fill(const cv::Mat& distance, cv::Mat& labels, cv::Mat& seeds, int i, int j, int& numlabels){
	if (labels.at<int>(i,j) > 0) return;
	//ROS_INFO("here %d, %d, value = %f", i, j, distance.at<float>(i,j));
	// TODO: check that distance is 32bit float and label 32bit integer
	int minval = distance.at<float>(i,j);
	int gi=0, gj=0;
	bool isLocalMinimum = true;
	for (int k=4; k<=4; k++){
		for (int ii = i-k; ii <= i+k; ii++){
			for (int jj = j-k; jj <= j+k; jj++){
				if (ii <0 || jj <0 || ii>=(int)distance.cols || jj>=(int)distance.rows) continue;
				//if (ii >=i-(k-1) && ii <=i+(k-1) && jj >= j-(k-1) && jj<=j+(k-1)) continue;
				if (ii==i && jj==j) continue;
				if (minval > distance.at<float>(ii,jj)){
					gi = ii; gj = jj;
					minval = distance.at<float>(ii,jj);
					isLocalMinimum = false;
					break;
				}
			}
		}
		if (!isLocalMinimum) break;
	}
	if (isLocalMinimum){
		bool nextToLabeledMinimum=false;
//		for (int k=1; k<=20; k++){
//			for (int ii = i-k; ii <= i+k; ii++){
//				for (int jj = j-k; jj <= j+k; jj++){
//					if (ii <0 || jj <0 || ii>=(int)distance.cols || jj>=(int)distance.rows) continue;
//					if (ii >=i-(k-1) && ii <=i+(k-1) && jj >= j-(k-1) && jj<=j+(k-1)) continue;
//					if (seeds.at<uchar>(ii,jj) > 0){
//						labels.at<int>(i,j) = labels.at<int>(ii,jj);
//						nextToLabeledMinimum = true;
//						if (distance.at<float>(ii,jj)>=minval ){
//		//					ROS_INFO("joinned to near cluster");
//							seeds.at<uchar>(ii,jj) = 0;
//							seeds.at<uchar>(i,j) = 255;
//						}
//						break;
//					}
//				}
//			}
//		}
		if (!nextToLabeledMinimum && minval < 30){
			numlabels++;
//			ROS_INFO("new cluster %d", numlabels);
			labels.at<int>(i,j) = numlabels;
			seeds.at<uchar>(i,j) = 255;
		}
	}
	else{
//		rec_flood_fill(distance, labels, seeds, gi, gj, numlabels);
//		labels.at<int>(i,j) = labels.at<int>(gi,gj);
	}
}


// converts from occupancygrid message to opencv matrix
void convex_segmentation::OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, cv::Mat& mask){
	// convert map to image
	mapasimg.create(map.info.height, map.info.width, CV_8UC1);
	mask.create(map.info.width, map.info.height, CV_8UC1);
	memcpy (mapasimg.data, &map.data[0], map.info.width*map.info.height*sizeof(uchar));

	bool init=false;
	int minx=0,miny=0,maxx=0,maxy=0;
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

