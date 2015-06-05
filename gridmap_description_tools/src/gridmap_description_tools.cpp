
#include <gridmap_description_tools/gridmap_description_tools.h>
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
#include <exception>

using namespace cv;
using namespace std;

void gridmap_description::find_features(const cv::Mat& gridmap, const cv::Mat& descmap, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, desc_type descriptor, bool distanceTransform){

	// Distance Transform
	cv::Mat distance(gridmap.rows, gridmap.cols, CV_32FC1);
	cv::Mat distanceNorm(gridmap.rows, gridmap.cols, CV_8UC1);
	cv::distanceTransform(gridmap, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::normalize(distance, distanceNorm, 0.0, 255.0, cv::NORM_MINMAX,CV_8UC1);

	// prepare detector/extractor
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	switch(descriptor){
		case SIFT_DESC:
			detector = new cv::SiftFeatureDetector();
			extractor = new cv::SiftDescriptorExtractor();			
			break;
		case SURF_DESC:
			detector = new cv::SurfFeatureDetector();
			extractor = new cv::SurfDescriptorExtractor();			
			break;
	}

	// DETECT FEATURES
	if (distanceTransform)
		detector->detect(distanceNorm, keypoints);
	else		
		detector->detect(gridmap, keypoints);

	// Extract features
	//if (distanceTransform)
	//	extractor->compute(distanceNorm, keypoints, descriptors);
	//else
		extractor->compute(descmap, keypoints, descriptors);

	detector.release(); 
	extractor.release();
	distance.release();
	distanceNorm.release();

}

// TODO review
void gridmap_description::dataAssociation(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches, float disth, int knn){
	// NN matches and crosscheck 
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = new cv::FlannBasedMatcher();
	matches.clear();
	vector<vector<DMatch> > matches12, matches21;
	descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
	descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
	DMatch forward, backward;
	for( size_t m = 0; m < matches12.size(); m++ ){
		bool findCrossCheck = false;
		for( size_t fk = 0; fk < matches12[m].size(); fk++ ){
			forward = matches12[m][fk];
			for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ ){
				backward = matches21[forward.trainIdx][bk];
				if( backward.trainIdx == forward.queryIdx){
					//if(forward.distance < disth){
					    matches.push_back(forward);
					    findCrossCheck = true;
					    break;
					//}
				}
			}
			if( findCrossCheck ) break;
		}
	}
}

// TODO review
int gridmap_description::align_maps(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch> matches, tf::Transform& map1_to_map2, cv::Mat& transform_covariance, int min_inliers, std::vector<cv::DMatch>& finalmatches){
	
	if (matches.size()<2){
		map1_to_map2.setOrigin(tf::Point(-2000,0,0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(0));
//		ROS_ERROR("COULDN'T FIND MAP ALIGNMENT");
		return -1;
	}

	// THIS IS RANSAC
	double edth = 10; 	// euclidean distance threshold for the 2 points selected (in pixels) 
	double inlierth = 4;	// euclidean distance between the proyected point with the test tranform and the matched point (in pixels) 
	int inliers=0;
	double txx=0, tyy=0, thh=0;
	std::vector<uint> bestgoodmatches;
	KeyPoint p1m1, p1m2, p2m1, p2m2;
	float dA, dB, dC, dD;
	//float dAA, dBB, dCC, dDD;
	float th,tx,ty;
	int ntests = 0;
	int filteredByDist = 0;
	int filteredByError = 0;
	// RANDOM
//	for (int iter=0; iter < 100*matches.size(); iter++){

//		int p1i, p2i;
//	 	p1i = rand() % matches.size();
//		do{
//			p2i = rand() % matches.size();
//		} while (p1i==p2i);

	// BRUTE FORCE
	int p1i, p2i;
	for (p1i=0; p1i < matches.size(); p1i++){
	for (p2i=0; p2i < matches.size(); p2i++){
		if (p1i==p2i) continue;

		p1m1 = keypoints1[matches[p1i].queryIdx]; 
		p1m2 = keypoints2[matches[p1i].trainIdx];
		p2m1 = keypoints1[matches[p2i].queryIdx]; 
		p2m2 = keypoints2[matches[p2i].trainIdx];

		dA = p1m2.pt.x - p2m2.pt.x;
		dB = p1m2.pt.y - p2m2.pt.y;
		dC = p1m1.pt.x - p2m1.pt.x;
		dD = p1m1.pt.y - p2m1.pt.y;

		ntests++;

		if (sqrt(dA*dA+dB*dB) < edth){
			filteredByDist++;
			continue;
		}
 		if (sqrt(dA*dA+dB*dB)-sqrt(dC*dC-dD*dD)<inlierth){
			filteredByError++;
			continue;
		}
		
		th = atan2(dB*dC - dA*dD, dA*dC + dB*dD);
		tx = p1m1.pt.x - (p1m2.pt.x * cos(th) - p1m2.pt.y * sin(th)); 
		ty = p1m1.pt.y - (p1m2.pt.y * cos(th) + p1m2.pt.x * sin(th));

//		if ( fabs(tx) > 21 || fabs (ty) >21 ){
//			ROS_ERROR("p1m1: (%f,%f) => p1m2: (%f,%f) ", p1m1.pt.x, p1m1.pt.y, p1m2.pt.x, p1m2.pt.y);
//			ROS_ERROR("p2m1: (%f,%f) => p2m2: (%f,%f) ", p2m1.pt.x, p2m1.pt.y, p2m2.pt.x, p2m2.pt.y);
//		}

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
	}}

	ROS_INFO("After %d tests, %d filtered by distance, %d filtered by error, best alignment has %d inliers",ntests, filteredByDist, filteredByError, inliers);

//	THIS IS THE 2 POINTS TRANSFORM
//	ROS_INFO("2P TRANSFORM IS x = %f, y = %f, th = %f (inliers: %d)", txx, tyy, thh, inliers);

	if (inliers >= min_inliers){
//		// this would take the 2P transform:
		map1_to_map2.setOrigin(tf::Point(txx,tyy,0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(thh));		

		// But we are going to do Least Squares with all the inliers
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

//			ROS_INFO("inlier %d, distx=%f, disty=%f", i, keypoints1[matches[bestgoodmatches[i]].queryIdx].pt.x-keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.x,
//								     keypoints1[matches[bestgoodmatches[i]].queryIdx].pt.y-keypoints2[matches[bestgoodmatches[i]].trainIdx].pt.y);

		}
		//cv::Mat trans = ptsm2.inv(cv::DECOMP_SVD)*ptsm1;
		cv::Mat ptsm2T;
		cv::transpose(ptsm2, ptsm2T);
		cv::Mat sqinv = (ptsm2T*ptsm2).inv(cv::DECOMP_SVD);
		cv::Mat trans = sqinv*ptsm2T*ptsm1;
		float sigma2 = 4.0;///inliers; //pixel error
		cv::Mat covariance = sigma2 * sqinv; 
//		ROS_INFO("COV: \n [%e, %e, %e, %e] \n [%e, %e, %e, %e] \n [%e, %e, %e, %e]\n [%e, %e, %e, %e]",
//				 covariance.at<double>(0,0), covariance.at<double>(0,1),covariance.at<double>(0,2), covariance.at<double>(0,3),
//				 covariance.at<double>(1,0), covariance.at<double>(1,1),covariance.at<double>(1,2), covariance.at<double>(1,3),
//				 covariance.at<double>(2,0), covariance.at<double>(2,1),covariance.at<double>(2,2), covariance.at<double>(2,3),
//				 covariance.at<double>(3,0), covariance.at<double>(3,1),covariance.at<double>(3,2), covariance.at<double>(3,3));

		cv::Mat outcovariance(3, 3, CV_64FC1);  // xx     xy     xtheta
							// yx 	  yy     ytheta
							// thetax thetay thetatheta // theta is aprox sin(theta) since theta is small
		outcovariance.at<double>(0,0)=covariance.at<double>(2,2); outcovariance.at<double>(0,1)=covariance.at<double>(2,3); outcovariance.at<double>(0,2)=covariance.at<double>(2,1);
		outcovariance.at<double>(1,0)=covariance.at<double>(3,2); outcovariance.at<double>(1,1)=covariance.at<double>(3,3); outcovariance.at<double>(1,2)=covariance.at<double>(3,1);
		outcovariance.at<double>(2,0)=covariance.at<double>(1,2); outcovariance.at<double>(2,1)=covariance.at<double>(1,3); outcovariance.at<double>(2,2)=covariance.at<double>(1,1);

		//ROS_INFO("LS TRANSFORM IS x = %f, y = %f, th = %f (inliers: %d)", trans.at<double>(2,0), trans.at<double>(3,0), atan2(trans.at<double>(1,0),trans.at<double>(0,0)), inliers);
		//ROS_INFO("LS COVARIANCE IS sigma_x = %f, sigma_y = %f, sigma_th = %f", covariance.at<double>(2,2), covariance.at<double>(3,3), asin(covariance.at<double>(0,0)) );

		// LEAST SQUARES VALUES
//		map1_to_map2.setOrigin(tf::Point(trans.at<double>(2,0), trans.at<double>(3,0),0));
//		map1_to_map2.setRotation(tf::createQuaternionFromYaw(atan2(trans.at<double>(1,0),trans.at<double>(0,0))));

		transform_covariance = outcovariance;
	}
	else{
		// if no alignment found we put the new map far away with a high covariance
		map1_to_map2.setOrigin(tf::Point(-2000,0,0));
		map1_to_map2.setRotation(tf::createQuaternionFromYaw(0));
		//ROS_ERROR("COULDN'T FIND MAP ALIGNMENT");
		for (uint i=0; i<bestgoodmatches.size(); i++)
			finalmatches.push_back(matches[bestgoodmatches[i]]);
		return -1;
	}
	return inliers;
}



/// converts from occupancygrid message to opencv matrix
void gridmap_description::OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, int unknown_value, int occupied_value, int free_value){
	// convert map to image
	mapasimg.create(map.info.height, map.info.width, CV_8UC1);
	for (uint i=0; i<map.info.width*map.info.height; i++){
		if(map.data[i]==-1) mapasimg.data[i]=unknown_value; 
		else{
			if(map.data[i]<=50) mapasimg.data[i]=free_value;	
			else mapasimg.data[i]=occupied_value;
		}
	}
}


