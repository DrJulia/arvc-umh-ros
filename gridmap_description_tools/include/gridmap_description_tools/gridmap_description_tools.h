// ******************************************************************************
// This library is used for:
//
// 	1 - Description of gridmaps by means of a set of features and their descriptors
//				
//		Method:		find_features
//		Input: 		cv::Mat gridmap
//		Output:		std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors
//		Parameters:	SIFT or SURF features
//
//	2 - Data association between two maps described by step 1
//
//		Method:		dataAssociation
//		Input:		cv::Mat descriptors1, cv::Mat descriptors2
//		Output:		std::list<cv::DMatch> matches
//		Parameters:	
//
//	3 - Map alignment from a set of corresponding features obtained from step 2 
//
//		Method:		align_maps
//		Input:		std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, std::vector<cv::DMatch> matches
//		Output:		tf::Transform map1_to_map2
//		Parameters:	
//
// Auxiliar:
//
//	Convert a nav_msgs::OccupancyGrid into a opencv mat  
//
//		Method:		OccupancyGrid2CvMat
//		Input:		nav_msgs::OccupancyGrid
//		Output:		cv::Mat
//		Parameters:	value for unknown cells (255)
//
// ******************************************************************************

#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/features2d/features2d.hpp>
#include <nav_msgs/GetMap.h>

namespace gridmap_description{

	enum desc_type{SIFT_DESC, SURF_DESC};


	// main methods
	void find_features(const cv::Mat& gridmap, const cv::Mat& descmap, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, desc_type descriptor=SIFT_DESC, bool distanceTransform=true);

	void dataAssociation(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches, float disth, int knn);

	int align_maps(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch> matches, tf::Transform& map1_to_map2, cv::Mat& transform_covariance, int min_inliers, std::vector<cv::DMatch>& finalmatches);

	// auxiliar methods
	void OccupancyGrid2CvMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mapasimg, int unknown_value=0, int occupied_value=0, int free_value=255);

};
