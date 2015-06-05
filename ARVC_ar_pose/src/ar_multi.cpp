/*
 *  Multi Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ARVC_ar_pose/ar_multi.h"
#include "ARVC_ar_pose/object.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_single");
  ros::NodeHandle n;
  ar_pose::ARSinglePublisher ar_single (n);
  ros::spin ();
  return 0;
}

namespace ar_pose
{
  typedef double mytransmat[3][4] ;

  ARSinglePublisher::ARSinglePublisher (ros::NodeHandle & n):n_ (n), it_ (n_)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
	std::string default_path = "data/object_4x4";
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    // **** get parameters

    if (!n_param.getParam ("publish_tf", publishTf_))
      publishTf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    if (!n_param.getParam ("publish_visual_markers", publishVisualMarkers_))
      publishVisualMarkers_ = true;
    ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

    if (!n_param.getParam ("threshold", threshold_))
      threshold_ = 70;
    ROS_INFO ("\tThreshold: %d", threshold_);

    if (!n_param.getParam ("min_confidence", min_confidence_))
      min_confidence_ = 75;
    ROS_INFO ("\tMin confidence: %d", min_confidence_);

    if (!n_param.getParam("use_history", useHistory_))
      useHistory_ = false;
    ROS_INFO("\tUse history: %d", useHistory_);
	
    if (!n_param.getParam("use_dynamic_threshold", useDynamicThreshold_))
      useDynamicThreshold_ = true;
    ROS_INFO("\tUse dynamic threshold: %d", useDynamicThreshold_);

	//modifications to allow path list from outside the package
	n_param.param ("marker_pattern_list", local_path, default_path);
	if (local_path.compare(0,5,"data/") == 0){
	  //according to previous implementations, check if first 5 chars equal "data/"
	  sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
	}
	else{
	  //for new implementations, can pass a path outside the package_path
	  sprintf (pattern_filename_, "%s", local_path.c_str ());
	}
	ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);

    if (!n_param.getParam("pixelerror", pixelerror_))
      pixelerror_ = 5.0;
    ROS_INFO("\tpixelerror: %f", pixelerror_);

    if (!n_param.getParam("caliberror", caliberror_))
      caliberror_ = 20.0;
    ROS_INFO("\tcaliberror: %f", caliberror_);

    if (!n_param.getParam("scaleerror", scaleerror_))
      scaleerror_ = 7.0;
    ROS_INFO("\tscaleerror: %f", scaleerror_);
	
    if (!n_param.getParam("alpha", alpha_))
      alpha_ = 1.0;
    ROS_INFO("\talpha: %f", alpha_);

    if (!n_param.getParam("beta", beta_))
      beta_ = 2.0;
    ROS_INFO("\tbeta: %f", beta_);

    if (!n_param.getParam("kappa", kappa_))
      kappa_ = 0.0;
    ROS_INFO("\tkappa: %f", kappa_);

    // **** subscribe

    ROS_INFO ("Subscribing to info topic");
    sub_ = n_.subscribe (cameraInfoTopic_, 1, &ARSinglePublisher::camInfoCallback, this);
    getCamInfo_ = false;

    // **** advertsie 

    arMarkerPub_ = n_.advertise < ar_pose::ARMarkers > ("ar_pose_marker", 0);
    if(publishVisualMarkers_)
    {
		rvizMarkerPub_ = n_.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	 }
  }

  ARSinglePublisher::~ARSinglePublisher (void)
  {
    //cvReleaseImage(&capture_); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }

  void ARSinglePublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!getCamInfo_)
    {
      cam_info_ = (*cam_info);

      cam_param_.xsize = cam_info_.width;
      cam_param_.ysize = cam_info_.height;

      cam_param_.mat[0][0] = cam_info_.P[0];
      cam_param_.mat[1][0] = cam_info_.P[4];
      cam_param_.mat[2][0] = cam_info_.P[8];
      cam_param_.mat[0][1] = cam_info_.P[1];
      cam_param_.mat[1][1] = cam_info_.P[5];
      cam_param_.mat[2][1] = cam_info_.P[9];
      cam_param_.mat[0][2] = cam_info_.P[2];
      cam_param_.mat[1][2] = cam_info_.P[6];
      cam_param_.mat[2][2] = cam_info_.P[10];
      cam_param_.mat[0][3] = cam_info_.P[3];
      cam_param_.mat[1][3] = cam_info_.P[7];
      cam_param_.mat[2][3] = cam_info_.P[11];

//      cam_param_.mat[0][0] = cam_info_.K[0];
//      cam_param_.mat[1][0] = cam_info_.K[3];
//      cam_param_.mat[2][0] = cam_info_.K[6];
//      cam_param_.mat[0][1] = cam_info_.K[1];
//      cam_param_.mat[1][1] = cam_info_.K[4];
//      cam_param_.mat[2][1] = cam_info_.K[7];
//      cam_param_.mat[0][2] = cam_info_.K[2];
//      cam_param_.mat[1][2] = cam_info_.K[5];
//      cam_param_.mat[2][2] = cam_info_.K[8];
//      cam_param_.mat[0][3] = 0.0;
//      cam_param_.mat[1][3] = 0.0;
//      cam_param_.mat[2][3] = 0.0;

//      cam_param_.dist_factor[0] = cam_info_.P[2];       // x0 = cX from openCV calibration
//      cam_param_.dist_factor[1] = cam_info_.P[6];       // y0 = cY from openCV calibration
      cam_param_.dist_factor[0] = cam_info_.K[2];       // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = cam_info_.K[5];       // y0 = cY from openCV calibration
      cam_param_.dist_factor[2] = 0; //-100*cam_info_.D[0];  	// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...
      
      arInit ();

      ROS_INFO ("Subscribing to image topic");
      cam_sub_ = it_.subscribe (cameraImageTopic_, 1, &ARSinglePublisher::getTransformationCallback, this);
      getCamInfo_ = true;
    }
  }

  void ARSinglePublisher::arInit ()
  {
    arInitCparam (&cam_param_);
    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load in the object data - trained markers and associated bitmap files
    if ((object = ar_object::read_ObjData (pattern_filename_, &objectnum)) == NULL)
      ROS_BREAK ();
    ROS_DEBUG ("Objectfile num = %d", objectnum);

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
  }

double  ARSinglePublisher::getDynamicThreshold(const ARUint8* dataPtr, const ARMarkerInfo* marker_info, const int marker_num){

	// we use kmeans to get the threshold
	
	cv::Mat thresimg = cv::Mat::zeros(cam_param_.ysize, cam_param_.xsize, CV_8UC1);

	// first we find the mean to do a good initialization
	double sum=0;
	double counter=0;
	printf("num markers: %d\n", marker_num);

	if (marker_num==0) return 70.0;

	for (int m=0; m < marker_num; m++){
		printf("marker: %d (cf=%f), dir = %d\n", m, marker_info[m].cf, marker_info[m].dir);
		
//		printf("corners: (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",	marker_info[m].vertex[0][0], marker_info[m].vertex[0][1], 
//									marker_info[m].vertex[1][0], marker_info[m].vertex[1][1], 
//									marker_info[m].vertex[2][0], marker_info[m].vertex[2][1], 
//									marker_info[m].vertex[3][0], marker_info[m].vertex[3][1]);

		//if (round(marker_info[m].cf * 100) < min_confidence_) continue;

/*
		int xmin = (marker_info[m].vertex[0][0] < marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0] : marker_info[m].vertex[1][0];
		xmin = (xmin < marker_info[m].vertex[2][0])? xmin : marker_info[m].vertex[2][0];
		xmin = (xmin < marker_info[m].vertex[3][0])? xmin : marker_info[m].vertex[3][0];

		int xmax = (marker_info[m].vertex[0][0] > marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
		xmax = (xmax > marker_info[m].vertex[2][0])? xmax : marker_info[m].vertex[2][0];
		xmax = (xmax > marker_info[m].vertex[3][0])? xmax : marker_info[m].vertex[3][0];

		int ymin = (marker_info[m].vertex[0][0] < marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
		ymin = (ymin < marker_info[m].vertex[2][0])? ymin : marker_info[m].vertex[2][0];
		ymin = (ymin < marker_info[m].vertex[3][0])? ymin : marker_info[m].vertex[3][0];

		int ymax = (marker_info[m].vertex[0][0] > marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
		ymax = (ymax > marker_info[m].vertex[2][0])? ymax : marker_info[m].vertex[2][0];
		ymax = (ymax > marker_info[m].vertex[3][0])? ymax : marker_info[m].vertex[3][0];

		for(int x = xmin; x <= xmax; x++){
			for(int y = ymin; y <= ymax; y++){
				sum+= dataPtr[3*y*cam_param_.xsize+3*x]; //dataPtr[x+y*cam_param_.xsize];
			//	printf("%d, ",dataPtr[x+y*cam_param_.xsize]);
				counter++;
			}
			
		}*/
	}
/*	if (counter<=0) return 70.0;
	double mean = sum/counter;
	printf("mean: %f\n", mean);
	double blackcenter = mean - 15;
	double whitecenter = mean + 15;

	for (int step=0; step < 5; step++){
		double blacksum = 0; 
		double whitesum = 0;		
		double blackcounter = 0;
		double whitecounter = 0;


		for (int m=0; m < marker_num; m++){
			if (round(marker_info[m].cf * 100) < min_confidence_) continue;

			int xmin = (marker_info[m].vertex[0][0] < marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0] : marker_info[m].vertex[1][0];
			xmin = (xmin < marker_info[m].vertex[2][0])? xmin : marker_info[m].vertex[2][0];
			xmin = (xmin < marker_info[m].vertex[3][0])? xmin : marker_info[m].vertex[3][0];

			int xmax = (marker_info[m].vertex[0][0] > marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
			xmax = (xmax > marker_info[m].vertex[2][0])? xmax : marker_info[m].vertex[2][0];
			xmax = (xmax > marker_info[m].vertex[3][0])? xmax : marker_info[m].vertex[3][0];

			int ymin = (marker_info[m].vertex[0][0] < marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
			ymin = (ymin < marker_info[m].vertex[2][0])? ymin : marker_info[m].vertex[2][0];
			ymin = (ymin < marker_info[m].vertex[3][0])? ymin : marker_info[m].vertex[3][0];

			int ymax = (marker_info[m].vertex[0][0] > marker_info[m].vertex[1][0])? marker_info[m].vertex[0][0]:marker_info[m].vertex[1][0];
			ymax = (ymax > marker_info[m].vertex[2][0])? ymax : marker_info[m].vertex[2][0];
			ymax = (ymax > marker_info[m].vertex[3][0])? ymax : marker_info[m].vertex[3][0];

			for(int x = xmin; x <= xmax; x++){
				for(int y = ymin; y <= ymax; y++){
					if (fabs (blackcenter-dataPtr[3*x+3*y*cam_param_.xsize]) < fabs (whitecenter-dataPtr[3*x+3*y*cam_param_.xsize])){
						blacksum += dataPtr[3*y*cam_param_.xsize+3*x] ;//dataPtr[x+y*cam_param_.xsize];
						blackcounter++;
					}
					else{
						whitesum += dataPtr[3*y*cam_param_.xsize+3*x]; //dataPtr[x+y*cam_param_.xsize];
						whitecounter++;
					}
				}
			}
		}
		blackcenter = blacksum/blackcounter;
		whitecenter = whitesum/whitecounter;
	}
	printf("clustering result: %f, %f\n",blackcenter,whitecenter);
*/

	double threshold =  70; //(whitecenter+blackcenter)/2.0;

//	printf("size %d x %d\n", cam_param_.xsize, cam_param_.ysize);

	for (int px = 0 ; px< cam_param_.xsize; px++){ // 0 - 639
		for (int py = 0 ; py< cam_param_.ysize; py++){ // 0 - 479
//			thresimg.at<uchar>(py,px) = dataPtr[3*py*cam_param_.xsize+3*px];
			if ( dataPtr[3*py*cam_param_.xsize+3*px] > threshold	)
				thresimg.at<uchar>(py,px) = 255;
			else
				thresimg.at<uchar>(py,px) = 0;
		}
	}
	cv::namedWindow( "thres", 0 );
	cv::imshow( "thres", thresimg );
	cv::waitKey(5);

	return threshold;
}

  void ARSinglePublisher::getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg)
  {
    ROS_INFO("image received");
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k, j;

    /* Get the image from ROSTOPIC
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the 
     * ARToolKit configure option (see config.h).*/
    try
    {
     	 capture_ = bridge_.imgMsgToCv (image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException & e)
    {
     	 ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str ());
    }
    //cvConvertImage(capture,capture,CV_CVTIMG_FLIP);
    dataPtr = (ARUint8 *) capture_->imageData;

    // detect the markers in the video frame

    if (useDynamicThreshold_){
	    //threshold_ = 127;
	    bool signpositive = false;
	    int increment = 5;
	    int nextinc = increment;
	    while (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
	    {
		if (signpositive){
			threshold_ += nextinc;
		}
		else{
			threshold_ -= nextinc;
		}
		nextinc += increment;
		if (nextinc > 80 ){
			threshold_ = 70;
			argCleanup ();
			ROS_BREAK ();
		}
	    }
	    threshold_ = getDynamicThreshold(dataPtr, marker_info, marker_num);
	    printf("Dynamic Threshold: %d\n", threshold_);

    }
    else{
    	    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0){
		argCleanup ();
		ROS_BREAK ();
 	    }
    }

    arPoseMarkers_.markers.clear ();
    arPoseMarkers_.header.frame_id = image_msg->header.frame_id;
    arPoseMarkers_.header.stamp = image_msg->header.stamp;

////////////////////////////////////////////////////////////////////////////////////////////
//
//	Spherical Symetric Unscented Transform (SSUT) is applied to get the mean and covariance of the transform
//
////////////////////////////////////////////////////////////////////////////////////////////


	      // Spherical Symetric Unscented Transform (SSUT) is applied now
//	      const int ndim = 8;
//	      int nsigmapoints = ndim + 2;		// 10 sigma points (n + 2) being n = 8 (4 corners x 2 dimensions)
//	      double mu = 0.01;				// = alpha^2 ; alpha = 0.1; // [0 < alpha < 1];	     
//	      double W0m = 0.991;
//	      double W0c = W0m;
//	      double W1 = (1-W0m)/(ndim+1);
//	      double sigmapoint[nsigmapoints][ndim];

//	      // incrementally build the sigma point set with mean 0 and covariance I
//	      for (int jj=1; jj <= ndim; jj++){
//			for (int ii = 0; ii <=jj+1; ii++){
//				if (ii==0)
//					sigmapoint[ii][jj-1] = 0;
//				else if (ii<jj+1) sigmapoint[ii][jj-1] = -1/sqrt(jj*(jj+1)*W1);
//				else{
//					for (int jjaux=1; jjaux<jj; jjaux++) 
//						sigmapoint[ii][jjaux-1] = 0;
//					sigmapoint[ii][jj-1] = jj/sqrt(jj*(jj+1)*W1);
//				}
//			}
//	      }
	      /////////////////////////////////////////////////////////////////////////////////////
	      // Sigma points for the normal Unscented Transform
	      const int ndim = 11;
	      const int nsigmapoints = 2*ndim + 1;		// 2L+1

	      const double lambda = pow(alpha_,2)*(ndim+kappa_) - ndim;
	      const double W0m = lambda/(ndim+lambda);
	      const double W0c = W0m+(1-pow(alpha_,2)+beta_);
	      const double W1 = 1/(2*(ndim+lambda));

	      double sigmapoint[nsigmapoints][ndim];

	      for (int ii=0; ii < nsigmapoints; ii++){
			if (ii==0){
				for (int jj=0; jj < ndim; jj++){
					sigmapoint[ii][jj] = 0;
				}
			}
			else if (ii <= ndim){
				for (int jj=0; jj < ndim; jj++){
					if (ii==jj+1) sigmapoint[ii][jj] = sqrt(ndim+lambda);
					else sigmapoint[ii][jj] = 0;
				}
			}
			else {
				for (int jj=0; jj < ndim; jj++){
					if (ii-ndim==jj+1) sigmapoint[ii][jj] = -sqrt(ndim+lambda);
					else sigmapoint[ii][jj] = 0;
				}
		      	}
	      }

    // check for known patterns
    for (i = 0; i < objectnum; i++)
    {
	      k = -1;
	      for (j = 0; j < marker_num; j++)
	      {
		if (object[i].id == marker_info[j].id)
		{
		  if (k == -1)
		    k = j;
		  else                  // make sure you have the best pattern (highest confidence factor)
		  if (marker_info[k].cf < marker_info[j].cf)
		    k = j;
		}
	      }
	      if (k == -1)
	      {
		object[i].visible = 0;
		continue;
	      }
	      if (round(marker_info[k].cf * 100) < min_confidence_) continue;

	      ////////////////////////////////////////////////////////////////////////////////////

	      // move the sigma points to the mean and put them in the marker info struct
	      ARMarkerInfo sp[nsigmapoints];
	      double widths[nsigmapoints];
	      for (int kk=0; kk < nsigmapoints; kk++){
		      sp[kk] = marker_info[k];
		      widths[kk] = object[i].marker_width + scaleerror_*sigmapoint[kk][10];
		      for (int vv=0; vv<4; vv++) { // the 8 pixel dimension
				sp[kk].vertex[vv][0] = marker_info[k].vertex[vv][0] + pixelerror_*sigmapoint[kk][2*vv]   + caliberror_*sigmapoint[kk][8];
				sp[kk].vertex[vv][1] = marker_info[k].vertex[vv][1] + pixelerror_*sigmapoint[kk][2*vv+1] + caliberror_*sigmapoint[kk][9];
		      }

//		      printf ("sigma point zero mean %d: (%f,%f), (%f,%f), (%f,%f), (%f,%f) \n",kk, sigmapoint[kk][0],sigmapoint[kk][1],sigmapoint[kk][2],sigmapoint[kk][3],
//												  sigmapoint[kk][4],sigmapoint[kk][5],sigmapoint[kk][6],sigmapoint[kk][7] );
//	
//		      printf ("sigma point traslated %d: (%f,%f), (%f,%f), (%f,%f), (%f,%f)\n",kk,sp[kk].vertex[0][0],sp[kk].vertex[0][1],
//											sp[kk].vertex[1][0],sp[kk].vertex[1][1],
//											sp[kk].vertex[2][0],sp[kk].vertex[2][1],
//											sp[kk].vertex[3][0],sp[kk].vertex[3][1]);
	      }

	      // call artoolkit to get the transforms for each sigma point
     	      double markertrans[nsigmapoints][3][4];
	      double res;
	      for (int kk=0; kk < nsigmapoints; kk++){
//		      if (object[i].visible == 0 || !useHistory_){
//				printf (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<>>>>>>>>>>>>>>>sigma point %d\n",kk);
				res=arGetTransMat (&sp[kk], object[i].marker_center, widths[kk], markertrans[kk]);
//		      }
//		      else{
//				res=arGetTransMatCont (&sp[kk], object[i].trans, object[i].marker_center, widths[kk], markertrans[kk]);
//		      }
		      //printf("error %f\n",res);
//		      if (markertrans[kk][2][3] < 0){
//			      printf("point y %d> %f, %f, %f, %f \n",kk, markertrans[kk][0][0],markertrans[kk][0][1],markertrans[kk][0][2],markertrans[kk][0][3]);
//			      printf("            %f, %f, %f, %f \n",    markertrans[kk][1][0],markertrans[kk][1][1],markertrans[kk][1][2],markertrans[kk][1][3]);
//			      printf("            %f, %f, %f, %f \n",    markertrans[kk][2][0],markertrans[kk][2][1],markertrans[kk][2][2],markertrans[kk][2][3]);
//			      exit;
//		      }
	      }
	      if (res<0) continue;
	      // get the mean of the transforms
	      double y[nsigmapoints][6];
	      double mean[6];
	      double meanangles[3][2];
	      
//	      double spquat[4];
//	      arUtilMat2QuatPos (markertrans[0], spquat, y[0]);
//	      btMatrix3x3(btQuaternion(-spquat[0],-spquat[1],-spquat[2],spquat[3])).getRPY(y[0][3], y[0][4], y[0][5]);
	      btMatrix3x3 rotmat0(markertrans[0][0][0],markertrans[0][0][1],markertrans[0][0][2],
				  markertrans[0][1][0],markertrans[0][1][1],markertrans[0][1][2],
				  markertrans[0][2][0],markertrans[0][2][1],markertrans[0][2][2]);
	      y[0][0]= markertrans[0][0][3];
	      y[0][1]= markertrans[0][1][3];
	      y[0][2]= markertrans[0][2][3];	      
	      // get euler angles
	      rotmat0.getEulerYPR(y[0][5], y[0][4], y[0][3]);

	      y[0][0]*= AR_TO_ROS;
	      y[0][1]*= AR_TO_ROS;
	      y[0][2]*= AR_TO_ROS;
	      mean[0] = W0m*y[0][0];
	      mean[1] = W0m*y[0][1];
	      mean[2] = W0m*y[0][2];
	      meanangles[0][0] = W0m*cos(y[0][3]);
	      meanangles[0][1] = W0m*sin(y[0][3]);
	      meanangles[1][0] = W0m*cos(y[0][4]);
	      meanangles[1][1] = W0m*sin(y[0][4]);
	      meanangles[2][0] = W0m*cos(y[0][5]);
	      meanangles[2][1] = W0m*sin(y[0][5]);
	      
	      printf("point y 0> x: %f, y: %f, z: %f, r: %f, p: %f, y: %f\n", y[0][0],y[0][1],y[0][2],y[0][3],y[0][4],y[0][5]);
//	      printf("point y %d> %f, %f, %f, %f \n", 0, markertrans[0][0][0],markertrans[0][0][1],markertrans[0][0][2],markertrans[0	][0][3]*AR_TO_ROS);
//	      printf("            %f, %f, %f, %f \n",    markertrans[0][1][0],markertrans[0][1][1],markertrans[0][1][2],markertrans[0][1][3]*AR_TO_ROS);
//	      printf("            %f, %f, %f, %f \n",    markertrans[0][2][0],markertrans[0][2][1],markertrans[0][2][2],markertrans[0][2][3]*AR_TO_ROS);
//	      
	      for (int kk=1; kk < nsigmapoints; kk++){

//		      arUtilMat2QuatPos (markertrans[kk], spquat, y[kk]);	      
//		      btMatrix3x3(btQuaternion(-spquat[0],-spquat[1],-spquat[2],spquat[3])).getRPY(y[kk][3], y[kk][4], y[kk][5]);

		      btMatrix3x3 rotmat( markertrans[kk][0][0],markertrans[kk][0][1],markertrans[kk][0][2],
					  markertrans[kk][1][0],markertrans[kk][1][1],markertrans[kk][1][2],
					  markertrans[kk][2][0],markertrans[kk][2][1],markertrans[kk][2][2]);
		      y[kk][0]= markertrans[kk][0][3];
		      y[kk][1]= markertrans[kk][1][3];
		      y[kk][2]= markertrans[kk][2][3];	      
		      // get euler angles
		      rotmat.getEulerYPR(y[kk][5], y[kk][4], y[kk][3]);

		      y[kk][0]*= AR_TO_ROS;
		      y[kk][1]*= AR_TO_ROS;
		      y[kk][2]*= AR_TO_ROS;
		      mean[0] += W1*y[kk][0];
		      mean[1] += W1*y[kk][1];
		      mean[2] += W1*y[kk][2];
		      meanangles[0][0] += W1*cos(y[kk][3]);
		      meanangles[0][1] += W1*sin(y[kk][3]);
		      meanangles[1][0] += W1*cos(y[kk][4]);
		      meanangles[1][1] += W1*sin(y[kk][4]);
		      meanangles[2][0] += W1*cos(y[kk][5]);
		      meanangles[2][1] += W1*sin(y[kk][5]);
		      printf("point y %d> x: %f, y: %f, z: %f, r: %f, p: %f, y: %f\n", kk, y[kk][0],y[kk][1],y[kk][2],y[kk][3],y[kk][4],y[kk][5]);
//		      printf("point y %d> %f, %f, %f, %f \n",kk, markertrans[kk][0][0],markertrans[kk][0][1],markertrans[kk][0][2],markertrans[kk][0][3]*AR_TO_ROS);
//		      printf("            %f, %f, %f, %f \n",    markertrans[kk][1][0],markertrans[kk][1][1],markertrans[kk][1][2],markertrans[kk][1][3]*AR_TO_ROS);
//		      printf("            %f, %f, %f, %f \n",    markertrans[kk][2][0],markertrans[kk][2][1],markertrans[kk][2][2],markertrans[kk][2][3]*AR_TO_ROS);
//		      if (markertrans[kk][2][3] < 0){
//			      exit(0);
//		      }

	      }
	      mean[3] = atan2( meanangles[0][1], meanangles[0][0] );
	      mean[4] = atan2( meanangles[1][1], meanangles[1][0] );
	      mean[5] = atan2( meanangles[2][1], meanangles[2][0] );
	      
	      printf("mean> x: %f, y: %f, z: %f, r: %f, p: %f, y: %f\n",mean[0],mean[1],mean[2],mean[3],mean[4],mean[5]);
	      
	      // get the covariance of the transform
	      double cov[36];
	      
	      for (int d1=0; d1<6; d1++){
 		      for (int d2=0; d2<6; d2++){
//				cov[6*d1+d2] += (W0c+1-mu)* (y[0][d1]-mean[d1]) * (y[0][d2]-mean[d2]) ;
				double aux1, aux2;
				aux1 = y[0][d1]-mean[d1];
				aux2 = y[0][d2]-mean[d2];
				if (d1>2){
					if (aux1 >= M_PI) aux1-= 2*M_PI;
					else if (aux1 < -M_PI) aux1+= 2*M_PI;
				}
				if (d2>2){
					if (aux2 >= M_PI) aux2-= 2*M_PI;
					else if (aux2 < -M_PI) aux2+= 2*M_PI;
				}
				cov[6*d1+d2] = W0c * aux1 * aux2 ;
		      }
	      }
	      for (int d1=0; d1<6; d1++){
			for (int d2=0; d2<6; d2++){
	      			for (int kk=1; kk < nsigmapoints; kk++){
					double aux1, aux2;
					aux1 = y[kk][d1]-mean[d1];
					aux2 = y[kk][d2]-mean[d2];
					if (d1>2){
						if (aux1 >= M_PI) aux1-= 2*M_PI;
						else if (aux1 < -M_PI) aux1+= 2*M_PI;
					}
					if (d2>2){
						if (aux2 >= M_PI) aux2-= 2*M_PI;
						else if (aux2 < -M_PI) aux2+= 2*M_PI;
					}
					cov[6*d1+d2] += W1 * aux1 * aux2 ;
				}
			}	
	      }

	      object[i].visible = 1;
	
	      double quat[4], pos[3];

	      pos[0] = mean[0];
	      pos[1] = mean[1];
	      pos[2] = mean[2];
	      
	      btQuaternion rotquat(mean[5], mean[4], mean[3]);
	      
	      quat[0] = rotquat.getX();
	      quat[1] = rotquat.getY();
	      quat[2] = rotquat.getZ();
	      quat[3] = rotquat.getW();
	      
////////////////////////////////////////////////////////////////////////////////////////////
//
//	Uncomment this to restore previous version
//
////////////////////////////////////////////////////////////////////////////////////////////

//	      // calculate the transform for each marker
//	      if (object[i].visible == 0 || !useHistory_)
//	      {
//		arGetTransMat (&marker_info[k], object[i].marker_center, object[i].marker_width, object[i].trans);
//	      }
//	      else
//	      {
//		arGetTransMatCont (&marker_info[k], object[i].trans,
//		                   object[i].marker_center, object[i].marker_width, object[i].trans);
//	      }


//	      object[i].visible = 1;

//	      double arQuat[4], arPos[3];

//	      //arUtilMatInv (object[i].trans, cam_trans);
//	      arUtilMat2QuatPos (object[i].trans, arQuat, arPos);

//	      // **** convert to ROS frame

//	      double quat[4], pos[3];

//	      pos[0] = arPos[0] * AR_TO_ROS;
//	      pos[1] = arPos[1] * AR_TO_ROS;
//	      pos[2] = arPos[2] * AR_TO_ROS;

//	      quat[0] = -arQuat[0];
//	      quat[1] = -arQuat[1];
//	      quat[2] = -arQuat[2];
//	      quat[3] = arQuat[3];

////////////////////////////////////////////////////////////////////////////////////////////

//	      ROS_INFO (" Object num %i ------------------------------------------------", i);
//	      ROS_DEBUG (" QUAT: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
//	      ROS_DEBUG ("     Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);

	      // **** publish the marker

	      ar_pose::ARMarker ar_pose_marker;
	      ar_pose_marker.header.frame_id = image_msg->header.frame_id;
	      ar_pose_marker.header.stamp = image_msg->header.stamp;
	      ar_pose_marker.id = object[i].id;

	      ar_pose_marker.pose.pose.position.x = pos[0];
	      ar_pose_marker.pose.pose.position.y = pos[1];
	      ar_pose_marker.pose.pose.position.z = pos[2];

	      ar_pose_marker.pose.pose.orientation.x = quat[0];
	      ar_pose_marker.pose.pose.orientation.y = quat[1];
	      ar_pose_marker.pose.pose.orientation.z = quat[2];
	      ar_pose_marker.pose.pose.orientation.w = quat[3];

////////////////////////////////////////////////////////////////////////////////////////////
//
//	      We add here the covariance to the message
//
////////////////////////////////////////////////////////////////////////////////////////////

//	      printf("covar=\n");
	      for (int ind=0; ind<6; ind++){
		      for (int ind2=0; ind2<6; ind2++){
				 ar_pose_marker.pose.covariance[ind*6+ind2] = cov[ind*6+ind2];
				 printf("%f, ", cov[ind*6+ind2]);
		      }
		      printf("\n");
	      }
	      printf("\n");
////////////////////////////////////////////////////////////////////////////////////////////

	      ar_pose_marker.confidence = round(marker_info[k].cf * 100);
	      arPoseMarkers_.markers.push_back (ar_pose_marker);

	      // **** publish transform between camera and marker

	      btQuaternion rotation (quat[0], quat[1], quat[2], quat[3]);
	      btVector3 origin (pos[0], pos[1], pos[2]);
	      btTransform t (rotation, origin);

	      if (publishTf_){

			tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, object[i].name);
			broadcaster_.sendTransform(camToMarker);
	      }
		
	      // **** publish visual marker
	      
	      if (publishVisualMarkers_){
			
//			btVector3 markerOrigin (0, 0, 0.25 * object[i].marker_width * AR_TO_ROS);
//			btTransform m (btQuaternion::getIdentity (), markerOrigin);
//			btTransform markerPose = t * m; // marker pose in the camera frame
//			tf::poseTFToMsg (markerPose, rvizMarker_.pose);
			
			btVector3 markerOrigin (0, 0, 0);
			btVector3 markerPose = t * markerOrigin; // marker pose in the camera frame
			tf::pointTFToMsg (markerPose, rvizMarker_.pose.position);
			
			rvizMarker_.header.frame_id = image_msg->header.frame_id;
			rvizMarker_.header.stamp = image_msg->header.stamp;
			rvizMarker_.id = object[i].id;
			rvizMarker_.action = visualization_msgs::Marker::ADD;
			
			rvizMarker2_ = rvizMarker3_ = rvizMarker_;

//			rvizMarker_.scale.x = 1.0 * object[i].marker_width * AR_TO_ROS;
//			rvizMarker_.scale.y = 1.0 * object[i].marker_width * AR_TO_ROS;
//			rvizMarker_.scale.z = 0.5 * object[i].marker_width * AR_TO_ROS;
//			rvizMarker_.type = visualization_msgs::Marker::CUBE;
			
			rvizMarker_.type = visualization_msgs::Marker::SPHERE;
			rvizMarker2_.type = visualization_msgs::Marker::ARROW;
			rvizMarker3_.type = visualization_msgs::Marker::TRIANGLE_LIST;

			cv::Mat poscovar = cv::Mat::zeros(3,3,CV_32FC1);
			poscovar.at<float>(0,0)=cov[0];
			poscovar.at<float>(0,1)=cov[1];
			poscovar.at<float>(0,2)=cov[2];
			poscovar.at<float>(1,0)=cov[6];
			poscovar.at<float>(1,1)=cov[7];
			poscovar.at<float>(1,2)=cov[8];
			poscovar.at<float>(2,0)=cov[12];
			poscovar.at<float>(2,1)=cov[13];
			poscovar.at<float>(2,2)=cov[14];
//			printf("%f, %f, %f \n", cov[0], cov[1], cov[2]);
//			printf("%f, %f, %f \n", cov[6], cov[7], cov[8]);
//			printf("%f, %f, %f \n", cov[12], cov[13], cov[14]);
//			printf("\n");
			cv::Mat eigenvalues, eigenvectors;
			cv::eigen(poscovar,eigenvalues,eigenvectors);
			
//			printf("vector 0 %f, %f, %f\n",eigenvectors.at<float>(0,0),eigenvectors.at<float>(0,1),eigenvectors.at<float>(0,2));			
//			printf("vector 1 %f, %f, %f\n",eigenvectors.at<float>(1,0),eigenvectors.at<float>(1,1),eigenvectors.at<float>(1,2));			
//			printf("vector 2 %f, %f, %f\n",eigenvectors.at<float>(2,0),eigenvectors.at<float>(2,1),eigenvectors.at<float>(2,2));			

			rvizMarker_.scale.x = 3*sqrt(eigenvalues.at<float>(0,0));
			rvizMarker_.scale.y = 3*sqrt(eigenvalues.at<float>(0,1));
			rvizMarker_.scale.z = 3*sqrt(eigenvalues.at<float>(0,2));

			rvizMarker2_.scale.x = 1;
			rvizMarker2_.scale.y = 1;
			rvizMarker2_.scale.z = 1;

			rvizMarker3_.scale.x = 1;
			rvizMarker3_.scale.y = 1;
			rvizMarker3_.scale.z = 1;
			
			geometry_msgs::Point p;
			p.x = p.y = p.z = 0;
			rvizMarker3_.points.push_back(p);
			p.x = 0.5*cos(3*sqrt(cov[28])); p.z = -0.5*sin(3*sqrt(cov[28]));
			rvizMarker3_.points.push_back(p);
			p.x = 0.5*cos(3*sqrt(cov[28])); p.z = 0.5*sin(3*sqrt(cov[28]));
			rvizMarker3_.points.push_back(p);
			
			printf("error angle: %f\n", 3*sqrt(cov[28]));			
			
			btMatrix3x3 rotmat(eigenvectors.at<float>(0,0),eigenvectors.at<float>(1,0),eigenvectors.at<float>(2,0),
					   eigenvectors.at<float>(0,1),eigenvectors.at<float>(1,1),eigenvectors.at<float>(2,1),
					   eigenvectors.at<float>(0,2),eigenvectors.at<float>(1,2),eigenvectors.at<float>(2,2));
			// get euler angles
			btScalar roll, pitch, yaw;
			rotmat.getEulerYPR(yaw,pitch,roll);
//			printf ("rvizmarkers: roll %f, pitch %f, yaw %f\n", roll, pitch, yaw);

			rvizMarker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
//			rvizMarker2_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
			tf::quaternionTFToMsg(rotation, rvizMarker2_.pose.orientation);
			rvizMarker3_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, mean[4], 0);

//			printf("scale %f, %f, %f\n",rvizMarker_.scale.x, rvizMarker_.scale.y, rvizMarker_.scale.z);
//			printf("angles %f, %f, %f\n",roll, pitch, yaw);

			rvizMarker_.ns = "ar_markers";
			rvizMarker_.color.r = 0.0f;
			rvizMarker_.color.g = 1.0f;
			rvizMarker_.color.b = 0.0f;
			rvizMarker_.color.a = 1.0f;
			rvizMarker2_.ns = "ar_markers_arrows";
			rvizMarker2_.color.r = 0.0f;
			rvizMarker2_.color.g = 0.0f;
			rvizMarker2_.color.b = 1.0f;
			rvizMarker2_.color.a = 1.0f;
			rvizMarker3_.ns = "ar_markers_triangles";
			rvizMarker3_.color.r = 0.0f;
			rvizMarker3_.color.g = 0.0f;
			rvizMarker3_.color.b = 1.0f;
			rvizMarker3_.color.a = 1.0f;
			
			rvizMarker_.lifetime = ros::Duration (0.5);
			rvizMarker2_.lifetime = ros::Duration (0.5);
			rvizMarker3_.lifetime = ros::Duration (0.5);

			rvizMarkerPub_.publish (rvizMarker_);
//			rvizMarkerPub_.publish (rvizMarker2_);
			rvizMarkerPub_.publish (rvizMarker3_);
			ROS_DEBUG ("Published visual marker");
	      }
    }
    arMarkerPub_.publish (arPoseMarkers_);
    ROS_INFO ("Published ar_multi markers");
  }
}                               // end namespace ar_pose
