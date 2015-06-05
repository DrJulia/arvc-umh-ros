#pragma once
#ifndef __EKF_SLAM__
#define __EKF_SLAM__


#include <geometry_msgs/Pose.h>
#include <ar_pose/ARMarkers.h>
#include <matFuns.h>
#include <boost/thread/mutex.hpp>
#include <vector>

/**
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2012
* 
* Class EKFSLAM
*
* Implements an slam algorithm consisting on a extended Kalman filter
* 
*/

class EKFSLAM
{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:


	// parameters
	int numrobots_;					// number of robots
	int maxmarks_;					// reserve space for this number of landmarks
	float maxdist_;
	float mahTh_;					// data asociation
	float descTh_;
	bool matchByDist_;
	bool matchByDesc_;
	float alpha1_, alpha2_, alpha3_, alpha4_;	// odometry params
	float covini_x_, covini_y_, covini_th_;
	
	// EKF STATE
	Ematrix state_;
	Ematrix covariance_;
	
	int nummarks_;
	std::vector<int> descriptorlist_;
	std::vector<double> disprobot_;
	double uncertainty_;
	
	// aux vars
	int nBotsx3_;
	boost::mutex mutexControl_;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	/// Destructor
	virtual ~EKFSLAM();
	/// Constructor
	EKFSLAM(int numrobots, int nummarks, float maxdist,
                float mahTh,float descTh,bool matchByDist,bool matchByDesc,
                float alpha1,float alpha2,float alpha3,float alpha4,
		float covini_x, float covini_y, float covini_th );

	///initializer	
	void initialize(int numrobots, int nummarks, float maxdist,
             	        float mahTh,float descTh,bool matchByDist,bool matchByDesc,
                        float alpha1,float alpha2,float alpha3,float alpha4,	
			float covini_x, float covini_y, float covini_th );

	/// predict filter
	void predict(int robotid, const pose& controllast, const pose& controlnew);
	/// update filter
	void update(int robotid, const ar_pose::ARMarkers& observation, const tf::Transform& robot_to_camera);

	// outputs
	/// returns the current pose of the robot
	pose getPos(int robot) const;
	/// returns the matrix that represents the covariance of the position of the robot
	Ematrix getCovariance(int robot) const;
	
	pose4 getMark(int l) const;
	Ematrix getMarkCovariance(int mark) const;
	int getNumMarks() const;

	/// returns the matrix that represents the covariance of robots and marks
	//Ematrix getGlobalCovariance() const;

private:

	void init();

	/// evaluates the uncertainty on the position of each robot
	float evalUncertainty();

	/// odometry noise matrix Q
	Ematrix noiseQ(const pose& pos, const pose& deltaOdo, float thetaodo);
	/// jacobian F
	Ematrix jacobianF(const pose& pos, const pose& deltaOdo, float thetaodo);
	/// jacobian G
	Ematrix jacobianG(const pose& pos, const pose& deltaOdo, float thetaodo);
	// jacobian H
	Ematrix jacobianH1(const pose& pos, const pose4& mark, const tf::Transform& camtf);
	Ematrix jacobianH2(const pose& pos, const tf::Transform& camtf);
	Ematrix jacobianJ1(const pose& pos, const pose4& mark, const tf::Transform& robot_to_camera);

	// data associaction
	int dataAssociation(const Ematrix& ZT, const Ematrix& Rt, int desc, const pose4& globalpos, const pose& robotPos, int r, const Ematrix& H2, const Ematrix& H2trans, Ematrix& H1, pose4& mark, Ematrix& zgorro, const tf::Transform& robot_to_camera);

	/// motion model
	Ematrix motionModel (const pose& pos, const pose& deltaOdo, float thetaodo);


	pose4 base2cam(const pose4& p, const tf::Transform& robot_to_camera) const;
	pose4 cam2base(const pose4& p, const tf::Transform& robot_to_camera) const;
	int getMarkerPose(const geometry_msgs::Pose p, pose4& dest) const;
	Ematrix markcov(const geometry_msgs::PoseWithCovariance p )const ;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif 
