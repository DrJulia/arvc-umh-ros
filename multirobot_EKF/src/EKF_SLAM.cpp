#include <multirobot_EKF/EKF_SLAM.h>

using namespace std;

/// Constructor
EKFSLAM::EKFSLAM(int numrobots, int maxmarks, float maxdist,
                 float mahTh,float descTh,bool matchByDist,bool matchByDesc,
                 float alpha1,float alpha2,float alpha3,float alpha4,
		 float covini_x, float covini_y, float covini_th):

	numrobots_	(numrobots),
	maxmarks_	(maxmarks),
	maxdist_	(maxdist),
	mahTh_		(mahTh),
	descTh_		(descTh),
	matchByDist_ 	(matchByDist),
	matchByDesc_	(matchByDesc),
	alpha1_		(alpha1),
	alpha2_		(alpha2),
	alpha3_		(alpha3),
	alpha4_		(alpha4),
	covini_x_	(covini_x),
	covini_y_	(covini_y),
	covini_th_	(covini_th),
	state_		(3*numrobots_,           1, 3*numrobots_+4*maxmarks_,                       1),
	covariance_	(3*numrobots_,3*numrobots_, 3*numrobots_+4*maxmarks_,3*numrobots_+4*maxmarks_)
{
	init();
}

/// Destructor
EKFSLAM::~EKFSLAM(){

}

///initializer	
void EKFSLAM::initialize(int numrobots, int maxmarks, float maxdist,
                         float mahTh,float descTh,bool matchByDist,bool matchByDesc,
                         float alpha1,float alpha2,float alpha3,float alpha4,
			 float covini_x, float covini_y, float covini_th ){

	numrobots_	= numrobots;
	maxmarks_	= maxmarks;
	maxdist_ 	= maxdist;
	mahTh_		= mahTh;
	descTh_		= descTh;
	matchByDist_ 	= matchByDist;
	matchByDesc_	= matchByDesc;
	alpha1_		= alpha1;
	alpha2_		= alpha2;
	alpha3_		= alpha3;
	alpha4_		= alpha4;
	covini_x_	= covini_x;
	covini_y_	= covini_y;
	covini_th_	= covini_th;

	state_.initialize     	(3*numrobots_,           1, 3*numrobots_+4*maxmarks_,                       1);
	covariance_.initialize	(3*numrobots_,3*numrobots_, 3*numrobots_+4*maxmarks_,3*numrobots_+4*maxmarks_);

	init();
	
}

void EKFSLAM::init(){

	// auxvars
	nummarks_ = 0;
	nBotsx3_ = 3*numrobots_;
	descriptorlist_.reserve(maxmarks_);
	disprobot_.resize(numrobots_);

	// initial state
	state_.clear();	// all the robots in the same starting position

	// initial covariance
	covariance_.clear();
	// robot 0 sets the map origin, covariance 0
	// the other robots should be close, we initiate with the same pose with a parameter covariance
	for (int r=1; r<numrobots_; r++){
		covariance_.set(r*3  ,r*3  , covini_x_*covini_x_); // x
		covariance_.set(r*3+1,r*3+1, covini_y_*covini_y_); // y
		covariance_.set(r*3+2,r*3+2, covini_th_*covini_th_); // theta
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						PREDICTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EKFSLAM::predict(int robotid, const pose& controllast, const pose& controlnew){
//	controlnew.print("odom=");
	int ri3 = 3*robotid;									// robotid times 3
	pose controlinc = controlnew - controllast;

	pose xrk = getPos(robotid);								// cogemos la pose del estado	
	state_.set(ri3, 0, motionModel (xrk, controlinc, controllast.th));			// modelo de movimiento

	Ematrix Ft = jacobianF(xrk, controlinc,controllast.th); 
	Ematrix Fttrans = Ft.transpose();
	Ematrix Gt = jacobianG(xrk, controlinc,controllast.th);	
	Ematrix Qt = noiseQ(xrk,controlinc,controllast.th); 
	//Qt.print("Qt=");
	for (int r = 0; r<numrobots_; r++){
		int r3=3*r;
		if (r==robotid){
			covariance_.set(ri3,ri3, Ft*(covariance_.subMat(ri3,ri3+2,ri3,ri3+2)*Fttrans) +  Gt*(Qt*Gt.transpose()) );
		}
		else {
			Ematrix FtPvv = Ft*covariance_.subMat(ri3,ri3+2,r3,r3+2);
			covariance_.set(ri3,r3, FtPvv);
			covariance_.set(r3,ri3, FtPvv.transpose());
		}
	}
	for (int l=0; l<nummarks_; l++){
		int l4=4*l;
		Ematrix FtPvl = Ft*covariance_.subMat(ri3,ri3+2 ,nBotsx3_+l4,nBotsx3_+l4+3);
		covariance_.set(ri3,nBotsx3_+l4,FtPvl);
		covariance_.set(nBotsx3_+l4,ri3,FtPvl.transpose());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						UPDATE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// update with new observations
void EKFSLAM::update(int robotid, const ar_pose::ARMarkers& observation, const tf::Transform& camtf){

	printf("\n>> updating %d markers \n", observation.markers.size());
	int nummarks4 = 4*nummarks_;				// 4 * number of current marks in the map
	int observedmarks = observation.markers.size();		// 
	int observedmarks4 = 4*observedmarks;   		// 4 * number of marks in the observation
	
	// reserve space for matrices
	Ematrix ZT(0,1,observedmarks4,1);			// full observation vector
	Ematrix Rt(0,0,observedmarks4,observedmarks4);		
	Ematrix zgorro(0,1,observedmarks4,1);			// full predicted observation vector

	int ri3 = 3*robotid;
	int ri3mas2 = ri3+2; 
	pose xrk = getPos(robotid);
//	xrk.print("current pos=");
	
	Ematrix H(0,nBotsx3_+nummarks4,  observedmarks4,nBotsx3_+nummarks4+observedmarks4);
	Ematrix H2 = jacobianH2(xrk, camtf);	// jacobian H >> partial derivative of the predicted observation vs mark pose
	Ematrix H2trans = H2.transpose();
	
	int obsid=0;												// landmarks matched counter
	
	for (int l = 0; l < observedmarks; l++){								// for each detected marker

		float eqdist = sqrt( pow(observation.markers[l].pose.pose.position.x,2) + 				
                          pow(observation.markers[l].pose.pose.position.y,2) + 
                          pow(observation.markers[l].pose.pose.position.z,2) );

		if ( eqdist > maxdist_){
			printf("mark %d (id: %d) is too far away\n",l,observation.markers[l].id); 
			continue;
		} 				// ignore faraway markers
//		if ( eqdist < 1.0) {
//			printf("mark %d (id: %d) is too close\n",l,observation.markers[l].id); 
//			continue;
//		}					// ignore close markers
		if (observation.markers[l].pose.pose.position.z <= 0.0) {
			printf("mark %d (id: %d) is behind\n",l,observation.markers[l].id); 
			continue;
		} // ignore makers behind
		
		pose4 camera_marker;
		if ( getMarkerPose(observation.markers[l].pose.pose, camera_marker) < 0){
			printf("mark %d (id: %d) is not vertical\n",l,observation.markers[l].id); 
			continue;	// in camera frame				// continue if mark is not vertically oriented
		}
		//camera_marker.print("marker in camera coordinates=");
		pose4 local_marker = cam2base(camera_marker,camtf);					// in robot frame
		//local_marker.print("marker in local coordinates=");	
		pose4 global_marker = base2global(local_marker, xrk);			 		// in global frame
		//global_marker.print("marker in global coordinates=");

		if ( global_marker.z < 0.1) {
			printf("mark %d (id: %d) has negative height\n",l,observation.markers[l].id); 
			continue;
		}			// ignore underground markers
		
		Ematrix ZTi = camera_marker;								// one marker observation vector
		Ematrix Rti = markcov(observation.markers[l].pose);

		// we have to add noise here
		//ZTi.print("ZT=");		
		ZTi = gaussianSamplexd(ZTi, Rti);
		//ZTi.print("ZT'=");		

		//Rti.print("Rti = ");

		// data association
		Ematrix H1;
		pose4 mark;
		Ematrix zgorroi;
		int markermapid = dataAssociation(ZTi, Rti, observation.markers[l].id, global_marker, xrk, robotid, H2, H2trans, H1, mark, zgorroi, camtf) ;

		if (markermapid >= 0){ // if landmark already in the map	// obsid matches markermapid 	
			
			printf("marker [%d] match found, mark %d in the observation is mark %d in the map, append to update in position %d\n",observation.markers[l].id,  l, markermapid, obsid);
			
			// update descriptor
			descriptorlist_[markermapid] = (descriptorlist_[markermapid] + observation.markers[l].id)/2;
			int markermapid4 = 4*markermapid;			// number in the map
			int obsid4 = obsid * 4;					// number in the observation

			// copy to full matrices
			ZT.extend(4,0);
			ZT.set(obsid4,0,ZTi);
			Rt.extend(4,4);
			Rt.set(obsid4,obsid4,Rti);
			zgorro.extend(4,0);
			zgorro.set(obsid4,0,zgorroi);
			H.extend(4,0);					
			H.set(obsid4,ri3,H1);
			H.set(obsid4,nBotsx3_+markermapid4,H2);	
			obsid++;
		}
		else{ // new mark
			printf("new mark detected id: %d, saved in map position %d\n", observation.markers[l].id, nummarks_);
			
			nummarks_++;

			markermapid = nummarks_-1;
			int markermapid4 = 4*markermapid;		// number in the map
			descriptorlist_.push_back(observation.markers[l].id);
			
			// add the mark to the state map
			state_.extend(4,0);
			covariance_.extend(4,4);
			H.extend(0,4);
			
			state_.set(nBotsx3_+markermapid4,0, Ematrix(global_marker));	// fill state
			
			Ematrix Pl2 = H2trans*(Rti*(H2));
			Ematrix J1 = jacobianJ1(xrk,global_marker, camtf);			
			Ematrix Pvv = covariance_.subMat(ri3,ri3mas2,ri3,ri3mas2);
			Ematrix Pl = J1*Pvv*J1.transpose() + Pl2;
			//H2. print("H2=");			
			//Pl.print("PL=");
			Ematrix R = J1*covariance_.subMat(ri3,ri3mas2, 0, nBotsx3_+markermapid4-1);
			
			covariance_.set(0,nBotsx3_+markermapid4,R.transpose());	// fill covariance
			covariance_.set(nBotsx3_+markermapid4,0,R);
			covariance_.set(nBotsx3_+markermapid4,nBotsx3_+markermapid4,Pl);
			
			//pose4 markerbase = cam2base(camera_marker,camtf);
			//xrk.print("robot=");
			//ZTi.print("camera=");
			//markerbase.print("base=");
			//global_marker.print("global=");
		}
	} // for each mark
//	state_.print("X=");
//	covariance_.print("P=");
	if(obsid){
		Ematrix Htrans		= H.transpose();
		Ematrix In		= (ZT - zgorro);					// innovacion

		for (int jjj = 0; jjj < obsid; jjj++){ 
			int iii = 4*jjj+3;
			if (In.get(0,iii) > M_PI)  In.set(0,iii, In.get(0,iii) - PIx2 );
			if (In.get(0,iii) < -M_PI) In.set(0,iii, In.get(0,iii) + PIx2 );
		}

		//In.print("In=");
		Ematrix Aux 		= covariance_.mul2(Htrans);
		//Rt.print("Rt=");
		Ematrix S		= ((H*Aux) + Rt) ; //+ Ematrix::identity(4*obsid)*2 ;	// covarianza de la innovacion
		Ematrix K		= covariance_.mul2(Htrans*(S.inverse()));		// ganancia

		//S.print("S=");
		//H.print("H=");
		//K.print("K=");
		//(K*H).print("KH=");		
		//(Aux.transpose()).print("HP=");
		state_			+= K*In;					// actualizamos el estado
		covariance_		-= K.mul2(Aux.transpose());			// actualizamos covarianza
		covariance_.makesymmetric();
	}

	static int k = 0;
	printf("%d,%f,%f,%f\n", k++, state_.get(0,0), state_.get(0,0), state_.get(0,0));

	printf("markers in the map %d\n", nummarks_);
	//state_.transpose().print("X=");
	//covariance_.print("P=");

	for (int r=0; r<numrobots_; r++){
		if (state_.get(3*r+2,0)>PI)			state_.set(3*r+2,0,state_.get(3*r+2,0)- PIx2);
		else if (state_.get(3*r+2,0) <= -PI)		state_.set(3*r+2,0,state_.get(3*r+2,0)+ PIx2);
		disprobot_[r] = evalDisp(getCovariance(r));
	}
	
	uncertainty_ = evalUncertainty();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						DATA ASSOCIATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// asociacion de datos
int EKFSLAM::dataAssociation(const Ematrix& ZT, const Ematrix& Rt, int desc, const pose4& globalpos, const pose& robotPos, int r, const Ematrix& H2, const Ematrix& H2trans, Ematrix& H1_match, pose4& marker, Ematrix& zgorro, const tf::Transform& camtf) {

	float minDescDist=descTh_;
	float minMahDist=mahTh_;
	int r3= r*3;
	int markid=-1;

	for (int i = 0; i< nummarks_; i++){ // for each mark in the state

		float descdist = abs(descriptorlist_[i] - desc);
		if (descdist < descTh_){				// check descriptor distantce

//			printf("comparing marker id %d with marker in the map %d with id %d\n", desc, i, descriptorlist_[i]);
			int i4 = i*4;
			pose4 global_marker_est = getMark(i); 					// mark from the state	
		
			Ematrix H1 = jacobianH1(robotPos, global_marker_est, camtf); 		// jacobian H >> partial derivative of the predicted observation vs robot pose
		
			Ematrix H(4,7);
			H.set(0,0,H1);
			H.set(0,3,H2);
			Ematrix Htrans = H.transpose();
		
			Ematrix P(7,7);
			P.set(0,0,covariance_.subMat(r3,r3+2, r3,r3+2));
			P.set(0,3,covariance_.subMat(r3,r3+2, nBotsx3_+i4,nBotsx3_+i4+3));
			P.set(3,0,covariance_.subMat(nBotsx3_+i4,nBotsx3_+i4+3, r3,r3+2));
			P.set(3,3,covariance_.subMat(nBotsx3_+i4,nBotsx3_+i4+3, nBotsx3_+i4,nBotsx3_+i4+3));
		
			pose4 local_marker_est = global2base(global_marker_est, robotPos);
			Ematrix camera_marker_est = base2cam(local_marker_est,camtf); 		// preditected observation = h(x)
			
			//global_marker_est.print("observation model, mark in global =");		
			//local_marker_est.print("observation model, mark in local =");		
			//camera_marker_est.print("observation model, mark in camera =");		

			Ematrix In	= ZT - camera_marker_est;
			if (In.get(0,3) > M_PI)  In.set(0,3, In.get(0,3) - PIx2 );
			if (In.get(0,3) < -M_PI) In.set(0,3, In.get(0,3) + PIx2 );
			Ematrix S	= (H*(P*(Htrans))) + Rt;
		
			
			float matchprob = mahalanobis(In, S);
			//global_marker_est.print("global_marker_est=");
			//local_marker_est.print("local_marker_est=");
			//camera_marker_est.transpose().print("camera_marker_est=");			
			//ZT.print("Zt=");
			//In.print("in=");
			//S.print("S=");
//			printf("match probability %f\n", matchprob);
			if ( matchprob < mahTh_){					// check mahalanobis
				
				if (matchByDesc_){ 				// nearest desc
					if(descdist < minDescDist){ 
						markid = i;
						minDescDist = descdist;
						H1_match = H1;
						marker = global_marker_est;
						zgorro = camera_marker_est;
					}	
				}
				else {						// nearest mahalanobis
					if(matchprob < minMahDist){ 
						markid = i;
						minMahDist = matchprob;
						H1_match = H1;
						marker = global_marker_est;
						zgorro = camera_marker_est;
					}
				}
			}
		}
	}
	return (markid);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						MOTION, NOISE & JACOBIANS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// odometry noise matrix Q													// OK
Ematrix EKFSLAM::noiseQ(const pose& pos, const pose& deltaOdo, float thetaodo){
	float deltat  = sqrt((deltaOdo.x*deltaOdo.x+deltaOdo.y*deltaOdo.y));
	float deltar1;
	if (!(fabs(deltaOdo.x) < 0.000001 && fabs(deltaOdo.y) < 0.000001))
		deltar1 = atan2(deltaOdo.y,deltaOdo.x)-thetaodo;
	else
		deltar1 = 0;
	float deltar2 = deltaOdo.th - deltar1;

	if 	(deltar1>PI)		deltar1-= PIx2;
	else if (deltar1 <= -PI)	deltar1+= PIx2;
	if 	(deltar2>PI)		deltar2-= PIx2;
	else if (deltar2 <= -PI)	deltar2+= PIx2;

	Ematrix Q(3,3);
	Q.set(0,0, pow(alpha1_*fabs(deltar1) + alpha2_*deltat                   ,2));   // rotation 1 noise
	Q.set(1,1, pow(alpha3_*deltat        + alpha4_*(fabs(deltar1+deltar2))  ,2));   // translation noise
	Q.set(2,2, pow(alpha1_*fabs(deltar2) + alpha2_*deltat                   ,2));   // rotation 2 noise
	return Q;
}

// Motion model															// OK
Ematrix EKFSLAM::motionModel (const pose& pos, const pose& deltaOdo, float thetaodo){
	float deltatrans = sqrt((deltaOdo.x*deltaOdo.x+deltaOdo.y*deltaOdo.y));
	float aux = atan2(deltaOdo.y,deltaOdo.x) - thetaodo + pos.th;
	Ematrix X(3,1);
	X.set(0,0,deltatrans*cos(aux)+pos.x);
	X.set(1,0,deltatrans*sin(aux)+pos.y);
	X.set(2,0,pos.th+deltaOdo.th);
	if (X.get(2,0)>PI)			X.set(2,0,X.get(2,0)- PIx2);
	else if (X.get(2,0) <= -PI)		X.set(2,0,X.get(2,0)+ PIx2);
	return X;
}

// jacobian F															// OK
Ematrix EKFSLAM::jacobianF(const pose& pos, const pose& deltaOdo, float thetaodo){
	float deltatrans = sqrt((deltaOdo.x*deltaOdo.x+deltaOdo.y*deltaOdo.y));
	float aux = atan2(deltaOdo.y,deltaOdo.x) - thetaodo + pos.th;
	Ematrix F(3,3);
	F.set(0,0,1);
	F.set(0,2,-deltatrans*sin(aux));
	F.set(1,1,1);
	F.set(1,2, deltatrans*cos(aux));
	F.set(2,2,1);
	return F;
}

// jacobian G															// OK
Ematrix EKFSLAM::jacobianG(const pose& pos, const pose& deltaOdo, float thetaodo){
	float deltatransm = sqrt((deltaOdo.x*deltaOdo.x+deltaOdo.y*deltaOdo.y));
	float aux = atan2(deltaOdo.y,deltaOdo.x) - thetaodo + pos.th;
	Ematrix G(3,3);
	G.set(0,0,-deltatransm*sin(aux));
	G.set(0,1, cos(aux));
	G.set(1,0, deltatransm*cos(aux));
	G.set(1,1, sin(aux));
	G.set(2,0,1);
	G.set(2,2,1);
	return G;
}

// jacobian H1
// TODO review
Ematrix EKFSLAM::jacobianH1(const pose& pos, const pose4& mark, const tf::Transform& camtf){
//	btMatrix3x3 rot2 = camtf.inverse().getBasis();
//	btMatrix3x3 rot1;
//	rot1.setEulerYPR(-pos.th,0,0);
//	btMatrix3x3 trot =  rot2*rot1;




//	float costh = (float)cos(pos.th);
//	float sinth = (float)sin(pos.th);
//	float auxx = mark.x - pos.x;
//	float auxy = mark.y - pos.y;
//	btVector3 trans(-(auxx)*sinth + (auxy)*costh, -(auxx)*costh - (auxy)*sinth, 0 );
//	btVector3 res = rot2*trans;

//	Ematrix H1(4,3);
//	H1.set(0,0, -trot.getRow(0).getX());
//	H1.set(0,1, -trot.getRow(0).getY());
//	H1.set(1,0, -trot.getRow(1).getX());
//	H1.set(1,1, -trot.getRow(1).getY());
//	H1.set(2,0, -trot.getRow(2).getX());
//	H1.set(2,1, -trot.getRow(2).getY());

//	H1.set(0,2, res.getX());
//	H1.set(1,2, res.getX());
//	H1.set(2,2, res.getX());
//	H1.set(3,2,-1.0f);
//	
	tf::Transform transfm = camtf.inverse();
	tf::Matrix3x3 rot = transfm.getBasis();
	tf::Vector3 tras = transfm.getOrigin();

	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	float auxx = mark.x - pos.x;
	float auxy = mark.y - pos.y;
	Ematrix H1(4,3);

	H1.set(0,0, rot.getColumn(1).getX()*sinth - rot.getColumn(0).getX()*costh );
	H1.set(1,0, rot.getColumn(1).getY()*sinth - rot.getColumn(0).getY()*costh );
	H1.set(2,0, rot.getColumn(1).getZ()*sinth - rot.getColumn(0).getZ()*costh );
	H1.set(3,0, 0 );
 
	H1.set(0,1, - rot.getColumn(1).getX()*costh - rot.getColumn(0).getX()*sinth );
	H1.set(1,1, - rot.getColumn(1).getY()*costh - rot.getColumn(0).getY()*sinth );
	H1.set(2,1, - rot.getColumn(1).getZ()*costh - rot.getColumn(0).getZ()*sinth );
	H1.set(3,1, 0);
 
	H1.set(0,2, (rot.getColumn(0).getX()*costh - rot.getColumn(1).getX()*sinth)*auxy - (rot.getColumn(1).getX()*costh + rot.getColumn(0).getX()*sinth)*auxx - tras.getX() );
	H1.set(1,2, (rot.getColumn(0).getY()*costh - rot.getColumn(1).getY()*sinth)*auxy - (rot.getColumn(1).getY()*costh + rot.getColumn(0).getY()*sinth)*auxx - tras.getY() );
	H1.set(2,2, (rot.getColumn(0).getZ()*costh - rot.getColumn(1).getZ()*sinth)*auxy - (rot.getColumn(1).getZ()*costh + rot.getColumn(0).getZ()*sinth)*auxx - tras.getZ() );
	H1.set(3,2, 1);
		
	return H1;
}

// Jacobian H2															// OK
Ematrix EKFSLAM::jacobianH2(const pose& pos, const tf::Transform& camtf){
//	tf::Matrix3x3 rot2 = camtf.inverse().getBasis();
//	tf::Matrix3x3 rot1;
//	rot1.setEulerYPR(-pos.th,0,0);
//	tf::Matrix3x3 trot= rot2*rot1;
//	Ematrix H2(4,4);
//	H2.set(0,0,trot.getRow(0).getX());
//	H2.set(0,1,trot.getRow(0).getY());
//	H2.set(0,2,trot.getRow(0).getZ());
//	H2.set(1,0,trot.getRow(1).getX());
//	H2.set(1,1,trot.getRow(1).getY());
//	H2.set(1,2,trot.getRow(1).getZ());
//	H2.set(2,0,trot.getRow(2).getX());
//	H2.set(2,1,trot.getRow(2).getY());
//	H2.set(2,2,trot.getRow(2).getZ());
//	H2.set(3,3,1);

	tf::Transform transfm = camtf.inverse();
	tf::Matrix3x3 rot = transfm.getBasis();
	tf::Vector3 tras = transfm.getOrigin();
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);

	Ematrix H2n(4,4);

	H2n.set(0,0, rot.getColumn(0).getX()*costh - rot.getColumn(1).getX()*sinth );
	H2n.set(1,0, rot.getColumn(0).getY()*costh - rot.getColumn(1).getY()*sinth );
	H2n.set(2,0, rot.getColumn(0).getZ()*costh - rot.getColumn(1).getZ()*sinth );
	H2n.set(3,0, 0 );
 
	H2n.set(0,1, rot.getColumn(1).getX()*costh + rot.getColumn(0).getX()*sinth );
	H2n.set(1,1, rot.getColumn(1).getY()*costh + rot.getColumn(0).getY()*sinth );
	H2n.set(2,1, rot.getColumn(1).getZ()*costh + rot.getColumn(0).getZ()*sinth );
	H2n.set(3,1, 0);
 
	H2n.set(0,2,rot.getColumn(2).getX());
	H2n.set(1,2,rot.getColumn(2).getY());
	H2n.set(2,2,rot.getColumn(2).getZ());
	H2n.set(3,2,0);
  
	H2n.set(0,3,tras.getX());
	H2n.set(1,3,tras.getY());
	H2n.set(2,3,tras.getZ());
	H2n.set(3,3,1);

//	H2.print("H2=");
//	H2n.print("H2n=");

	return H2n;
}

// jacobian J1
// TODO review
Ematrix EKFSLAM::jacobianJ1(const pose& pos, const pose4& mark, const tf::Transform& camtf){
	Ematrix J1(4,3);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	pose4 p = cam2base(mark,camtf);
	J1.set(0,0,1.0f);
	J1.set(0,2,-p.x*sinth - p.y*costh);
	J1.set(1,1,1.0f);
	J1.set(1,2, p.x*costh - p.y*sinth);
	J1.set(3,2,1.0f);
	return J1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						TRANSFORMS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// translates a position in the robot's frame of reference to the camera frame of reference				// ok
pose4 EKFSLAM::base2cam(const pose4& p, const tf::Transform& camera_to_robot) const{
	pose4 output;
	tf::Vector3 input(p.x,p.y,p.z);
	tf::Vector3 res = camera_to_robot.inverse()*input;
	output.x = res.getX();
	output.y = res.getY();
	output.z = res.getZ();
	output.th = p.th; 	// this assumes the camera is not rotated (only swapping axis)!!!
	return output;
}

// translates a position in the camera frame of reference to the robot's frame of reference				// ok
pose4 EKFSLAM::cam2base(const pose4& p, const tf::Transform& camera_to_robot) const{
	pose4 output;
	tf::Vector3 input(p.x,p.y,p.z);
	tf::Vector3 res = camera_to_robot*input;
	output.x = res.getX();
	output.y = res.getY();
	output.z = res.getZ();
	//tf::Scalar roll,pitch,yaw;
	//tf::Matrix3x3(res.getRotation()).getRPY(roll, pitch, yaw);
	//printf("cam2base: roll: %f, pitch: %f, yaw: %f\n",roll, pitch, yaw);
	//tf::getYaw(res.getRotation());
	output.th = p.th; 	// this assumes the camera is not rotated (only swapping axis)!!!
	return output;
}

int EKFSLAM::getMarkerPose(const geometry_msgs::Pose p, pose4& output) const{						// ok
	output.x = p.position.x;
	output.y = p.position.y;
	output.z = p.position.z;
	tf::Pose marktf;
	tf::poseMsgToTF(p,marktf);
	double roll,pitch,yaw;
	tf::Matrix3x3(marktf.getRotation()).getRPY(roll, pitch, yaw);
	ROS_INFO("mark pose: roll: %f, pitch: %f, yaw: %f",roll, pitch, yaw);
	if ( fabs(fabs(roll)-M_PI) > 0.5) return -1;
	tf::Vector3 p1(0,0,-1); 
	tf::Vector3 p2 = marktf*p1;
	output.th = atan2(-(p2.getX()-output.x), p2.getZ()-output.z); // this assumes the camera is not rotated (only swapping axis)!!!
	return 0;
}

Ematrix EKFSLAM::markcov(const geometry_msgs::PoseWithCovariance p ) const{
	matrix Rti(4,4);
//	Rti.set(0,0,p.covariance[0]) ;
//	Rti.set(0,1,p.covariance[1]) ;
//	Rti.set(0,2,p.covariance[2]) ;
//	Rti.set(0,3,p.covariance[4]) ;
//	Rti.set(1,0,p.covariance[6]) ;
//	Rti.set(1,1,p.covariance[7]) ;
//	Rti.set(1,2,p.covariance[8]) ;
//	Rti.set(1,3,p.covariance[10]) ;
//	Rti.set(2,0,p.covariance[12]) ;
//	Rti.set(2,1,p.covariance[13]) ;
//	Rti.set(2,2,p.covariance[14]) ;
//	Rti.set(2,3,p.covariance[16]) ;
//	Rti.set(3,0,p.covariance[30]) ;
//	Rti.set(3,1,p.covariance[31]) ;
//	Rti.set(3,2,p.covariance[32]) ;
//	Rti.set(3,3,p.covariance[34]) ; 

//	Rti.set(0,0,0.002) ;
//	Rti.set(1,1,0.002) ;
//	Rti.set(2,2,0.008) ;
//	Rti.set(3,3,100) ;


	Rti.set(0,0,p.covariance[0]) ;
	Rti.set(0,1,p.covariance[1]) ;
	Rti.set(0,2,p.covariance[2]) ;
	Rti.set(0,3,p.covariance[4]) ;
	Rti.set(1,0,p.covariance[6]) ;
	Rti.set(1,1,p.covariance[7]) ;
	Rti.set(1,2,p.covariance[8]) ;
	Rti.set(1,3,p.covariance[10]) ;
	Rti.set(2,0,p.covariance[12]) ;
	Rti.set(2,1,p.covariance[13]) ;
	Rti.set(2,2,p.covariance[14]) ;
	Rti.set(2,3,p.covariance[16]) ;
	Rti.set(3,0,p.covariance[24]) ;
	Rti.set(3,1,p.covariance[25]) ;
	Rti.set(3,2,p.covariance[26]) ;
//	Rti.set(3,3,p.covariance[28]) ;  //  it's the roll rotation followed by the pitch rotation, so.... we add up their errors, it is not exact, but...
	Rti.set(3,3,0.3); //p.covariance[28]) ;  //  it's the roll rotation followed by the pitch rotation, so.... we add up their errors, it is not exact, but...
	return Rti;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//						OTHER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// evaluates the uncertainty 
float EKFSLAM::evalUncertainty(){

	float trace = 0;

	for (int i = 0; i < covariance_.getRows(); i++){
		trace += covariance_(i,i);
		//determinante += log(cvmGet(evals,0,i));
		//eigenvalues.push_back(cvmGet(evals,0,i));
	}
	trace /= covariance_.getRows();

	return trace;
}

/// returns the current pose of the robot
pose EKFSLAM::getPos(int r) const{

	return state_.subMat(r*3,r*3+2,0,0).toPose();
}

/// returns the current pose of the robot
pose4 EKFSLAM::getMark(int l) const{
	return state_.subMat(nBotsx3_+4*l,nBotsx3_+4*l+3,0,0).toPose4();
}

/// Returns the number of marks
int EKFSLAM::getNumMarks() const{
	return nummarks_;
};

/// returns the matrix that represents the covariance of the position of the robot
Ematrix EKFSLAM::getCovariance(int r) const{
	return covariance_.subMat(r*3,r*3+2,r*3,r*3+2);
}


/// returns the matrix that represents the covariance of the position of the robot
Ematrix EKFSLAM::getMarkCovariance(int mark) const{
	return covariance_.subMat(nBotsx3_+4*mark,nBotsx3_+4*mark+3,nBotsx3_+4*mark,nBotsx3_+4*mark+3);
}


