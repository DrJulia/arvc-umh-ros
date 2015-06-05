#pragma once

#ifndef __REFLECTION__PROBABILITY__
#define __REFLECTION__PROBABILITY__

#include "binMap.h"
#include "robotTypes.h"
#include "occupancyGridMap.h"
#include <nav_msgs/OccupancyGrid.h>

/***************************************************************************************************************
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2011
* 
* Class OGMros
*
* Implements an ocupancy grid map with a nav_msgs::OccupancyGrid ros message for data
*
*/

class OGMros: public occupancyGridMap{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:
	//int totalsize;
	nav_msgs::OccupancyGrid* cells;		// Data cell array
	//float step;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	/// Default Constructor
	OGMros();
	/// From ros message constructor
	OGMros(nav_msgs::OccupancyGrid& gmap);

	/// Constructor with size in meters and related to real coordinates
	OGMros(float w, float h, float res, float x, float y);
	/// Destructor
	virtual ~OGMros();
	
	/// Initializes with size in meters and related to real coordinates
	void initialize(float w, float h, float res, float x, float y);
	void initialize(nav_msgs::OccupancyGrid&);
	/// clone method
	OGMros* clone() const;
	
	/// assigment operator
	gridMapInterface& operator=(const gridMapInterface&);
	
	/// return the occupancy probability of the cell
	float getValue(const int& x, const int& y) const;
	/// return the occupancy probability of the cell
	float getValue(const uint& x, const uint& y) const;
	/// return the occupancy probability of the cell
	float getValueNotSafe(const int& x, const int& y) const;
	/// return the occupancy probability of the cell
	float getValueNotSafe(const uint& x, const uint& y) const;
	/// Resets the occupation probability for the total map
	void reset();
	
	/// Updates the occupancy grid using the new data
	void update(const rangeSensorData& rsData, const pose& rpos, float disp=0){};
	/// save a map to disk
	int saveMapToFile(const char* file) const{return 0;};
	/// load a map from disk
	int loadMapFromFile(const char* file){return 0;};

	bool isfree(const uint& x, const uint& y) const;
	bool isoccupied(const uint& x, const uint& y) const;
	bool isfree(const int& x, const int& y) const;
	bool isoccupied(const int& x, const int& y) const;
	bool isunknown(const uint& x, const uint& y) const;
	bool isunknown(const int& x, const int& y) const;
private:

	/// copy constructor
	OGMros(const OGMros&);


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline float OGMros::getValue(const int& x, const int& y) const{
	int idx(y*width+x);
	return (idx >=0 && idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) !=255)? cells->data[idx]:50.0f ) : 50.0f;
};


inline float OGMros::getValue(const uint& x, const uint& y) const{
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) !=255)? cells->data[idx]:50.0f ) : 50.0f;
};

inline float OGMros::getValueNotSafe(const int& x, const int& y) const{
	int idx(y*width+x);
	return  (((uchar)cells->data[idx]) !=255)? cells->data[idx]:50.0f;
};


inline float OGMros::getValueNotSafe(const uint& x, const uint& y) const{
	int idx(y*width+x);
	return (((uchar)cells->data[idx]) !=255)? cells->data[idx]:50.0f;
};

inline OGMros* OGMros::clone() const {return new OGMros(*this);};

inline bool OGMros::isunknown(const uint& x, const uint& y) const {
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) ==255)? true: false ) : true;
}
inline bool OGMros::isunknown(const int& x, const int& y) const	{
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) ==255)? true: false ) : true;
}


inline bool OGMros::isfree(const int& x, const int& y) const {
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) <50)? true: false ) : false;
}
inline bool OGMros::isoccupied(const int& x, const int& y) const {
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) >=50 && ((uchar)cells->data[idx]) !=255)? true: false ) : false;
}
inline bool OGMros::isfree(const uint& x, const uint& y) const {
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) <50)? true: false ) : false;
}
inline bool OGMros::isoccupied(const uint& x, const uint& y) const {
	int idx(y*width+x);
	return (idx < ((int)cells->data.size()))? ( ( ((uchar)cells->data[idx]) >=50  && ((uchar)cells->data[idx]) !=255)? true: false ) : false;
}


#endif
