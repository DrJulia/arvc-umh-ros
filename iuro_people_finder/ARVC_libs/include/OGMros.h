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
	nav_msgs::OccupancyGrid cells;		// Data cell array
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
	OGMros(const nav_msgs::OccupancyGrid& gmap);
	/// Constructor with size in meters and related to real coordinates
	OGMros(float w, float h, float res, float x, float y);
	/// Destructor
	virtual ~OGMros();
	
	/// Initializes with size in meters and related to real coordinates
	void initialize(float w, float h, float res, float x, float y);
	/// clone method
	OGMros* clone() const;
	
	/// assigment operator
	gridMapInterface& operator=(const gridMapInterface&);
	
	/// return the occupancy probability of the cell
	float getValue(int x, int y) const;
	/// Resets the occupation probability for the total map
	void reset();
	
	/// Updates the occupancy grid using the new data
	void update(const rangeSensorData& rsData, const pose& rpos, float disp=0){};
	/// save a map to disk
	int saveMapToFile(const char* file) const{return 0;};
	/// load a map from disk
	int loadMapFromFile(const char* file){return 0;};

private:

	/// copy constructor
	OGMros(const OGMros&);


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline float OGMros::getValue(int x, int y) const{
	int idx(y*width+x);
	return (idx >=0 && idx < ((int)cells.data.size()))? ( ( ((uchar)cells.data[idx]) !=255)? cells.data[idx]:50.0f ) : 50.0f;
};

inline OGMros* OGMros::clone() const {return new OGMros(*this);};

#endif
