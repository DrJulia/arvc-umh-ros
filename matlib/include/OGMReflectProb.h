#pragma once

#ifndef __REFLECTION__PROBABILITY__
#define __REFLECTION__PROBABILITY__

#include "binMap.h"
#include "robotTypes.h"
#include "occupancyGridMap.h"

///represents the occupation probability of a cell in a grid occupancy map
typedef struct cell{
	float occ;
	float total;
	float val;
	//float val2;
	cell():occ(0),total(0),val(50.0f)//,val2(50.0f)
	{};
}cell;

static cell nullcell;

/***************************************************************************************************************
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class OGMReflectProb
*
* Implements an ocupancy grid map 
*
*/

class OGMReflectProb: public occupancyGridMap{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:
	int totalsize;
	cell* cells;		// Data cell array
	float step;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	/// Default Constructor
	OGMReflectProb();
	/// Constructor with size in meters and related to real coordinates
	OGMReflectProb(float w, float h, float res, float x, float y);
	/// Destructor
	virtual ~OGMReflectProb();
	
	/// Initializes with size in meters and related to real coordinates
	void initialize(float w, float h, float res, float x, float y);
	/// clone method
	OGMReflectProb* clone() const;
	
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
	void update(const rangeSensorData& rsData, const pose& rpos, float disp=0);
	
	/// save a map to disk
	int saveMapToFile(const char* file) const;
	/// load a map from disk
	int loadMapFromFile(const char* file);

private:

	/// copy constructor
	OGMReflectProb(const OGMReflectProb&);
	/// Returns the occupation probability for a cell 
	cell& get(const int& x, const int& y) ;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline float OGMReflectProb::getValue(const int& x, const int& y) const{
	int idx(y*width+x);
	return (idx >=0 && idx < totalsize)? cells[idx].val : 50.0f;
};
inline float OGMReflectProb::getValue(const uint& x, const uint& y) const{
	int idx(y*width+x);
	return (idx < totalsize)? cells[idx].val : 50.0f;
};
inline float OGMReflectProb::getValueNotSafe(const int& x, const int& y) const{
	int idx(y*width+x);
	return cells[idx].val;
};
inline float OGMReflectProb::getValueNotSafe(const uint& x, const uint& y) const{
	int idx(y*width+x);
	return cells[idx].val;
};

inline cell& OGMReflectProb::get(const int& x, const int& y) {
	int idx(y*width+x);
	return (idx >=0 && idx < totalsize)? cells[idx]: nullcell;
};

inline OGMReflectProb* OGMReflectProb::clone() const {return new OGMReflectProb(*this);};

#endif
