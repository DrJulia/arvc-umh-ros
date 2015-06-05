#pragma once
#ifndef __GRID__MAP__ITERFACE__
#define __GRID__MAP__ITERFACE__

#include "binMap.h"
#include "robotTypes.h"
#include "matFuns.h"
#include "rangeSensorData.h"
#include <cv.h>
#include <loki/Factory.h>
#include <loki/Typelist.h>
#include <loki/Functor.h>

/**
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class gridMapInterface
*
* Interface to grid map algorithm class
*
*/
class gridMapInterface
{
///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:

	virtual ~gridMapInterface(){};

	/// Initializes with size in meters and related to real coordinates
	virtual void initialize(float width, float height, float resolution, float xorigin, float yorigin)=0;
	/// clone method
	virtual gridMapInterface* clone() const=0;
	/// assigment operator
	virtual gridMapInterface& operator=(const gridMapInterface&)=0;

	/// return the occupancy probability of the cell
	virtual float getValue(const int& x, const int& y) const=0;
	/// return the occupancy probability of the cell
	virtual float getValue(const uint& x, const uint& y) const=0;
	/// return the occupancy probability of the cell
	virtual float getValue(const uint& x, const uint& y, const uint& range) const=0;
	/// Resets the occupation probability for the total map
	virtual void reset()=0;
	/// Updates the occupancy grid using the new data
	virtual void update(const rangeSensorData& rsData, const pose& rpos, float disp)=0;

	/// sets the x real coordinates of pixel(0,0)
	virtual void setXOrigin(float xori)=0;
	/// sets the y real coordinates of pixel(0,0)
	virtual void setYOrigin(float yori)=0;
	/// sets the grid map resolution in meters
	virtual void setResolution(float res)=0;
	
	/// returns the width of the grid map
	virtual int getWidth() const=0;
	/// returns the height of the grid map
	virtual int getHeight() const=0;
	/// returns the width of the grid map
	virtual float getRealWidth() const=0;
	/// returns the height of the grid map
	virtual float getRealHeight() const=0;
	/// returns the x real coordinates of pixel(0,0)
	virtual float getXOrigin() const=0;
	/// returns the y real coordinates of pixel(0,0)
	virtual float getYOrigin() const=0;
	/// returns the grid map resolution in meters
	virtual float getResolution() const=0;
	
	/// Returns a binMap of the frontiers
	virtual int frontiers(binMap& frontiers) const=0;
	/// Returns a binMap of the frontiers inside the given zone
	virtual int frontiersIn(const binMap& zone, binMap& frontiers) const=0;
	/// Evaluates the expected safe zone viewed from a given point
	virtual void esz(int x, int y, binMap& esz, int dilaterad, int size) const=0;
	/// Evaluates the expected safe zone viewed from a given point
	virtual void sz(int x, int y, binMap& esz, int dilaterad, int size) const=0;
	virtual void fov(int x, int y, binMap& map, int dilaterad, int max) const=0;
	virtual void secview(int x, int y, binMap& map, int dilaterad, int max) const=0;
	virtual void rec_cast_light(binMap& map, int cx, int cy, int row, float start, float end, int radius, int xx, int xy, int yx, int yy, int id, int dilaterad) const=0;
	virtual void rec_cast_light_safe(binMap& map, const int& cx, const int& cy, const int& row, float& start, const float& end, const int& radius, const int& xx, const int& xy, const int& yx, const int& yy, const int& dilaterad) const=0;	
	/// Looks for gateways in a given zone
	virtual void gateways(const binMap& esz, binMap& gateways) const=0;
	virtual void clearUnconnect(binMap& vz,uint x, uint y) const=0;
	/// Return the occupied cells
	virtual void occupiedCells(binMap& occupied, int dilateRad = 1) const=0;
	
	/// true if the cell is free
	virtual bool isfree(const int& x, const int& y) const=0;
	virtual bool isfreeNotSafe(const uint& x, const uint& y) const=0;
	/// true if the cell is occupied
	virtual bool isoccupied(const int& x, const int& y) const=0;
	/// true if the cell is unknown
	virtual bool isunknown(const int& x, const int& y) const=0;
	/// true if the cell is frontier
	virtual bool isfrontier(const int& x, const int& y) const=0;
	/// true if the cell is free
	virtual bool isfree(const uint& x, const uint& y) const=0;
	/// true if the cell is occupied
	virtual bool isoccupied(const uint& x, const uint& y) const=0;
	/// true if the cell is unknown
	virtual bool isunknown(const uint& x, const uint& y) const=0;
	/// true if the cell is frontier
	virtual bool isfrontier(const uint& x, const uint& y) const=0;
	
	/// true if all cells in a given square mask of radius rad are free (Totally free)
	virtual bool isfree(const uint& x, const uint& y, const uint& rad) const=0;	
	/// true if all cells in a diamond 1px mask of radius rad are free (Totally free)
	virtual bool isfreeD(const uint& x, const uint& y) const=0;	
	/// true if a cell in a given square mask of radius rad is occupied (Partially occupied)
	virtual bool isoccupied(const uint& x, const uint& y, const uint& rad) const=0;
	/// true if a cell in a given square mask of radius rad is unknown (Partially unknown)
	virtual bool isunknown(const uint& x, const uint& y, const uint& size) const=0;
	
	/// Save function
	virtual void saveMapAsImage(const char* file) const=0;
	/// Get the map as a new opencv image
	virtual IplImage* getMapAsImage() const=0;
	/// Show binMap
	virtual void showMap(const char* windowname)const =0;

	/// save a map to disk
	virtual int saveMapToFile(const char* file) const = 0;
	/// load a map from disk
	virtual int loadMapFromFile(const char* file) = 0;

	virtual int countAccessible(const point* p, int numpoints, binMap& accessible) const=0;

	virtual point toCell(float x, float y) const=0;
	virtual pointf toCoords(int x, int y) const=0;
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

typedef Loki::Functor<gridMapInterface*, LOKI_TYPELIST_5(float,float,float,float,float)> gridMapCreator;
typedef Loki::SingletonHolder< Loki::Factory< gridMapInterface, int, LOKI_TYPELIST_5(float,float,float,float,float)> > gridMapFactory;


#endif

