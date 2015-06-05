/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class OGMros
*
* Implements an ocupancy grid map
*
*
*
*/

#include <math.h> 
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <loki/Factory.h>
#include "OGMros.h"
#include <iostream>

// Para registrar la clase en la factoria
namespace{
	gridMapInterface* CreateOGMFromROS(float w, float h, float res, float x, float y){
		return new OGMros(w,h,res,x,y);
	};
	const int OGMFROMROSMSG = 5;
	const bool registered = gridMapFactory::Instance().Register(OGMFROMROSMSG, gridMapCreator(CreateOGMFromROS));
}

using namespace std;

// Constructors
OGMros::OGMros():
	occupancyGridMap()
{
};

OGMros::OGMros(float w, float h, float res, float x, float y):
	occupancyGridMap(w,h,res,x,y)
{	
	cells = new nav_msgs::OccupancyGrid();
	cells->info.resolution=res;
	cells->info.width=width;
	cells->info.height=height;
	cells->data.resize(width*height);
	reset();
}

OGMros::~OGMros(){
	
}

OGMros::OGMros(const OGMros& gmap):
	occupancyGridMap(gmap.realWidth, gmap.realHeight, gmap.resolution, gmap.xorigin, gmap.yorigin)
{
	cells = gmap.cells;
}

OGMros::OGMros(nav_msgs::OccupancyGrid& mapmsg):
	occupancyGridMap(mapmsg.info.width*mapmsg.info.resolution, mapmsg.info.height*mapmsg.info.resolution, mapmsg.info.resolution, mapmsg.info.origin.position.x, mapmsg.info.origin.position.y)
{
	cells = &mapmsg;
}


gridMapInterface& OGMros::operator=(const gridMapInterface& gmap){
  	assert(typeid(gmap) == typeid(*this));
  	const OGMros& map = dynamic_cast<const OGMros&>(gmap);
  	occupancyGridMap::operator=(gmap);
	cells = map.cells;
	return *this;
}

void OGMros::initialize( nav_msgs::OccupancyGrid&  mapmsg){
  	initialize(mapmsg.info.width*mapmsg.info.resolution, mapmsg.info.height*mapmsg.info.resolution, mapmsg.info.resolution, mapmsg.info.origin.position.x, mapmsg.info.origin.position.y);
	cells = &mapmsg;
}


// Initializer
void OGMros::initialize(float w, float h, float res, float x, float y){
	width  = (int)floor(w/res+0.5);
	height = (int)floor(h/res+0.5);
	realWidth  = w;
	realHeight = h;
	xorigin = x;
	yorigin = y;
	resolution = res;
	cells = new nav_msgs::OccupancyGrid();
	cells->info.resolution=res;
	cells->info.width=width;
	cells->info.height=height;
	cells->data.resize(width*height);
	reset();
};

// Data operations
void OGMros::reset(){
	for(uint i=0;i<cells->data.size();i++){
		cells->data[i] = 255;
	}
}


