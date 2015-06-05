#include "occupancyGridMap.h"
#include <math.h> 
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include "binMap.h"

// Constructors
occupancyGridMap::occupancyGridMap():
	width(0),
	height(0),
	realWidth(0),
	realHeight(0),
	xorigin(0),
	yorigin(0),
	resolution(0)
{

}

occupancyGridMap::occupancyGridMap(float w, float h, float res, float x, float y):
	width((int)floor(w/res+0.5)),
	height((int)floor(h/res+0.5)),
	realWidth(w),
	realHeight(h),
	xorigin(x),
	yorigin(y),
	resolution(res)
{
	
}

gridMapInterface& occupancyGridMap::operator=(const gridMapInterface& gmap){
    assert(typeid(gmap) == typeid(*this));
    const occupancyGridMap& map = dynamic_cast<const occupancyGridMap&>(gmap);
	width = map.width;
	height = map.height;
	realWidth = map.realWidth;
	realHeight = map.realHeight;
	xorigin = map.xorigin;
	yorigin = map.yorigin;
	resolution = map.resolution;
	return *this;
}

occupancyGridMap::~occupancyGridMap(){};


// Search
int occupancyGridMap::frontiers(binMap& newfrontiers) const{
	int i,j;
	int nfront=0;

	newfrontiers.initialize(width,height,resolution, xorigin, yorigin);

	for (i =0 ; i < width ; i++){
		for (j = 0 ; j < height ; j++){  				// for each cell
			if (isfrontier(i,j)) {						// celda en la zona segura
				newfrontiers.set(i,j,true);
				nfront++;			
			}
		}
	}
	return nfront;
}

void occupancyGridMap::secview(int x, int y, binMap& map, int dilaterad, int max) const{

	static const int mult[] = {     1,  0,  0, -1, -1,  0,  0,  1,
					0,  1, -1,  0,  0, -1,  1,  0,
					0,  1,  1,  0,  0, -1, -1,  0,
					1,  0,  0,  1, -1,  0,  0, -1};

	ros::WallTime startInitTime = ros::WallTime::now();
	map.initialize(width, height, resolution, xorigin, yorigin);
	ros::WallTime startRayCastTime = ros::WallTime::now();	
	double totalInitTime = startRayCastTime.toSec() - startInitTime.toSec();
	
	RoI roi;
	roi.x = x;
	roi.y = y;
	roi.width = 1;
	roi.height = 1;
	map.set(x,y,true);
	map.setRoi(roi);
 
        for (int oct = 0; oct < 8; oct++){
	    float start = 1.0f, stop = 0.0f;
            rec_cast_light_safe(map, x, y, 1, start, stop, max, mult[oct], mult[8+oct], mult[16+oct], mult[24+oct], dilaterad);
	}
	ros::WallTime startDilateTime = ros::WallTime::now();	
	double totalRayCastTime = startDilateTime.toSec() - startRayCastTime.toSec();


	map.dilate(dilaterad);
	ros::WallTime endTime = ros::WallTime::now();	
	double totalDilateTime = endTime.toSec() - startDilateTime.toSec();

//	ROS_INFO("TOTAL TIME INITIALIZING %f, TIME RAY CASTING: %f, TIME DILATING: %f", totalInitTime, totalRayCastTime, totalDilateTime);
}

void occupancyGridMap::fov(int x, int y, binMap& map, int dilaterad, int max) const{

	static const int mult[] = {     1,  0,  0, -1, -1,  0,  0,  1,
					0,  1, -1,  0,  0, -1,  1,  0,
					0,  1,  1,  0,  0, -1, -1,  0,
					1,  0,  0,  1, -1,  0,  0, -1};

	map.initialize(width, height, resolution, xorigin, yorigin);
	RoI roi;
	roi.x = x;
	roi.y = y;
	roi.width = 1;
	roi.height = 1;
	map.set(x,y,true);
	map.setRoi(roi);
 
        for (int oct = 0; oct < 8; oct++)
            rec_cast_light(map, x, y, 1, 1.0, 0.0, max, mult[oct], mult[8+oct], mult[16+oct], mult[24+oct], 0, dilaterad);

	map.dilate(dilaterad);
}

void occupancyGridMap::rec_cast_light(binMap& map, int cx, int cy, int row, float start, float end, int radius, int xx, int xy, int yx, int yy, int id, int dilaterad) const{
        if (start < end) return;
	float new_start=0;
        float radius_squared = radius*radius;
        for (int j = row; j <= radius; j++){
            int dx = -j-1, dy = -j;
            bool blocked = false;
            while (dx <= 0){
                dx += 1;
                // l_slope and r_slope store the slopes of the left and right
                // extremities of the square we're considering:
                float l_slope = (dx-0.5)/(dy+0.5);
	        float r_slope = (dx+0.5)/(dy-0.5);
                if (start < r_slope) continue;
                else if (end > l_slope) break;
                else{
                    // Translate the dx, dy coordinates into map coordinates:
                    uint X = cx + dx * xx + dy * xy;
		    uint Y = cy + dx * yx + dy * yy;
		    if (X+dilaterad >= getWidth()  || (int)X-dilaterad < 0) continue;
        	    if (Y+dilaterad >= getHeight() || (int)Y-dilaterad < 0) continue;
                    // Our light beam is touching this square; light it:
                    if (dx*dx + dy*dy < radius_squared)
                        map.set(X, Y,true);
                    if (blocked){
                        // we're scanning a row of blocked squares:
                        if (isoccupied(X, Y, dilaterad)){
                            new_start = r_slope;
                            continue;
			}
                        else{
                            blocked = false;
                            start = new_start;
			}
		    }
                    else{
                        if (j < radius && isoccupied(X, Y,dilaterad)){
                            // This is a blocking square, start a child scan:
                            blocked = true;
                            rec_cast_light(map, cx, cy, j+1, start, l_slope, radius, xx, xy, yx, yy, id+1,dilaterad);
                            new_start = r_slope;
			    if(X > map.getRoi().x+map.getRoi().width-1)		 map.getRoi().width   = X-map.getRoi().x+1;
			    else if(X < map.getRoi().x) 			{map.getRoi().width  += map.getRoi().x-X;   map.getRoi().x    = X;}
			    if(Y > map.getRoi().y+map.getRoi().height-1) 	 map.getRoi().height  = Y-map.getRoi().y+1;
			    else if(Y < map.getRoi().y) 			{map.getRoi().height += map.getRoi().y-Y;   map.getRoi().y    = Y;}
			}
		    }
		}
            }
	    // Row is scanned; do next row unless last square was blocked:
            if (blocked) break;
	}
}


void occupancyGridMap::rec_cast_light_safe(binMap& map, const int& cx, const int& cy, const int& row, float& start, const float& end, const int& radius, const int& xx, const int& xy, const int& yx, const int& yy, const int& dilaterad) const{
        if (start < end) return;
	float new_start=0;
        float radius_squared = radius*radius;
	float l_slope, r_slope;
	uint X,Y;
	int dx,dy;
	bool blocked;	
        for (int j = row; j <= radius; j++){
            dx = -j-1, dy = -j;
            blocked = false;
	    float dyminus05 = dy-0.5;
	    float dyplus05 = dy+0.5;
	    float dyxy = dy * xy;
	    float dyyy = dy * yy;
	    float dy2 = dy*dy;
            while (dx <= 0){
                dx += 1;
                // l_slope and r_slope store the slopes of the left and right
                // extremities of the square we're considering:
	        r_slope = (dx+0.5)/dyminus05;
                if (start < r_slope) continue;
                l_slope = (dx-0.5)/dyplus05;
                if (end > l_slope) break;
                else{
		    // Translate the dx, dy coordinates into map coordinates:
		    X = cx + dx * xx + dyxy;
		    Y = cy + dx * yx + dyyy;
		    if (X+dilaterad >= getWidth()  || (int)X-dilaterad < 0) continue;
        	    if (Y+dilaterad >= getHeight() || (int)Y-dilaterad < 0) continue;
                    // Our light beam is touching this square; light it:
                    if (dx*dx + dy2 < radius_squared)
                        map.set(X, Y,true);
                    if (blocked){
                        // we're scanning a row of blocked squares:
                        if (isfreeNotSafe(X, Y, dilaterad)){
                            blocked = false;
                            start = new_start;
			}
                        else{
                            new_start = r_slope;
                            continue;
			}
		    }
                    else if (!isfreeNotSafe(X, Y,dilaterad)){
                            // This is a blocking square, start a child scan:
                            blocked = true;
                            rec_cast_light_safe(map, cx, cy, j+1, start, l_slope, radius, xx, xy, yx, yy,dilaterad);
                            new_start = r_slope;
			    if(X > map.getRoi().x+map.getRoi().width-1)			 map.getRoi().width   = X-map.getRoi().x+1;
			    else if(X < map.getRoi().x) 				{map.getRoi().width  += map.getRoi().x-X;   map.getRoi().x    = X;}
			    if(Y > map.getRoi().y+map.getRoi().height-1) 		 map.getRoi().height  = Y-map.getRoi().y+1;
			    else if(Y < map.getRoi().y) 				{map.getRoi().height += map.getRoi().y-Y;   map.getRoi().y    = Y;}
		    }
		}
            }
	    // Row is scanned; do next row unless last square was blocked:
            if (blocked) break;
	}
}

void occupancyGridMap::esz(int x, int y, binMap& map, int dilaterad, int max) const{
	float max2 = (float)(max*max);
//	printf("[ESZ] map initialize...\n");
	map.initialize(width, height, resolution, xorigin, yorigin);
//	ROS_INFO("[ESZ] size %d, %d origin %f,%f res %f", width, height, xorigin, yorigin, resolution);
//	printf("[ESZ] map initialize OK\n");
	RoI roi;
	roi.x = x;
	roi.y = y;
	roi.width = 1;
	roi.height = 1;
	map.set(x,y,true);

	// trazamos rayos desde el centro hasta los bordes de un cuadrado de area 2max x 2max
	int i,j;
	// linea inferior
	j = y-max;
//	ROS_INFO("new safe zone\n");
	uint size;
	for (i = x-max; i <= x+max; i++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
			if (line[k].x<width && line[k].y< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || isoccupied(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					//if(jj > roi.y+roi.height-1)			roi.height = line[k].y-roi.y+1;
					if(line[k].y < roi.y) 				{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
			}
		}
		delete[] line;
	}

	// linea superior
	j = y+max;
	for (i = x-max; i <= x+max; i++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
			if (line[k].x<width && line[k].y< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || isoccupied(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					//else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
			}
		}
		delete[] line;
	}

	// linea izda
	i = x-max;
	for (j = y-max+1; j <= y+max-1; j++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
			if (line[k].x<width && line[k].y< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || isoccupied(line[k].x,line[k].y,dilaterad)){
					//if(ii > roi.x+roi.width-1)			roi.width = line[k].x-roi.x+1;
					if(line[k].x < roi.x) 				{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
			}
		}
		delete[] line;
	}

	// linea dcha
	i = x+max;
	for (j = y-max+1; j <= y+max-1; j++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
			if (line[k].x<width && line[k].y< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || isoccupied(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					//else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
			}
		}
		delete[] line;
	}

	map.setRoi(roi);
	map.set(x,y,true);
//	printf("[ESZ] dilate...\n");
	map.dilate(dilaterad);
//	printf("[ESZ] dilate OK\n");

//	ROS_INFO ("ESZ ROI: %d, %d, %d, %d", roi.x, roi.y = y, roi.width, roi.height);
}


void occupancyGridMap::sz(int x, int y, binMap& map, int dilaterad, int max) const{
	float max2 = (float)(max*max);
//	printf("[ESZ] map initialize...\n");
	map.initialize(width, height, resolution, xorigin, yorigin);
//	ROS_INFO("[ESZ] size %d, %d origin %f,%f res %f", width, height, xorigin, yorigin, resolution);
//	printf("[ESZ] map initialize OK\n");
	RoI roi;
	roi.x = x;
	roi.y = y;
	roi.width = 1;
	roi.height = 1;
	map.set(x,y,true);

	// trazamos rayos desde el centro hasta los bordes de un cuadrado de area 2max x 2max
	int i,j;

	// linea inferior
	j = y-max;
	uint size;
//	ROS_INFO("new safe zone\n");
	for (i = x-max; i <= x+max; i++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
//			if (ii>=0 && jj>=0 && ii<width && jj< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || !isfree(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					//if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					if(line[k].y < roi.y) 				{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
//			}
		}
		delete[] line;
	}

	// linea superior
	j = y+max;
	for (i = x-max; i <= x+max; i++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
//			if (ii>=0 && jj>=0 && ii<width && jj< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || !isfree(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					//else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
//			}
		}
		delete[] line;
	}

	// linea izda
	i = x-max;
	for (j = y-max+1; j <= y+max-1; j++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
//			if (ii>=0 && jj>=0 && ii<width && jj< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || !isfree(line[k].x,line[k].y,dilaterad)){
					//if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					if(line[k].x < roi.x) 				{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
//			}
		}
		delete[] line;
	}

	// linea dcha
	i = x+max;
	for (j = y-max+1; j <= y+max-1; j++){
		point* line = getLine(x,y,i,j,size);
		for (uint k = 1; k < size; k++){
//			ii = line[k].x; jj = line[k].y;
//			if (ii>=0 && jj>=0 && ii<width && jj< height){
				float xdiff = (float)(x-line[k].x), ydiff= (float)(y-line[k].y);
				if (xdiff*xdiff+ydiff*ydiff>max2 || !isfree(line[k].x,line[k].y,dilaterad)){
					if(line[k].x > roi.x+roi.width-1)		roi.width = line[k].x-roi.x+1;
					//else if(line[k].x < roi.x) 			{roi.width  += roi.x-line[k].x; roi.x    = line[k].x;}
					if(line[k].y > roi.y+roi.height-1)		roi.height = line[k].y-roi.y+1;
					else if(line[k].y < roi.y) 			{roi.height += roi.y-line[k].y; roi.y    = line[k].y;}
					break;
				}
				map.set(line[k].x,line[k].y,true);
//			}
		}
		delete[] line;
	}

	map.setRoi(roi);
	map.set(x,y,true);
//	printf("[ESZ] dilate...\n");
	map.dilate(dilaterad);
//	printf("[ESZ] dilate OK\n");

//	ROS_INFO ("ESZ ROI: %d, %d, %d, %d", roi.x, roi.y = y, roi.width, roi.height);
}


void occupancyGridMap::gateways(const binMap& vz, binMap& newgateways) const {
	uint i,j;
	newgateways.initialize(vz.getWidth(),vz.getHeight(),vz.getResolution(), vz.getXOrigin(), vz.getYOrigin());
	RoI roi = vz.getRoi();
	newgateways.setRoi(roi);
	
	for (i = roi.x ; i < roi.x + roi.width ; i++)
		for (j = roi.y ; j < roi.y + roi.height ; j++)  		 	// for each cell
			if (vz.get(i,j) && (!vz.isInsideD(i,j)) && isfreeD(i,j)) 	// celda en la zona segura
				newgateways.set(i,j,true);
}


int occupancyGridMap::frontiersIn(const binMap& vz,  binMap& newfrontiers) const{
	uint i,j;
	int nfront=0;

	newfrontiers.initialize(vz.getWidth(),vz.getHeight(),vz.getResolution(), vz.getXOrigin(), vz.getYOrigin());
	RoI roi = vz.getRoi();
	newfrontiers.setRoi(roi);

	for (i = roi.x ; i < roi.x + roi.width ; i++)
		for (j = roi.y ; j < roi.y + roi.height; j++)  				// for each cell
			if (isfrontier(i,j) && vz.get(i,j)){ 				// celda en la zona segura
				newfrontiers.set(i,j,true);
				nfront++;			
			}
	return nfront;
}


void occupancyGridMap::clearUnconnect(binMap& vz,uint x, uint y) const{
	RoI roi = vz.getRoi();
	for (uint i = roi.x ; i < roi.x + roi.width ; i++)
		for (uint j = roi.y ; j < roi.y + roi.height ; j++)  		 	// for each cell
			if (!isfree(i,j)) vz.set(i,j,false);
	vz.removeUnconnected(x,y);
}

void occupancyGridMap::occupiedCells(binMap& occupied, int dilateRad) const {
	int i,j;

	occupied.initialize(width,height,resolution, xorigin, yorigin);
	
	for (i = 0 ; i < width ; i++){
		for (j =0 ; j < height ; j++){  		 // for each cell
			if (isoccupied(i,j,dilateRad)){	// celda ocupada
				occupied.set(i,j,true);
			}
		}
	}
}

inline bool occupancyGridMap::isfree(const uint& x, const uint& y, const uint& size) const {
	for (uint i = x-size; i<= x+size; i++)
		for (uint j = y-size; j<= y+size; j++)
			if (!isfree(i,j))
				return false;
	return true;
}

inline bool occupancyGridMap::isfreeNotSafe(const uint& x, const uint& y, const uint& size) const {
	for (uint i = x-size; i<= x+size; i++)
		for (uint j = y-size; j<= y+size; j++)
			if (!isfreeNotSafe(i,j))
				return false;
	return true;
}

bool occupancyGridMap::isfreeD(const uint& x, const uint& y) const {
	uint i,j;

	i = x; j = y;
	if (!isfree(i,j))	return false;

	i = x+1; j = y;
	if (!isfree(i,j))	return false;

	i = x-1; j = y;
	if (!isfree(i,j))	return false;

	i = x; j = y+1;
	if (!isfree(i,j))	return false;

	i = x; j = y-1;
	if (!isfree(i,j))	return false;

	return true;
}

bool occupancyGridMap::isoccupied(const uint& x, const uint& y, const uint& size) const {
	for (uint i = x-size; i<= x+size; i++)
		for (uint j = y-size; j<= y+size; j++)
			if (isoccupied(i,j)) return true;
	return false;
}

float occupancyGridMap::getValue(const uint& x, const uint& y, const uint& range) const {
	float val=0.0f;
	int count=0;	
	for (uint i = x-range; i<= x+range; i++){
		for (uint j = y-range; j<= y+range; j++){
			val+=getValue(i,j);
			count++;
		}
	}
	return val/count;
}

bool occupancyGridMap::isunknown(const uint& x, const uint& y, const uint& size) const {
	for (uint i = x-size; i<= x+size; i++)
		for (uint j = y-size; j<= y+size; j++)
			if (isunknown(i,j))
				return true;
	return false;
}

void occupancyGridMap::saveMapAsImage(const char* file) const {
	IplImage* img = getMapAsImage();
	cvSaveImage(file, img);
	cvReleaseImage(&img);
}

IplImage* occupancyGridMap::getMapAsImage() const {
	IplImage* img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3); 

	for (int i = 0; i< width; i++)
		for(int j = 0; j< height; j++){
			uchar greyvalue = (uchar) (255-2.55f*getValue(i,j));
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+1] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+2] = greyvalue;		
		}
	return img;
}

void occupancyGridMap::showMap(const char* windowname)const{
	IplImage* img = getMapAsImage();
	cvNamedWindow(windowname, 0);
	cvShowImage(windowname,img);
	cvReleaseImage(&img);
}

// TODO: ampliar a que rellene desde varias poses
int occupancyGridMap::countAccessible(const point* p, int numpoints, binMap& accessible) const{
	// N = some floodfill algorithm counting
	//printf("[omap] [countAccessible] npoints :%d, map size (%d, %d)\n", numpoints, getWidth(), getHeight());
	accessible.initialize(getWidth(), getHeight());
	int N=0;
	for (int i=0; i< numpoints; i++){
	//	printf("[omap] [countAccessible] %d\n",i);
		accessible.set(p[i].x,p[i].y,true);
		N++;
		N+= floodfill(p[i].x,p[i].y,accessible);
	}
	//printf("[omap] [countAccessible] result %d\n",N);
	return N;
}

int occupancyGridMap::floodfill(const int& nx, const int& ny, binMap& processed) const{
	int N=0;
	//printf("[TARGETPROBMAPPER]  para el punto (%d, %d), acumulado: %d\n", nx,ny,N);
	//printf("[TARGETPROBMAPPER]  size (%d x %d)\n", processed.getWidth(), processed.getHeight());
	for (int x = nx-1; x <= nx+1; x++){
//		printf("[TARGETPROBMAPPER]  x=%d\n",x);
		if(x < 0 ||  x>= getWidth()-1) continue;
		for (int y = ny-1; y <= ny+1; y++){
//			printf("[TARGETPROBMAPPER]  y=%d\n",y);
			if(y < 0 || y >= getHeight()-1) continue;
//			printf("[TARGETPROBMAPPER]  considering point (%d, %d) processed= %d, is occupied= %d\n", x,y, processed.get(x,y), omap.isoccupied(x,y));
			if(!processed.get(x,y)){
				processed.set(x,y,true);
				if(!isoccupied(x,y)){
//					omap.showMap("proc");
//					printf("[TARGETPROBMAPPER]  iteration ... point (%d, %d) => (%d, %d)\n", nx, ny, x, y);	
//					cvWaitKey(2);
					N += floodfill(x, y, processed)+1;
//					printf("[TARGETPROBMAPPER]  N=%d\n",N);
				}
			}
		}
	}
	return N;
}

