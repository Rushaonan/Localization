//#include "stdafx.h"
#include <iostream>
#include <limits.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Map.h"
#include "MonteCarloLocalization.h"
#include "Robot.h"
using namespace cv;

void InitializeMinsAndMaxs();

#define DBL_MIN 2.2250738585072014e-308 /* min positive value */

double MAP[320][320] = { 0 };

int endpoints[14][2] = 
{
  {40, 40},
  {110, 40},
  {110, 60},
  {260, 60},
  {260, 130},
  {280, 130},
  {280, 230},
  {260, 230},
  {260, 280},
  {40, 280},
  {40, 190},
  {200, 190},
  {200, 110},
  {40, 110}
};

int Maplen = sizeof(endpoints) / sizeof(endpoints[0]);

int XMin = INT_MAX;
int XMax = INT_MIN;
int YMin = INT_MAX;
int YMax = INT_MIN;


/*************************************************
// Method: InitializeMap
// Description: Draw Map
// Author: RSN
// Date: 2019/05/20
// Returns: void
// History:
*************************************************/
void InitializeMap()
{
  
 //point1 = MapPoints[i]
 //point2 = MapPoints[(i + 1) % len(MapPoints)]

  for (int point=0; point<Maplen; point++)
  {
    int flags, next = (point+1)%Maplen;
    if (endpoints[point][1] == endpoints[next][1])
    {
      flags = (endpoints[point][0]-endpoints[next][0]) < 0 ? 1 : -1;
      for(int i=endpoints[point][0]; i!=endpoints[next][0]+flags; i+=flags)
      { 
	MAP[i][endpoints[point][1]] = 1;
      }
    }

    else if (endpoints[point][0] == endpoints[next][0])
    {
      flags = (endpoints[point][1]-endpoints[next][1]) < 0 ? 1 : -1;
      for(int i=endpoints[point][1]; i!=endpoints[next][1]+flags; i+=flags)
      {
	MAP[endpoints[point][0]][i] = 1;
      }
    }
   }
  InitializeMinsAndMaxs();
}

/*************************************************
// Method: InitializeMinsAndMaxs
// Description: Calculate the minimum and the maximum values of the map
// Author: RSN
// Date: 2019/05/20
// Returns: void
// History:
*************************************************/
void InitializeMinsAndMaxs()
{
  for (int i = 0; i < Maplen; i++)
  {
     if (endpoints[i][0] < XMin)  XMin = endpoints[i][0];
     if (endpoints[i][0] > XMax)  XMax = endpoints[i][0];
     if (endpoints[i][1] < YMin)  YMin = endpoints[i][1];
     if (endpoints[i][1] > YMax)  YMax = endpoints[i][1];
  }
}

/*************************************************
// Method: IsInside
// Description: Determine whether the particle are on the map
// Author: RSN
// Date: 2019/05/20
// Returns: bool
// Parameter: x
// Parameter: y
// History:
*************************************************/
bool IsInside(double x, double y)
{

	bool isIn = false;
	for (int i = 0; i < Maplen; i++)
	{
		int j = (i + (Maplen)-1) % (Maplen);
		if (
			((endpoints[i][1] > y) != (endpoints[j][1] > y))
			&& (x < (endpoints[j][0] - endpoints[i][0]) * (y - endpoints[i][1]) / (endpoints[j][1] - endpoints[i][1]) + endpoints[i][0])
			)
			isIn = !isIn;
	}
	return isIn;
}


/*************************************************
// Method: IntersectsAtDistanceDegree
// Description: From one point along the theta emits rays, 
//              intersects the line segment, and calculates the length of the rays.
// Author: RSN
// Date: 2019/05/20
// Returns: double distance: ray's length
// Parameter: rayOriginX
// Parameter: rayOriginY
// Parameter: headAng
// Parameter: point1: The coordinates of the line segment start
// Parameter: point2: The coordinates of the line segment end
// History:
*************************************************/
double IntersectsAtDistanceDegree(double rayOriginX, double rayOriginY, double headAng, int point1[2], int point2[2])
{
	/*
	As mentioned by others, the 2D problem may be solved by
	equating the parametric equations. The ray is P0+s*D0, where
	P0 is the ray origin, D0 is a direction vector (not necessarily
	unit length), and s >= 0. The segment is P1+t*D1, where P1
	and P1+D1 are the endpoints, and 0 <= t <= 1. Equating, you
	have P0+s*D0 = P1+t*D1. Define perp(x,y) = (y,-x). Then
	if Dot(perp(D1),D0) is not zero,
	s = Dot(perp(D1),P1-P0)/Dot(perp(D1),D0)
	t = Dot(perp(D0),P1-P0)/Dot(perp(D1),D0)
	The s and t value are where the *lines* containing the ray
	and segment intersect. The ray and segment intersect as
	long as s >= 0 and 0 <= t <= 1. If Dot(perp(D1),D0) is zero,
	then the ray and segment are parallel. If they do not lie on
	the same line, there is no intersection. If they do lie on the
	same line, you have to test for overlap, which is a 1D problem.
	*/

	double cosTheta, sinTheta;
	cosTheta = cos(ang2rad(headAng));
	sinTheta = sin(ang2rad(headAng));
	double deltaX = point2[0] - point1[0];
	double deltaY = point2[1] - point1[1];

	if (fabs(deltaY * cosTheta - deltaX * sinTheta) < DBL_MIN)
		return -1;
	else
	{
		double s = (deltaY * (point1[0] - rayOriginX) - deltaX * (point1[1] - rayOriginY))
			/ (deltaY * cosTheta - deltaX * sinTheta);
		double t = (sinTheta * (point1[0] - rayOriginX) - cosTheta * (point1[1] - rayOriginY))
			/ (deltaY * cosTheta - deltaX * sinTheta);
		if (s < 0)
			return -1;
		else if (t < 0 || t > 1)
			return -1;
		else
			return s;
	}
}


/*************************************************
// Method: GetClosestWallDistance
// Description: Calculating the most closet distance to the wall in 5 oriented.
// Author: RSN
// Date: 2019/05/20
// Returns: double The shortest distance
// Parameter: rayOriginX
// Parameter: rayOriginY
// Parameter: headAng
// History:
*************************************************/
double GetClosestWallDistance(double rayOriginX, double rayOriginY, double headAng)
{
	int closestDistance = 500;
	int point1[2], point2[2];
	double distance;

	for (int i = 0; i<Maplen; i++)
	{
		point1[0] = endpoints[i][0];
		point1[1] = endpoints[i][1];
		point2[0] = endpoints[(i+1)%Maplen][0];
		point2[1] = endpoints[(i+1)%Maplen][1];

		distance = IntersectsAtDistanceDegree(rayOriginX, rayOriginY, headAng, point1, point2);

		if (distance < 0)
			continue;
		if (distance < closestDistance)
			closestDistance = distance;
	}
	return closestDistance;
}


void DrawRobot(double x, double y)
{
  int X, Y;
  X = ceil(x);
  Y = ceil(y);
  MAP[X][Y] = 1;
  MAP[X-1][Y-1] = 1;
  MAP[X-1][Y+1] = 1;
  MAP[X+1][Y-1] = 1;
  MAP[X+1][Y+1] = 1;
}

void InitParicle(double x, double y)
{
  int X,Y;
  X = ceil(x);
  Y = ceil(y);
  MAP[X][Y] = 0.4;
}


void showMAP()
{
  Mat frame = Mat(320, 320, CV_64FC1, MAP);
  Mat bigImage;
  resize(frame, bigImage, Size( frame.cols * 2, frame.rows * 2));
  
  imshow("Map", bigImage);
  cvMoveWindow( "Map", 50, 50 );
  waitKey(50);
}

void Clear()
{
	int X, Y;
	for (int i = 0; i < ParticlesCount; i++)
	{
		X = ceil(Particles[i].x);
		Y = ceil(Particles[i].y);
		MAP[X][Y] = 0;
	}
}

void showUI()
{
  for (int i=0; i <ParticlesCount; i++)
    InitParicle(Particles[i].x, Particles[i].y);
  DrawRobot(RobotPose[loop+1].x, RobotPose[loop+1].y);
  showMAP();
  Clear();
}
