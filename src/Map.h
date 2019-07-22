#pragma once
void InitializeMap();
bool IsInside(double x, double y);
void DrawRobot(double x, double y);
void InitParicle(double x, double y);
double GetClosestWallDistance(double rayOriginX, double rayOriginY, double headAng);
void showUI();
void showMAP();
void Clear();

extern int closestDistance, XMax, XMin, YMax, YMin;
extern int endpoints[14][2];
extern double MAP[320][320];
extern int Maplen;
