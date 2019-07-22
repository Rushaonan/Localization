//#include "stdafx.h"
#include <iostream>
#include <cmath>
#include "Robot.h"
#include "Map.h"
#include "BeamModel.h"
#include "MonteCarloLocalization.h"


Move DriveCommand[87] = {
// 第三段直线 3 * 5 = 15
	{ 5,0,1 },
	{ 5,0,1 },
	{ 5,0,1 },
// 第一次转向	左 90°
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
	{ 5,10,1 },
// 第三段直线 5
	{ 5,0,1 },
// 第二次转向 右 90°
	{ 5,-30,1 },
	{ 5,-30,1 },
	{ 5,-30,1 },
// 第三段直线
	{ 5,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 7,0,1 },
// 第三次转向 左 90°
	{ 5,30,1 },
	{ 5,30,1 },
	{ 5,30,1 },
// 第四段直线
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 5,0,1 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 5,0,1 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
// 第四次转向 左 90°
	{ 5,30,1 },
	{ 5,30,1 },
	{ 5,30,1 },
// 第五段直线
	{ 6,0,2 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 5,0,2 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 7,0,1 },
	{ 7,0,1 },
	{ 8,0,2 },
	{ 8,0,2 },
	{ 6,0,1 },
	{ 7,0,1 },
	{ 8,0,1 },
	{ 5,0,1 },
	{ 5,0,1 },
	{ 5,0,1 },
// 第 5 次转向 右 90°
	{ 5,-30,1 },
	{ 5,-30,1 },
	{ 5,-30,1 },
// 第 6 段直线
	{ 5,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
// 第 6 次转向 右 90°
	{ 5,-30,1 },
	{ 5,-30,1 },
	{ 5,-30,1 },
// 第 8 段直线
	{ 5,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
	{ 6,0,1 },
};
int DCLen = sizeof(DriveCommand) / sizeof(Move);
int Sensor[5] = { -120, -60, 0, 60, 120 };
int SenLen = sizeof(Sensor) / sizeof(int);
Pose RobotPose[100] = { 0 };
int loop = -1;


/*************************************************
// Method: MoveExact
// Description: Move the Robot
// Author: RSN
// Date: 2019/05/21
// Returns: Pose: Next position of the Robot
// Parameter: step
// History:
*************************************************/
Pose MoveExact(int step)
{
	if (step == -1)
	{
		return RobotPose[0];
	}
	else
	{
		Pose newRP; // new Robot Position
		double currentX = RobotPose[step].x;
		double currentY = RobotPose[step].y;
		double newThetaRad = ang2rad(RobotPose[step].theta);

		if (DriveCommand[step].omega != 0)
		{
			double vOverOmegaRad = DriveCommand[step].v * 1.0 / ang2rad(DriveCommand[step].omega);
			newRP.x = currentX - vOverOmegaRad * sin(newThetaRad)
				+ vOverOmegaRad * sin(newThetaRad + ang2rad(DriveCommand[step].omega) * DriveCommand[step].time);
			newRP.y = currentY + vOverOmegaRad * cos(newThetaRad)
				- vOverOmegaRad * cos(newThetaRad + ang2rad(DriveCommand[step].omega) * DriveCommand[step].time);
			newThetaRad = newThetaRad + ang2rad(DriveCommand[step].omega) * DriveCommand[step].time;
		}
		else
		{
			double distanceTravel = DriveCommand[step].v * DriveCommand[step].time;
			newRP.x = currentX + distanceTravel * cos(newThetaRad);
			newRP.y = currentY + distanceTravel * sin(newThetaRad);
		}

		newRP.theta = rad2ang(newThetaRad);

		RobotPose[step + 1] = newRP;
		//DrawRobot(newRP.x, newRP.y);

		return newRP;
	}
}


/*************************************************
// Method: GetMeasurements
// Description: Measuring distances of the robot from the 5 directions of the wall;
// Author: RSN
// Date: 2019/05/21
// Returns: void*
// Parameter: measurement[5]
// Parameter: nextPose
// History:
*************************************************/
void* GetMeasurements(double *measurement, Pose nextPose)
{
    double realDistanceToWall;
    for (int i=0; i<SenLen; i++)
    {
        double ThetaAddSensor =nextPose.theta + Sensor[i];
        realDistanceToWall = GetClosestWallDistance(nextPose.x, nextPose.y, ThetaAddSensor);
        measurement[i] = GetSample(realDistanceToWall);
    }
	return 0;
}

