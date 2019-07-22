#ifndef	 _DRIVE_H_
#define  _DRIVE_H_

#include "MonteCarloLocalization.h"

Pose MoveExact(int i);
void* GetMeasurements(double *measurement, Pose nextPose);

class Move {
public:
	int v;
	int omega;
	int time;
};

extern int Sensor[5];
extern int SenLen;
extern int loop;
extern Move DriveCommand[87];
extern Pose RobotPose[100];
extern int DCLen;

#endif
