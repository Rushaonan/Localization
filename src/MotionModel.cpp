//#include "stdafx.h"
#include <cmath>
#include "MotionModel.h"
#include "BeamModel.h"
#include "MonteCarloLocalization.h"
#include "Robot.h"
#include "SamlperNormal.h"

double A[6] = { 0.01, 0.001, 0.001, 0.01, 0.01, 0.01 };

/*************************************************
// Method: Sample
// Description: Algorithm sample_motion_model_velocity(ut, x(t-1))
//              Probabilistic Robotics P93
// Author: RSN
// Date: 2019/05/21
// Returns: Pose
// Parameter: particle
// History:
*************************************************/
Pose Sample(Pose particle)
{
    double velocity = DriveCommand[loop].v;
    double omegaRad = ang2rad(DriveCommand[loop].omega);
    double DeltaTime = DriveCommand[loop].time;

    double headRad = particle.rad();

    double vSquared = velocity * velocity;
    double omegaRadSquared = omegaRad * omegaRad;

    double vVariance = A[0] * vSquared + A[1] * omegaRadSquared;
    double vNoisy = velocity + Sampler(vVariance);

    double omegaVariance = A[2] * vSquared + A[3] * omegaRadSquared;
    double omegaRadNoisy = omegaRad + Sampler(omegaVariance);

    double gammaVariance = A[4] * vSquared + A[5] * omegaRadSquared;
    double gammaNoisy = Sampler(gammaVariance);

    double vOverOmegaNoisy = vNoisy * 1.0 / omegaRadNoisy;
    
    Pose newPose = {0,0,0};

    newPose.x = particle.x - vOverOmegaNoisy * sin(headRad) 
                  +  vOverOmegaNoisy * sin(headRad + omegaRadNoisy * DeltaTime);

    newPose.y = particle.y + vOverOmegaNoisy * cos(headRad) 
                  - vOverOmegaNoisy * cos(headRad + omegaRadNoisy * DeltaTime);

    double newThetaRad = headRad + omegaRadNoisy * DeltaTime + gammaNoisy * DeltaTime;

    newPose.theta = rad2ang(newThetaRad);
    
    return newPose;
}

    
/*************************************************
// Method: poseSample
// Description: Particles sample
// Author: RSN
// Date: 2019/05/21
// Returns: Pose
// Parameter: currentPose
// History:
*************************************************/
Pose poseSample(Pose currentPose)
{
    if (loop == -1)
        return currentPose;
    else
        return Sample(currentPose);
}
