#ifndef	 _POINTS_H_
#define  _POINTS_H_

void MonteCarloLocalization(int particleCount);
double ang2rad(double degree);
double rad2ang(double rad);
void update(double measurements[5]);

#define p_PI 3.1415926

class Pose{
public:
  double x;
  double y;
  double theta;
  
public:
  void printX();
  double rad();  // change angle to radius 
};

extern int ParticlesCount;
extern double DeltaR;
extern double m_AggregatedWeights[10000];
extern Pose Particles[10000];

#endif