#pragma once
void BeamModel(int maxRange, double measurementVariance, double lambdaShort, double deltar);
double GetSample(double realDistance);
double GenerateRandom();
double Getprobability(double robotToWall, double particleToWall, double deltaR);
extern int MaxRange;
extern double LambdaShort;
extern double ZHit, ZShort, ZMax, ZRand;
