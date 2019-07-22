//#include "stdafx.h"
#include <iostream>
#include "WeighingFactors.h"
#include "BeamModel.h"

double ZHitRaw=0, ZShortRaw=0, ZMaxRaw=0, ZRandRaw=0;

void Normalize()
{
   
    double total = ZHitRaw + ZShortRaw + ZMaxRaw + ZRandRaw;

    ZHit = ZHitRaw / total;
    ZShort = ZShortRaw / total;
    ZMax = ZMaxRaw / total;
    ZRand = ZRandRaw / total;
}

void WeighingFactor(double zHit, double zShort, double zMax, double zRand)
{
    ZHitRaw = zHit;
    ZShortRaw = zShort;
    ZMaxRaw = zMax;
    ZRandRaw = zRand;
    Normalize();
}
