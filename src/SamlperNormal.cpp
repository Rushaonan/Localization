//#include "stdafx.h"
#include <iostream>
#include <cmath>
#include "SamlperNormal.h"
#include "BeamModel.h"


double GetRandomInRange(double b)
{
    return GenerateRandom() * 2 * b - b;
}


double Sampler(double variance)
{
    double b = sqrt(variance);
    double result = 0.0;
    int c_AggregationCount = 12;

    for (int i=0; i<c_AggregationCount; i++)
        result += GetRandomInRange(b);
    return 0.5 * result;
}
