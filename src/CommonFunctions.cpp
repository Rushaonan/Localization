//#include "stdafx.h"
#include <iostream>
#include <cmath>
#include "CommonFunctions.h"
#include "MonteCarloLocalization.h"

double a = 8.0 * (p_PI - 3) / (3 * p_PI * (4 - p_PI));
double s_SquareRoot2 = sqrt(2.0);


int Sign(double x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}

double ErrorFunction(double x)
{
    double xSquared = x * x;

    double er = Sign(x) * sqrt(1 - exp(-xSquared * (4.0 / p_PI + a * xSquared) / (1 + a * xSquared)));

    return er;
}


double  DistributionFunction(double x, double mean, double variance)
{
    return 0.5 * (1 + ErrorFunction((x - mean) * 1.0 / (sqrt(variance) * s_SquareRoot2)));
}
