//
//  primitives.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 17.05.2022.
//

#include "primitives.h"

//  Samples a pseudorandom integer in a given range
double auxiliary::genRand(const Bounds& bounds)
{
    std::random_device device;
    std::mt19937 generator(device());
    std::uniform_real_distribution<> dist(bounds.min, bounds.max);
    return dist(generator);
}

//  Returns squared Euclidean distance between two points
double auxiliary::findSqDistance(const Point& first, const Point& second)
{
    double xdisp = first.x - second.x;
    double ydisp = first.y - second.y;
    return xdisp*xdisp + ydisp*ydisp;
}

//  Returns Euclidean distance between two points
double auxiliary::findDistance(const Point& first, const Point& second)
{
    return sqrt(findSqDistance(first, second));
}
