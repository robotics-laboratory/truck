//
//  primitives.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 17.05.2022.
//

#include "primitives.h"

//  Samples a pseudorandom integer in a given range
double auxillary::genRand(const Bounds& bounds)
{
    std::random_device device;
    std::mt19937 generator(device());
    std::uniform_real_distribution<> dist(bounds.min, bounds.max);
    return dist(generator);
}

//  Returns squared Euclidean distance between two points
double auxillary::findSqDistance(const Point& first, const Point& second)
{
    double xdisp = first.x - second.x;
    double ydisp = first.y - second.y;
    return xdisp*xdisp + ydisp*ydisp;
}

//  Returns Euclidean distance between two points
double auxillary::findDistance(const Point& first, const Point& second)
{
    return sqrt(findSqDistance(first, second));
}

//  Finds 'numInter' equidistant intermediate points on the trajectory
std::queue<StatePoint> Arc::getIntermediate()
{
    std::queue<StatePoint> intermediate = {};
//  arc's centre point in the workspace (Euclidean plane)
    Point centre((start.x + goal.x)/2, (start.y + goal.y)/2);
//  angle of the arc's base diameter to the x-axis
    double baseAngle = atan((goal.y - start.y)/(goal.x - start.x));
//  increment angle of intermediate points
    double incrAngle = pi/(numInter + 1);
    double baseSpeed = start.v;
    double incrSpeed = (goalV - start.v)/(numInter + 1);
    if(start.x < goal.x)
    {
        baseAngle += pi;
    }
    double initTheta = baseAngle + pi/2;
    if(initTheta > 2*pi)
    {
        initTheta -= 2*pi;
    }
    for(size_t i = 1; i <= numInter; ++i)
    {
        baseAngle += incrAngle;
        if(baseAngle > 2*pi)
        {
            baseAngle -= 2*pi;
        }
        baseSpeed += incrSpeed;
        double x_coord = centre.x + cos(baseAngle)*radius;
        double y_coord = centre.y + sin(baseAngle)*radius;
        double dir = baseAngle + pi/2;
        if(dir > 2*pi)
        {
            dir -= 2*pi;
        }
        double wheelAngle = atan((goal.y - y_coord)/(goal.x - x_coord));
        wheelAngle -= dir;
        if(x_coord > goal.x)
        {
            wheelAngle += pi;
        }
        if(wheelAngle < 0)
        {
            wheelAngle += 2*pi;
        }
        intermediate.push(StatePoint(x_coord, y_coord, dir, wheelAngle, baseSpeed));
    }
    initTheta += pi;
    if(initTheta > 2*pi)
    {
        initTheta -= 2*pi;
    }
//  Push the end point (goal)
    intermediate.push(StatePoint(goal.x, goal.y, initTheta, 0, goalV));
    return intermediate;
}

double Arc::getCost()
{
    return pi * radius;
}
