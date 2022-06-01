//
//  primitives.h
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 17.05.2022.
//

#pragma once
#include <cmath>
#include <queue>
#include <random>

//The pi number
const double pi = 3.1415926535897932384626433;

//Workspace primitives
struct Bounds
{
    double min;
    double max;
    Bounds(double inp_min, double inp_max) : min(inp_min), max(inp_max){}
};

struct Point
{
    double x;
    double y;
    Point(double inp_x, double inp_y) : x(inp_x), y(inp_y){}
};

// Workspace auxiliary methods
namespace auxiliary
{
//  Samples a pseudorandom integer in a given range
    double genRand(const Bounds& bounds);
//  Returns squared Euclidean distance between two points
    double findSqDistance(const Point& first, const Point& second);
//  Returns Euclidean distance between two points
    double findDistance(const Point& first, const Point& second);
}

// Statespace Primitives
struct StatePoint
{
    double x;
    double y;
    double theta;
    double delta;
    double v;
    StatePoint(double inp_x, double inp_y, double inp_t, double inp_d, double inp_v) :
    x(inp_x), y(inp_y), theta(inp_t), delta(inp_d), v(inp_v){}
};

struct Arc
{
    double radius;
    StatePoint start;
    Point goal;
    double goalV;
    int numInter; //number of intermediate points, equals 11 by default
//  Finds 'numInter' equidistant intermediate points on the trajectory
    std::queue<StatePoint> getIntermediate();
    
//  Returns the cost of the arc (in terms of Euclidean distance)
    double getCost();
    StatePoint getLastPoint();
    Arc(StatePoint inp_start, Point inp_end, double goalSpeed) : start(inp_start), goal(inp_end), goalV(goalSpeed), numInter(11)
    {
        radius = auxiliary::findDistance(Point(start.x, start.y), goal) / 2;
    }
};

typedef std::shared_ptr<Arc> arcPtr;
