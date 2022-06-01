//
//  primitives.h
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 17.05.2022.
//

#pragma once
#include <cmath>
#include <queue>
#include <deque>
#include <random>
#include "geom/arc.hpp"
#include "geom/vector.hpp"

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
    geom::Vec2d direction;
    StatePoint(double pos_x, double pos_y, geom::Vec2d dir) :
    x(pos_x), y(pos_y), direction(dir / dir.len()) {}
};

typedef std::shared_ptr<std::optional<geom::Arc>> arcPtr;
