//
//  wstree.h
//  Workspace Tree
//  2D RRT
//  CDT-RRT*
//  Assumes the workspace being parametrised into integer coordinates
//
//  Created by Konstantin Shashkov on 13.04.2022.
//
#pragma once
#include <vector>
#include <queue>
#include <random>
#include <cmath>
#include <algorithm>
#include <deque>
#include <fstream>
#include "json.hpp"

//Primitives
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

class WorkSpaceTree
{
    struct Node
    {
        Point pos;
        std::vector<std::shared_ptr<Node>> children;
        std::shared_ptr<Node> parent;
        
        Node(Point coords) : pos(coords), parent(nullptr)
        {}
        
        ~Node()
        {}
    };
    
//  Maximum extension length
    double _maxExtend;
//  Workspace boundaries
    Bounds _xBounds;
    Bounds _yBounds;
//  Goal coordinates
    Point _goal;
//  Maximum expasion radius
    double _r;
    //  Stores the root node
    std::shared_ptr<Node> _root;
    
    /*-------------------*/
    /*Implement obstacles*/
    /*-------------------*/

//  Samples a random point in the workspace using a uniform distribution
    Point sampleCoords();
    
//  Finds the nearest exisiting node in the tree
    std::shared_ptr<Node> findNearest(const std::shared_ptr<Node> rand);
    
//  Moves new point towards nearest node
    double moveToNear(std::shared_ptr<Node> newnode, const std::shared_ptr<Node> near);
    
//  Expands the tree by 1 vertice
    std::shared_ptr<Node> expand();
    
//  Grows the tree from root to goal
    std::shared_ptr<Node> grow();
    
//  Finds a path from root to the given node
    std::deque<Point> trace(std::shared_ptr<Node> fin);
    
//  Returns the closest alternative
    static std::shared_ptr<Node> chooseNearest(const std::shared_ptr<Node>& target, std::shared_ptr<Node>& alt1, std::shared_ptr<Node>& alt2);


public:
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {goal point}, {allowed deviation from goal}
    WorkSpaceTree(Bounds x_bounds, Bounds y_bounds, double mExt, Point start, Point goal, double rad) :
    _xBounds(x_bounds), _yBounds(y_bounds), _maxExtend(mExt), _root(new Node(start)), _goal(goal), _r(rad)
    {}
    
    ~WorkSpaceTree()
    {}

//  Finds a path and exports in to JSON
    std::deque<Point> findAndExport();
};


//Auxillary methods
namespace WorkSpaceAuxillary{

    //  Samples a pseudorandom integer in a given range
    double genRand(const Bounds& bounds);
        
    //  Returns squared Euclidean distance between two points
    double findSqDistance(const Point& first, const Point& second);
}
