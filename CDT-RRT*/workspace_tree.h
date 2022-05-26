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
#include "primitives.h"
#include "statespace_tree.h"

class WorkSpaceTree
{
    struct Node
    {
        Point pos;
        std::vector<std::shared_ptr<Node>> children;
        std::shared_ptr<Node> parent;
        StateSpaceTree::nodePtr stateNode;
        
        Node(Point coords) : pos(coords), parent(nullptr)
        {}
        
        ~Node()
        {}
        //Links workspace node to a statespace node
        void assignStateNode(const StateSpaceTree::nodePtr& node);
        
    };
    
    typedef std::shared_ptr<Node> nodePtr;
    
//  Workspace boundaries
    Bounds _xBounds;
    Bounds _yBounds;
//  Maximum extension length
    double _maxExtend;
//  Goal coordinates
    Point _goal;
//  Maximum expasion radius
    double _r;
    //  Stores the root node
    StateSpaceTree physical;
    nodePtr _root;
    size_t _card;
    double _lebesgue;
    double _speedInMovement;
    /*-------------------*/
    /*Implement obstacles*/
    /*-------------------*/

//  Samples a random point in the workspace using a uniform distribution
    Point sampleCoords();
    
//  Finds the nearest exisiting node in the tree
    nodePtr findNearest(const nodePtr rand);
    
//  Moves new point towards nearest node
    double moveToNear(nodePtr newnode, const nodePtr near);
    
//  Calculates the vicinity radius for parent candidates
    double getVicinityRadius();
    
    std::vector<nodePtr> getNodesInVicinity(double radius, const Point& target);
    
//  Find least-cost parent in the given list
    std::pair<nodePtr, arcPtr> cheapestParent(const std::vector<nodePtr>& cand, Point target, double targetSpeed);
    
//  Expands the tree by 1 vertice
    nodePtr expand(nodePtr child, double goalSpeed);
    
//  Grows the tree from root to goal
    nodePtr grow();
    
//  Finds a path from root to the given node
    std::deque<Point> trace(nodePtr fin);
    
//  Returns the closest alternative
    static nodePtr chooseNearest(const nodePtr& target, nodePtr& alt1, nodePtr& alt2);
public:
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {goal point}, {allowed deviation from goal}, {speed in intermediate points}
    WorkSpaceTree(Bounds x_bounds, Bounds y_bounds, double mExt, Point start, Point goal, double rad, double speed) :
    _xBounds(x_bounds), _yBounds(y_bounds), _maxExtend(mExt), _goal(goal), _r(rad),
    physical(StatePoint(start.x, start.y, 0, 0, 0)), _root(new Node(start)), _speedInMovement(speed)
    {
        _root->assignStateNode(physical._root);
        _card = 1;
        _lebesgue = (x_bounds.max - x_bounds.min)*(y_bounds.max - y_bounds.min);
    }
    ~WorkSpaceTree()
    {}
//  Finds a path and exports in to JSON
    std::deque<Point> findAndExport();
};
