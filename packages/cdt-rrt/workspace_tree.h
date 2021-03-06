//
//  wstree.h
//  Workspace Tree
//  2D RRT
//  CDT-RRT*
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
//  Node structure
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
//  Stores the state space tree
    StateSpaceTree physical;
//  Stores the root node
    nodePtr _root;
//  Number of vertices
    size_t _card;
//  Area of the free work space
    double _lebesgue;

//  Samples a random point in the workspace using a uniform distribution
    Point sampleCoords();
    
//  Finds the nearest exisiting node in the tree
    nodePtr findNearest(const nodePtr rand);
    
//  Moves new point towards nearest node
    double moveToNear(nodePtr newnode, const nodePtr near);
    
//  Calculates the vicinity radius for parent candidates
    double getVicinityRadius();
    
//  Finds all vertices in the given vicinity
    std::vector<nodePtr> getNodesInVicinity(double radius, const Point& target);
    
//  Finds the least-cost parent in the given list
    std::pair<nodePtr, arcPtr> cheapestParent(const std::vector<nodePtr>& cand, Point target);
    
//  Reconnects the neigbours in the vicinity
    void reconnectNeighbours(std::vector<nodePtr>& neighbours, nodePtr vertex);
    
//  Expands the tree by 1 vertice
    nodePtr expand(nodePtr child);
    
//  Grows the tree from root to goal
    nodePtr grow();
    
//  Finds a path from root to the given node
    std::deque<Point> trace(nodePtr fin);
    
//  Returns the closest alternative
    static nodePtr chooseNearest(const nodePtr& target, nodePtr& alt1, nodePtr& alt2);
public:
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {tangental vector at start}, {goal point}, {allowed deviation from goal}
    WorkSpaceTree(Bounds x_bounds, Bounds y_bounds, double mExt, Point start, geom::Vec2d dirAtStart, Point goal, double rad);
    ~WorkSpaceTree()
    {}
//  Finds a path and exports in to JSON
    std::deque<Point> findAndExport();
};
