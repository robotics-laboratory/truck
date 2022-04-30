//
//  workspace_tree.h
//  Workspace Tree
//  2D RRT
//  CDT-RRT*
//  Assumes the workspace being parametrised into integer coordinates
//
//  Created by Konstantin Shashkov on 13.04.2022.
//
#include <vector>
#include <queue>
#include <random>
#include <cmath>
#include <algorithm>
#include <deque>
#include <fstream>
#include "json.hpp"
#ifndef workspace_tree_h
#define workspace_tree_h

class WorkSpaceTree
{
//  Maximum extension length
    int _maxExtend;
//  Workspace boundaries
    std::pair<int,int> _xBounds;
    std::pair<int,int> _yBounds;
    std::pair<int,int> _goal;
    int _r;
    
    /*-------------------*/
    /*Implement obstacles*/
    /*-------------------*/
//  Samples a pseudorandom integer in a given range
    static int genRand(std::pair<int,int> bounds)
    {
        std::random_device device;
        std::mt19937 generator(device());
        std::uniform_int_distribution<> dist(bounds.first, bounds.second);
        return dist(generator);
    }
    
//  Samples a random point in the workspace using a uniform distribution
    static std::pair<int, int> sampleCoords(const std::pair<int,int>& x, const std::pair<int,int>& y)
    {
        std::pair<int, int> result;
        result.first = genRand(x);
        result.second = genRand(y);
        return result;
    }

//  Node structure
    struct Node
    {
        //Coordinates
        std::pair<int,int> pos;
        //Children
        std::vector<Node*> children;
        //Parent
        Node* parent;
        
        Node(std::pair<int, int> coords) : pos(coords), parent(nullptr)
        {
        }
        
        ~Node()
        {
//          Recursively destroys children before destroying the node
            for(Node* nd : children)
            {
                delete nd;
            }
        }
    };

public:
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {goal point}, {allowed deviation from goal}
    WorkSpaceTree(std::pair<int, int> x, std::pair<int, int> y, int mExt, std::pair<int, int> init, std::pair<int, int> goal, int rad) :
    _xBounds(x), _yBounds(y), _maxExtend(mExt), _root(new Node(init)), _goal(goal), _r(rad)
    {}
    
    ~WorkSpaceTree()
    {
//      Destroys the root and recursively its children
        delete _root;
    }
//  Exports a node to json
    
private:
//  Returns squared Euclidean distance between two points
    static int findSqDistance(const std::pair<int,int>& first, const std::pair<int,int>& second)
    {
        int xdisp = first.first - second.first;
        int ydisp = first.second - second.second;
        return xdisp*xdisp + ydisp*ydisp;
    }
    
//  Returns the closest alternative
    static Node* chooseNearest(const Node* target, Node* alt1, Node* alt2)
    {
//      Squared distances to vertices
        int dist1 = findSqDistance(target->pos, alt1->pos);
        int dist2 = findSqDistance(target->pos, alt2->pos);
//      Compares distances
        if(dist1 > dist2)
        {
            return alt2;
        }
        return alt1;
    }
    
//  Finds the nearest exisiting node in the tree
    Node* findNearest(const Node* rand)
    {
        std::queue<Node*> q;
        Node* nearest = _root;
        q.push(_root);
        while(q.size() > 0)
        {
            nearest = chooseNearest(rand, nearest, q.front());
            for(auto ch : q.front()->children)
            {
                q.push(ch);
            }
            q.pop();
        }
        return nearest;
    }
    
//  Moves new point towards nearest node
    int moveToNear(Node* newnode, const Node* near)
    {
        int xdisp = newnode->pos.first - near->pos.first;
        int ydisp = newnode->pos.second - near->pos.second;
        int sqd = xdisp*xdisp + ydisp*ydisp;
        if(sqd > _maxExtend*_maxExtend)
        {
            newnode->pos.first = near->pos.first + _maxExtend * xdisp / sqrt(sqd);
            newnode->pos.second = near->pos.second + _maxExtend * ydisp / sqrt(sqd);
        }
        return sqrt(findSqDistance(newnode->pos, near->pos));
    }
    
//  Expands the tree
    Node* expand()
    {
//      Samples a new random point
        Node* rand = new Node(sampleCoords(_xBounds, _yBounds));
//      Finds the nearest existing node
        Node* nearest = findNearest(rand);
//      Moves new point towards nearest node and returns resultant distance
//      Note sure if I need the distance yet
        int sqdToNear = moveToNear(rand, nearest);
        
        /*----------------------*/
        /*Find parent candidates*/
        /*Choose parent---------*/
        /*Connect to parent-----*/
        /*RRT* part TBA---------*/
        /*----------------------*/
        
//      Placeholder connection to physically nearest node
        nearest->children.push_back(rand);
        rand->parent = nearest;
        return rand;
    }
    
//  Grows the tree from root to goal
    Node* grow()
    {
        Node* cur = _root;
//      Finds distance to the goal
        int dist = findSqDistance(cur->pos, _goal);
//      While current node is not in the goal region
        while(dist > _r*_r)
        {
//          Expand the tree by 1 node and recalculate the distance
            cur = expand();
            dist = findSqDistance(cur->pos, _goal);
            
        }
//      Returns the tree node in the goal region
        return cur;
    }
    
//  Finds a path from root to the given node
    std::deque<std::pair<int,int>> trace(Node* fin)
    {
        std::deque<std::pair<int,int>> path;
        while(fin != _root)
        {
            path.insert(path.begin(), fin->pos);
            fin = fin->parent;
        }
        path.insert(path.begin(), _root->pos);
        return path;
    }
    
//  Exports the tree and the given path to a JSON file
    void exp(const std::deque<std::pair<int,int>>& path)
    {
        nlohmann::json j;
        
//      Store the workspace bounds
        j["x"] = { {"min", _xBounds.first}, {"max", _xBounds.second} };
        j["y"] = { {"min", _yBounds.first}, {"max", _yBounds.second} };
        
//      Store the starting point
        j["start"] = { {"x", _root->pos.first}, {"y", _root->pos.second} };
        
//      Store the goal point
        j["goal"] = { {"x", _goal.first}, {"y", _goal.second} };
        
//      Store the acceptance radius
        j["radius"] = _r;
        
//      Store the tree
        j["tree"] = nlohmann::json::array();
        std::queue<Node*> q;
        q.push(_root);
        while(q.size() > 0)
        {
            auto cur = q.front();
            for(auto ch : cur->children)
            {
                q.push(ch);
            }
            q.pop();
            
            if(cur->parent == nullptr)
            {
                continue;
            }
            nlohmann::json node;
            node["position"] = {{"x", cur->pos.first}, {"y", cur->pos.second}};
            node["parent"] = {{"x", cur->parent->pos.first}, {"y", cur->parent->pos.second}};
            j["tree"].push_back(node);
            
        }
        
//      Store the path
        j["path"] = nlohmann::json::array();
        for(auto it = path.begin(); it != path.end(); ++it)
        {
            j["path"].push_back({{"x", it->first}, {"y", it->second}});
        }
        
//      Write the JSON file
        std::ofstream o("info.json");
        o << std::setw(4) << j << std::endl;
    }
    
//  Stores the root node
    Node* _root;
    
public:
//  Finds a trajectory from root to goal
    std::deque<std::pair<int,int>> getTrajectory()
    {
//      Returns a queue of coordinate pairs
        auto path = trace(grow());
        exp(path); //not redy yet
        return path;
    }
};

#endif /* workspace_tree_h */
