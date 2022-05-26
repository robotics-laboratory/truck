//
//  workspace_tree.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 06.05.2022.
//

#include "workspace_tree.h"
#include <iostream>

void WorkSpaceTree::Node::assignStateNode(const StateSpaceTree::nodePtr& node)
{
    stateNode = node;
}

//  Samples a random point in the workspace using a uniform distribution
Point WorkSpaceTree::sampleCoords()
{
    double x = auxillary::genRand(_xBounds);
    double y = auxillary::genRand(_yBounds);
    return Point(x, y);
}

//  Returns the closest alternative
std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::chooseNearest(const nodePtr& target, nodePtr& alt1, nodePtr& alt2)
{
//      Squared distances to vertices
    double dist1 = auxillary::findSqDistance(target->pos, alt1->pos);
    double dist2 = auxillary::findSqDistance(target->pos, alt2->pos);
//      Compares distances
    if(dist1 > dist2)
    {
        return alt2;
    }
    return alt1;
}

//  Finds the nearest exisiting node in the tree
std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::findNearest(const nodePtr rand)
{
    std::queue<nodePtr> q;
    nodePtr nearest = _root;
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
double WorkSpaceTree::moveToNear(nodePtr newnode, const nodePtr near)
{
    double xdisp = newnode->pos.x - near->pos.x;
    double ydisp = newnode->pos.y - near->pos.y;
    double sqd = xdisp*xdisp + ydisp*ydisp;
    if(sqd > _maxExtend*_maxExtend)
    {
        double rootsqd = sqrt(sqd);
        newnode->pos.x = near->pos.x + _maxExtend * xdisp / rootsqd;
        newnode->pos.y = near->pos.y + _maxExtend * ydisp / rootsqd;
    }
    return auxillary::findDistance(newnode->pos, near->pos);
}
//  Calculates the vicinity radius for parent candidates
double WorkSpaceTree::getVicinityRadius()
{
    if(_card == 1)
    {
        return 3 * _maxExtend;
    }
    double treeMeasure = 2*sqrt(1.5*(_lebesgue/pi)*(log(_card)/_card));
    return std::min(treeMeasure, 3 * _maxExtend);
}
// Gets parent candidates in the vicinity
std::vector<WorkSpaceTree::nodePtr> WorkSpaceTree::getNodesInVicinity(double radius, const Point& target)
{
    radius = radius * radius;
    std::vector<WorkSpaceTree::nodePtr> result;
    std::queue<nodePtr> q;
    q.push(_root);
    while(q.size() > 0)
    {
        Point pt(q.front()->stateNode->pos.x, q.front()->stateNode->pos.y);
        if(auxillary::findSqDistance(target, pt) < radius)
        {
            result.push_back(q.front());
        }
        for(auto ch : q.front()->children)
        {
            q.push(ch);
        }
        q.pop();
    }
    
    return result;
}

std::pair<WorkSpaceTree::nodePtr, arcPtr> WorkSpaceTree::cheapestParent(const std::vector<nodePtr>& candParents, Point target, double targetSpeed)
{
    size_t ln = candParents.size();
    nodePtr cheapest = candParents[0];
    arcPtr fromCheapest(new Arc(candParents[0]->stateNode->pos, target, targetSpeed));
    double minCost = candParents[0]->stateNode->cost + fromCheapest->getCost();
    for(size_t i = 1; i < ln; ++i)
    {
        arcPtr arc(new Arc(candParents[i]->stateNode->pos, target, targetSpeed));
        double cost = candParents[i]->stateNode->cost + arc->getCost();
        if(cost < minCost)
        {
            cheapest = candParents[i];
            fromCheapest = arc;
            minCost = cost;
        }
    }
    
    return {cheapest, fromCheapest};
}

//  Reconnects the neigbours in the vicinity
    void WorkSpaceTree::reconnectNeighbours(std::vector<nodePtr>& neighbours, nodePtr vertex)
    {
        std::cout << vertex->pos.x << ' ' << vertex->pos.y << '\n'; //vertex
        for(nodePtr nb : neighbours)
        {
            std::cout << nb->pos.x << ' ' << nb->pos.y << ' ' << nb->stateNode->cost << ' '; //position and initial cost
            arcPtr arcToNeighbour(new Arc(vertex->stateNode->pos, nb->pos, _speedInMovement));
            
            if (nb->stateNode->cost > vertex->stateNode->cost + arcToNeighbour->getCost())
            {
                std::cout << vertex->stateNode->cost + arcToNeighbour->getCost() << ' '; //alternative cost
                physical.detach(nb->stateNode->parent, nb->stateNode);
                nb->stateNode = physical.attach(vertex->stateNode, arcToNeighbour->getLastPoint(), arcToNeighbour);
                nb->parent->children.erase(std::find(nb->parent->children.begin(), nb->parent->children.end(), nb));
                vertex->children.push_back(nb);
                nb->parent = vertex;
            }
            std::cout << nb->stateNode->cost << '\n'; //final cost
        }
        std::cout << '\n';
    }

//  Expands the tree by 1 vertice
WorkSpaceTree::nodePtr WorkSpaceTree::expand(nodePtr child, double goalSpeed)
{
//  Calculate the vicinity radius
    double vicinityRadius = getVicinityRadius();
//  Find candidate parent nodes in the vicinity
    std::vector<nodePtr> neighbours = getNodesInVicinity(vicinityRadius, child->pos);
    
//  Find costs and assosiated ss points through arcs
//  Choose the parent point with the least cost
    std::pair<nodePtr, arcPtr> bestParentPair = cheapestParent(neighbours, child->pos, goalSpeed);
    //attach the new point to the chosen parent point
    //attach the chosen parent point to the ws tree
    //build a link
    child->stateNode = physical.attach(bestParentPair.first->stateNode, bestParentPair.second->getLastPoint(), bestParentPair.second);
    //  Connect to the parent node in the WS tree
    bestParentPair.first->children.push_back(child);
    child->parent = bestParentPair.first;
    neighbours.erase(std::find(neighbours.begin(), neighbours.end(), bestParentPair.first));
    //reconnect nodes in the vicinity
    reconnectNeighbours(neighbours, child);
    return child;
}

std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::grow()
{
    nodePtr cur = _root;
//      Finds distance to the goal
    double dist = auxillary::findSqDistance(cur->pos, _goal);
//  While current node is not in the goal region
    while(dist > _r*_r)
    {
    //  Samples a new random point
        nodePtr rand(new Node(sampleCoords()));
    //  Finds the nearest existing node
        nodePtr nearest = findNearest(rand);
    //  Moves new point towards nearest node and returns resultant distance
        moveToNear(rand, nearest);
//      Attach the new node to the tree
        cur = expand(rand, _speedInMovement);
        ++_card;
        dist = auxillary::findSqDistance(Point(cur->stateNode->pos.x, cur->stateNode->pos.y), _goal);
    }
    arcPtr end(new Arc(cur->stateNode->parent->pos, Point(cur->stateNode->pos.x, cur->stateNode->pos.y), 0));
    cur->stateNode->fromParent = end;
    cur->parent->stateNode->children[cur->stateNode] = end;
//      Returns the tree node in the goal region
    return cur;
}

//  Finds a path from root to the given node
std::deque<Point> WorkSpaceTree::trace(nodePtr fin)
{
    std::deque<Point> path;
    while(fin != _root)
    {
        path.insert(path.begin(), fin->pos);
        fin = fin->parent;
    }
    path.insert(path.begin(), _root->pos);
    return path;
}

//Finds a path and exports in to JSON
std::deque<Point> WorkSpaceTree::findAndExport()
{
//  Calculate the path
    std::deque<Point> path = trace(grow());
//  Export the path to a JSON file
    nlohmann::json j;
    
//      Store the workspace bounds
    j["x"] = { {"min", _xBounds.min}, {"max", _xBounds.max} };
    j["y"] = { {"min", _yBounds.min}, {"max", _yBounds.max} };
    
//      Store the starting point
    j["start"] = { {"x", _root->pos.x}, {"y", _root->pos.y} };
    
//      Store the goal point
    j["goal"] = { {"x", _goal.x}, {"y", _goal.y} };
    
//      Store the acceptance radius
    j["radius"] = _r;
    
//      Store the tree
    j["tree"] = nlohmann::json::array();
    std::queue<nodePtr> q;
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
        node["position"] = {{"x", cur->pos.x}, {"y", cur->pos.y}};
        node["parent"] = {{"x", cur->parent->pos.x}, {"y", cur->parent->pos.y}};
        j["tree"].push_back(node);
        
    }
    
//      Store the path
    j["path"] = nlohmann::json::array();
    for(auto it = path.begin(); it != path.end(); ++it)
    {
        j["path"].push_back({{"x", it->x}, {"y", it->y}});
    }
    
//      Write the JSON file
    std::ofstream o("info.json");
    o << std::setw(4) << j << std::endl;
    return path;
}
