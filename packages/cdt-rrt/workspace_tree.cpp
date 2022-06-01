//
//  workspace_tree.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 06.05.2022.
//

#include "workspace_tree.h"

using namespace geom;

WorkSpaceTree::WorkSpaceTree(Bounds x_bounds, Bounds y_bounds, double mExt, Point start, geom::Vec2d dirAtStart, Point goal, double rad) :
_xBounds(x_bounds), _yBounds(y_bounds), _maxExtend(mExt), _goal(goal), _r(rad),
physical(StatePoint(start.x, start.y, dirAtStart)), _root(new Node(start))
{
    _root->assignStateNode(physical._root);
    _card = 1;
    _lebesgue = (x_bounds.max - x_bounds.min)*(y_bounds.max - y_bounds.min);
}

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

//Finds the cheapest parent and the path from it
std::pair<WorkSpaceTree::nodePtr, arcPtr> WorkSpaceTree::cheapestParent(const std::vector<nodePtr>& candParents, Point target)
{
    size_t ln = candParents.size();
    nodePtr cheapest = candParents[0];
    std::optional<Arc> fromCheapest = Arc::fromTwoPointsAndTangentalVector(Vec2d(candParents[0]->stateNode->pos.x, candParents[0]->stateNode->pos.y), Vec2d(target.x, target.y), candParents[0]->stateNode->pos.direction);
    double minCost = candParents[0]->stateNode->cost + fromCheapest->getLength();
    for(size_t i = 1; i < ln; ++i)
    {
        std::optional<Arc> arc = Arc::fromTwoPointsAndTangentalVector(Vec2d(candParents[i]->stateNode->pos.x, candParents[i]->stateNode->pos.y), Vec2d(target.x, target.y), candParents[i]->stateNode->pos.direction);
        double cost = candParents[i]->stateNode->cost + arc->getLength();
        if(cost < minCost)
        {
            cheapest = candParents[i];
            fromCheapest = arc;
            minCost = cost;
        }
    }
    
    return {cheapest, arcPtr(&fromCheapest)};
}

//  Expands the tree by 1 vertice
WorkSpaceTree::nodePtr WorkSpaceTree::expand(nodePtr child)
{
//  Calculate the vicinity radius
    double vicinityRadius = getVicinityRadius();
//  Find candidate parent nodes in the vicinity
    std::vector<nodePtr> neighbours = getNodesInVicinity(vicinityRadius, child->pos);
    
//  Find costs and assosiated ss points through arcs
//  Choose the parent point with the least cost
    std::pair<nodePtr, arcPtr> bestParentPair = cheapestParent(neighbours, child->pos);
    //attach the new point to the chosen parent point
    //attach the chosen parent point to the ws tree
    //build a link
    child->stateNode = physical.attach(bestParentPair.first->stateNode,
                                       StatePoint(bestParentPair.second->value().getFinish().x, bestParentPair.second->value().getFinish().y, bestParentPair.first->stateNode->pos.direction.rotate(bestParentPair.second->value().getAngle())),
                                       bestParentPair.second);
    //  Connect to the parent node in the WS tree
    bestParentPair.first->children.push_back(child);
    child->parent = bestParentPair.first;
    neighbours.erase(std::find(neighbours.begin(), neighbours.end(), bestParentPair.first));
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
        cur = expand(rand);
        ++_card;
        dist = auxillary::findSqDistance(Point(cur->stateNode->pos.x, cur->stateNode->pos.y), _goal);
    }
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
