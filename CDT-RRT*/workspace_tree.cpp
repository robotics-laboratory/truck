//
//  workspace_tree.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 06.05.2022.
//

#include "workspace_tree.h"

//  Samples a random point in the workspace using a uniform distribution
Point WorkSpaceTree::sampleCoords()
{
    double x = auxillary::genRand(_xBounds);
    double y = auxillary::genRand(_yBounds);
    return Point(x, y);
}

//  Returns the closest alternative
std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::chooseNearest(const std::shared_ptr<Node>& target, std::shared_ptr<Node>& alt1, std::shared_ptr<Node>& alt2)
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
std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::findNearest(const std::shared_ptr<Node> rand)
{
    std::queue<std::shared_ptr<Node>> q;
    std::shared_ptr<Node> nearest = _root;
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
double WorkSpaceTree::moveToNear(std::shared_ptr<Node> newnode, const std::shared_ptr<Node> near)
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

//  Expands the tree by 1 vertice
std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::expand()
{
//  Samples a new random point
    std::shared_ptr<Node> rand(new Node(sampleCoords()));
//  Finds the nearest existing node
    std::shared_ptr<Node> nearest = findNearest(rand);
//  Moves new point towards nearest node and returns resultant distance
//  Note sure if I need the distance yet
    double sqdToNear = moveToNear(rand, nearest);
    
    /*----------------------*/
    /*Find parent candidates*/
    /*Choose parent---------*/
    /*Connect to parent-----*/
    /*RRT* part TBA---------*/
    /*----------------------*/
    
//  Placeholder connection to physically nearest node
    nearest->children.push_back(rand);
    rand->parent = nearest;
    return rand;
}

std::shared_ptr<WorkSpaceTree::Node> WorkSpaceTree::grow()
{
    std::shared_ptr<Node> cur = _root;
//      Finds distance to the goal
    double dist = auxillary::findSqDistance(cur->pos, _goal);
//      While current node is not in the goal region
    while(dist > _r*_r)
    {
//          Expand the tree by 1 node and recalculate the distance
        cur = expand();
        dist = auxillary::findSqDistance(cur->pos, _goal);
        
    }
//      Returns the tree node in the goal region
    return cur;
}

//  Finds a path from root to the given node
std::deque<Point> WorkSpaceTree::trace(std::shared_ptr<Node> fin)
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
    std::queue<std::shared_ptr<Node>> q;
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
