//
//  statespace_tree.h
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 22.04.2022.
//

#pragma once
#include "primitives.h"
#include <map>

struct StateSpaceTree
{
    struct Node
    {
        StatePoint pos;
        std::map<std::shared_ptr<Node>, arcPtr> children;
        std::shared_ptr<Node> parent;
        arcPtr fromParent;
        double cost;
        
        
        Node(StatePoint coords) :
        pos(coords), parent(nullptr), fromParent(nullptr), cost(0)
        {}
        
        Node(StatePoint coords, std::shared_ptr<Node> inp_parent, arcPtr path, double prevCost) :
        pos(coords), parent(inp_parent), fromParent(path), cost(prevCost + path->value().getLength())
        {}
        
        ~Node()
        {}
    };
    
    typedef std::shared_ptr<Node> nodePtr;
    nodePtr _root;
    
    nodePtr attach(nodePtr parent, const StatePoint& child, const arcPtr path)
    {
        nodePtr newNode(new Node(child, parent, path, parent->cost));
        parent->children.insert({newNode, path});
        return newNode;
    }
    
    void detach(nodePtr parent, nodePtr child)
    {
        parent->children.erase(parent->children.find(child));
        child->parent = nullptr;
        child->fromParent = nullptr;
    }

    StateSpaceTree(StatePoint root) : _root(new Node(root)){}
};
