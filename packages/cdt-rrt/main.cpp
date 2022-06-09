//
//  main.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 13.04.2022.
//

#include <iostream>
#include "workspace_tree.h"
 
int main(int argc, char** argv) {
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {starting direction},{goal point}, {allowed deviation from goal}
    int a, b;
    std::cout << "Enter the x-bounds: ";
    std::cin >> a >> b;
    Bounds x(a, b);
    std::cout << "Enter the y-bounds: ";
    std::cin >> a >> b;
    Bounds y(a, b);
    
    double maxExtend;
    std::cout << "Enter maximum extension length: ";
    std::cin >> maxExtend;
    
    std::cout << "Enter the initial point: ";
    std::cin >> a >> b;
    Point start(a, b);
    
    std::cout << "Enter the initial direction vector: ";
    std::cin >> a >> b;
    geom::Vec2d dir(a, a);
    
    std::cout << "Enter the goal point: ";
    std::cin >> a >> b;
    Point goal(a, b);
    
    double devRad;
    std::cout << "Enter the allowed deviation from goal: ";
    std::cin >> devRad;
    
    
    WorkSpaceTree tree(x, y, maxExtend, start, dir, goal, devRad);
    auto tr = tree.findAndExport();
    std::cout << "RRT test: coordinates from start to goal\n";
    std::cout << "START\n";
    while(tr.size() > 0)
    {
        std::cout << tr.front().x << '\t' << tr.front().y << '\n';
        tr.pop_front();
    }
    std::cout << "FINISH\n";
    return 0;
}
