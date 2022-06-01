//
//  main.cpp
//  CDT-RRT*
//
//  Created by Konstantin Shashkov on 13.04.2022.
//

#include <iostream>
#include "workspace_tree.h"
 
int main(int argc, char** argv) {
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {goal point}, {allowed deviation from goal}, {speed at movement}
    WorkSpaceTree tree(Bounds(0, 100), Bounds(0, 100), 2, Point(10, 40), Point(80, 80), 2, 5);
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
