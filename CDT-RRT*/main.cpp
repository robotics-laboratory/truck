//
//  main.cpp
//  CDT-RRT*
//
//  Created by Константин Шашков on 13.04.2022.
//

#include <iostream>
#include "workspace_tree.h"
 
int main(int argc, char** argv) {
//  {x-bounds}, {y-bounds}, {maximum extension length}, {starting point}, {goal point}, {allowed deviation from goal}
    WorkSpaceTree tree({0,100}, {0, 100}, 3, {10, 40}, {80, 80}, 2);
    auto tr = tree.getTrajectory();
    std::cout << "RRT test: coordinates from start to goal\n";
    std::cout << "START\n";
    while(tr.size() > 0)
    {
        std::cout << tr.front().first << ' ' << tr.front().second << '\n';
        tr.pop_front();
    }
    std::cout << "FINISH\n";
    
    return 0;
}
