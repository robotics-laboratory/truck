#include "node.hpp"


int main(int argc, char** argv) {
    std::thread subThread = planning_node::start_planning_node(argc, argv);
    subThread.join();
    return 0;
}
