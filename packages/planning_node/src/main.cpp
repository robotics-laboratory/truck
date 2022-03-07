#include "node.hpp"


int main(int argc, char** argv) {
    std::thread sub_thread = planning_node::start_planning_node(argc, argv);
    sub_thread.join();
    return 0;
}
