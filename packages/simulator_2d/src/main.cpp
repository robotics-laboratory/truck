//#include "simulator_2d/demo.h"
#include "simulator_2d/simulator_node.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();
    return 0;
}