#include <iostream>
#include "hardware_node_2.h"

using namespace truck::hardware_node;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}