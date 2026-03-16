#include "simulator_2d/truck_simulator_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::simulator::TruckSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}