#include "visualization/visualization_node.h"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::visualization::VisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
