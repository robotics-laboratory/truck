#include "motion_planner/motion_planner.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<truck::routing::RoutingNode>());
    rclcpp::shutdown();
    return 0;
}
