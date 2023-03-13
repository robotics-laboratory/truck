#include "waypoint_follower/waypoint_follower_node.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::waypoint_follower::WaypointFollowerNode>());
    rclcpp::shutdown();
    return 0;
}