#include "route_follower/route_follower_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::route_follower::RouteFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
