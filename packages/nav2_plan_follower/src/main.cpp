#include "nav2_plan_follower/nav2_plan_follower_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::nav2_plan_follower::Nav2PlanFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
