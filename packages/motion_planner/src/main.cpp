#include "motion_planner/motion_planner_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::motion_planner::MotionPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
