#include "rclcpp/rclcpp.hpp"

#include "pure_pursuit/node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::pure_pursuit::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
