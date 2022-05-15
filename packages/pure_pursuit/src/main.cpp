#include "pure_pursuit/node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pure_pursuit::PursuitNode>());
    rclcpp::shutdown();
}
