#include <memory>

#include "control_proxy_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::control_proxy::ControlProxyNode>());
    rclcpp::shutdown();
    return 0;
}
