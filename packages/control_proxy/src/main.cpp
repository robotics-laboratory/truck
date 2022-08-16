#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "control_proxy.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::control_proxy::ControlProxyNode>());
    rclcpp::shutdown();
    return 0;
}