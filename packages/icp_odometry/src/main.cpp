#include "icp_odometry/icp_odometry_node.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::icp_odometry::IcpOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
