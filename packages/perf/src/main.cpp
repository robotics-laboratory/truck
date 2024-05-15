#include "perf/node.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::perf::PerfStatNode>());
    rclcpp::shutdown();
    return 0;
}
