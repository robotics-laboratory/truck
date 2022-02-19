#include "rclcpp/rclcpp.hpp"
#include "planning_interfaces/msg/path.hpp"
#include "nav_msgs/msg/path.hpp"

#include <iostream>
#include <memory>


namespace unwrapping_node {

using std::placeholders::_1;

struct UnwrappingNode : public rclcpp::Node {
    UnwrappingNode() : Node("UnwrappingNode") {
        path_subscription = create_subscription<planning_interfaces::msg::Path>(
            "path", 10, std::bind(&UnwrappingNode::new_path_callback, this, _1)
        );
        raw_path_publisher = create_publisher<nav_msgs::msg::Path>("raw_path", 10);
    }

private:
    void new_path_callback(const planning_interfaces::msg::Path::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New path: created_at=%ld", message->created_at);
        raw_path_publisher->publish(message->path);
    }

    rclcpp::Subscription<planning_interfaces::msg::Path>::SharedPtr path_subscription;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher;
};

}

int main(int argc, char** argv) {
    std::cout << "Starting unwrapping node" << std::endl;

    rclcpp::init(argc, argv);
    std::shared_ptr<unwrapping_node::UnwrappingNode> node = std::make_shared<unwrapping_node::UnwrappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
