#include "model/model.hpp"
#include "truck_interfaces/msg/control.hpp"
#include <chrono>
#include <cmath>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace truck_interfaces;

using std::placeholders::_1;

struct ControlNode : rclcpp::Node {
    ControlNode(model::Model model) 
        : Node("ControlNode")
        , model(std::move(model)) {
        control_command_subscription = create_subscription<msg::Control>(
            "control_command", 10, std::bind(&ControlNode::new_control_command_callback, this, _1)
        );

        velocity_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10
        );
        RCLCPP_INFO(get_logger(), "Created velocity controller publisher. Topic: /velocity_controller/commands");

        steering_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_controller/commands", 10
        );
        RCLCPP_INFO(get_logger(), "Created steering controller publisher. Topic: /steering_controller/commands");
    }

private:
    void new_control_command_callback(msg::Control::SharedPtr message) {
        RCLCPP_INFO(
            get_logger(),
            "New control: curvature=%.6f velocity=%.6f acceleration=%.6f",
            message->curvature,
            message->velocity,
            message->acceleration
        );

        double abs_curvature = std::abs(message->curvature);
        bool sign = message->curvature < 0;

        // default velocity controller ignores acceleration
        // to use acceleration, we need to build custom controller
        std_msgs::msg::Float64MultiArray velocity_command;
        double angular_velocity = message->velocity / model.wheel_radius;
        double left_wheel_velocity = angular_velocity * (1 - abs_curvature * model.truck_width / 2.0);
        double right_wheel_velocity = angular_velocity * (1 + abs_curvature * model.truck_width / 2.0);
        if (sign) {
            std::swap(left_wheel_velocity, right_wheel_velocity);
        }
        
        RCLCPP_INFO(
            get_logger(),
            "Calculated wheel velocity from curvature %.6f: left=%.6f right=%.6f",
            message->curvature,
            left_wheel_velocity,
            right_wheel_velocity
        );

        velocity_command.data.push_back(left_wheel_velocity);
        velocity_command.data.push_back(right_wheel_velocity);

        velocity_publisher->publish(velocity_command);

        double r = 1.0 / std::max(abs_curvature, 1e-9);
        double left_wheel_angle = std::atan2(model.truck_length, r - model.truck_width / 2.0);
        double right_wheel_angle = std::atan2(model.truck_length, r + model.truck_width / 2.0);
        if (sign) {
            left_wheel_angle = -left_wheel_angle;
            right_wheel_angle = -right_wheel_angle;
        }

        RCLCPP_INFO(
            get_logger(),
            "Calculated wheel angles from curvature %.6f: left=%.6f right=%.6f",
            message->curvature,
            left_wheel_angle,
            right_wheel_angle
        );

        std_msgs::msg::Float64MultiArray steering_command;
        steering_command.data.push_back(left_wheel_angle);
        steering_command.data.push_back(right_wheel_angle);
        steering_publisher->publish(steering_command);
    }
    
    rclcpp::Subscription<msg::Control>::SharedPtr control_command_subscription;
    model::Model model;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_publisher;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <model-config-path>";
        return 0;
    }

    std::string model_config_path = argv[1];
    model::Model model{model_config_path};

    rclcpp::init(argc, argv);
    std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>(std::move(model));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
