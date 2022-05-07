#include "truck_interfaces/msg/control.hpp"
#include <chrono>
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace truck_interfaces;

using std::placeholders::_1;

struct ControlNode : rclcpp::Node {
    ControlNode() 
        : Node("ControlNode")
        , velocity_joints_count(2) {
        control_subscription = create_subscription<msg::Control>(
            "control", 10, std::bind(&ControlNode::new_control_callback, this, _1)
        );

        velocity_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10
        );

        RCLCPP_INFO(get_logger(), "Created velocity controller publisher. Topic: /velocity_controller/commands");

        steering_action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "/joint_trajectory_controller/follow_joint_trajectory"
        );

        if (!steering_action_client->wait_for_action_server(std::chrono::seconds(1))) {
            throw std::runtime_error{"could not connect to steering action server"};
        }

        RCLCPP_INFO(get_logger(), "Created steering action client. Topic: /joint_trajectory_controller/follow_joint_trajectory");
    }

private:
    void new_control_callback(msg::Control::SharedPtr message) {
        RCLCPP_INFO(
            get_logger(),
            "New control: curvature=%.6f velocity=%.6f max_acceleration=%.6f",
            message->curvature,
            message->velocity,
            message->max_acceleration
        );

        // default velocity controller ignores acceleration
        // to use max_acceleration, we need to build custom velocity + acceleration controller
        std_msgs::msg::Float64MultiArray command;
        for (size_t i = 0; i < velocity_joints_count; ++i) {
            command.data.push_back(message->velocity);
        }
        velocity_publisher->publish(command);

        auto sign = message->curvature < 0 ? true : false;

        double left_wheel_angle = std::atan2(kTruckLength, 1.0 / std::abs(message->curvature) - kTruckWidth / 2.0);
        double right_wheel_angle = std::atan2(kTruckLength, 1.0 / std::abs(message->curvature) + kTruckWidth /  2.0);
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
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start as soon as possible
        point.positions.push_back(left_wheel_angle);
        point.positions.push_back(right_wheel_angle);

        control_msgs::action::FollowJointTrajectory_Goal goal_msg;
        goal_msg.trajectory.joint_names = { "left_steering_joint", "right_steering_joint" };  // todo: move to config
        goal_msg.trajectory.points = { point };

        auto goal_future_handle = steering_action_client->async_send_goal(goal_msg, {});
        // if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future_handle) != rclcpp::FutureReturnCode::SUCCESS) {
        //     throw std::runtime_error{"failed to send steering goal"};
        // }
        // RCLCPP_INFO(get_logger(), "Successfully sent steering goal");

        // rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle = goal_future_handle.get();
        // if (!goal_handle) {
        //     throw std::runtime_error{"goal was rejected by the steering action server"};
        // }
        // RCLCPP_INFO(get_logger(), "Goal was accepted by the steering action server");

        // auto result_future = steering_action_client->async_get_result(goal_handle);
        // if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        //     throw std::runtime_error{"failed to get result from the steering action server"};
        // }
        // RCLCPP_INFO(get_logger(), "Successfully got result from steering action server");
    }
    rclcpp::Subscription<msg::Control>::SharedPtr control_subscription;
    
    size_t velocity_joints_count;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr steering_action_client;

    static constexpr double kTruckLength = 1;  // todo: use external vehicle params
    static constexpr double kTruckWidth = 0.5;  // todo: same
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
