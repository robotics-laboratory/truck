#include "pure_pursuit/pure_pursuit_node.h"

namespace truck::pure_pursuit {

using namespace std::placeholders;

PurePursuitNode::PurePursuitNode() : Node("pure_pursuit") {
    const auto model =
        model::load(this->get_logger(), Node::declare_parameter<std::string>("model_config"));

    const Parameters params = {
        .period = std::chrono::duration<double>(this->declare_parameter<double>("period", 0.02)),
        .radius = Limits<double>{
            this->declare_parameter<double>("radius/min", 0.15),
            this->declare_parameter<double>("radius/max", 0.5),
        },
        .velocity = this->declare_parameter<double>("velocity", 0.4),
        .velocity_factor = this->declare_parameter<double>("velocity_factor", 0.2),
        .tolerance = this->declare_parameter<double>("tolerance", 0.1),
        .max_distance = this->declare_parameter<double>("max_distance", 0.1)
    };

    RCLCPP_INFO(this->get_logger(), "period %f", params.period.count());
    RCLCPP_INFO(this->get_logger(), "radius [%f, %f]", params.radius.min, params.radius.max);
    RCLCPP_INFO(this->get_logger(), "velocity %f", params.velocity);
    RCLCPP_INFO(this->get_logger(), "velocity factor %f", params.velocity_factor);
    RCLCPP_INFO(this->get_logger(), "tolerance %f", params.tolerance);
    RCLCPP_INFO(this->get_logger(), "max_distance %f", params.max_distance);

    controller_ = std::make_unique<PurePursuit>(params, model);

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    timer_ = Node::create_wall_timer(params.period, std::bind(&PurePursuitNode::publishCommand, this));

    slot_.odometry = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PurePursuitNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory", 1, std::bind(&PurePursuitNode::handleTrajectory, this, _1));

    signal_.command = Node::create_publisher<truck_msgs::msg::Control>("/motion/command", 1);
}

void PurePursuitNode::publishCommand() {
    auto toMsg = [this](const Command& cmd) {
        truck_msgs::msg::Control msg;

        msg.header.frame_id = "base";
        msg.header.stamp = now();

        msg.curvature = cmd.curvature;
        msg.velocity = cmd.velocity;
        msg.acceleration = cmd.acceleration;

        msg.has_target = cmd.target.has_value();
        if (cmd.target) {
            msg.target.header = state_.odometry->header;
            msg.target.point = geom::msg::toPoint(*cmd.target);
        }

        return msg;
    };

    const bool has_trajectory = state_.trajectory && !state_.trajectory->states.empty();
    const bool has_localization = state_.localization && state_.odometry;

    if (!has_trajectory || !has_localization) {
        signal_.command->publish(toMsg(Command::stop()));
        return;
    }
    const auto result = controller_->command(*state_.localization, *state_.trajectory);

    if (!result) {
        RCLCPP_ERROR(get_logger(), "%s", toString(result.error()).data());
        signal_.command->publish(toMsg(Command::stop()));
        return;
    }

    signal_.command->publish(toMsg(*result));
}

void PurePursuitNode::handleTrajectory(truck_msgs::msg::Trajectory::SharedPtr trajectory) {
    state_.trajectory = motion::toTrajectory(*trajectory);
}

void PurePursuitNode::handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
    state_.localization = geom::toLocalization(*odometry);
    state_.odometry = odometry;
}

}  // namespace truck::pure_pursuit