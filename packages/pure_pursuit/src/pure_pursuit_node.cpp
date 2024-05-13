#include "pure_pursuit/pure_pursuit_node.h"

namespace truck::pure_pursuit {

using namespace std::placeholders;

namespace {

truck_msgs::msg::PurePursuitStatus toOkStatus(const std_msgs::msg::Header& header) {
    truck_msgs::msg::PurePursuitStatus status;

    status.header.stamp = header.stamp;
    status.header.frame_id = "base";
    status.status = truck_msgs::msg::PurePursuitStatus::OK;

    return status;
}

truck_msgs::msg::PurePursuitStatus toErrorStatus(const std_msgs::msg::Header& header, Error error) {
    truck_msgs::msg::PurePursuitStatus status;

    status.header.stamp = header.stamp;
    status.header.frame_id = "base";
    status.status = truck_msgs::msg::PurePursuitStatus::ERROR;
    status.error = static_cast<uint8_t>(error);

    return status;
}

truck_msgs::msg::PurePursuitStatus toNoLocalizationStatus(const rclcpp::Time t) {
    truck_msgs::msg::PurePursuitStatus status;

    status.header.stamp = t;
    status.header.frame_id = "base";
    status.status = truck_msgs::msg::PurePursuitStatus::NO_LOCALIZATION;

    return status;
}

truck_msgs::msg::PurePursuitStatus toNoTrajectoryStatus(const std_msgs::msg::Header& header) {
    truck_msgs::msg::PurePursuitStatus status;

    status.header = header;
    status.status = truck_msgs::msg::PurePursuitStatus::NO_TRAJECTORY;

    return status;
}

}  // namespace

PurePursuitNode::PurePursuitNode() : Node("pure_pursuit") {
    const auto model =
        model::load(this->get_logger(), Node::declare_parameter<std::string>("model_config"));

    const Parameters params = {
        .period = std::chrono::duration<double>(this->declare_parameter<double>("period", 0.02)),
        .radius =
            Limits<double>{
                this->declare_parameter<double>("radius/min", 0.15),
                this->declare_parameter<double>("radius/max", 0.5),
            },
        .velocity_factor = this->declare_parameter<double>("velocity_factor", 0.2),
        .tolerance = this->declare_parameter<double>("tolerance", 0.1),
        .max_distance = this->declare_parameter<double>("max_distance", 0.1)};

    RCLCPP_INFO(this->get_logger(), "period: %.2fs", params.period.count());
    RCLCPP_INFO(this->get_logger(), "radius: [%.2fm, %.2fm]", params.radius.min, params.radius.max);
    RCLCPP_INFO(this->get_logger(), "velocity_factor: %.2f", params.velocity_factor);
    RCLCPP_INFO(this->get_logger(), "tolerance: %.2fm", params.tolerance);
    RCLCPP_INFO(this->get_logger(), "max_distance: %.2fm", params.max_distance);

    controller_ = std::make_unique<PurePursuit>(params, model);

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    timer_ =
        Node::create_wall_timer(params.period, std::bind(&PurePursuitNode::publishCommand, this));

    slot_.odometry = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PurePursuitNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory", 1, std::bind(&PurePursuitNode::handleTrajectory, this, _1));

    signal_.command = Node::create_publisher<truck_msgs::msg::Control>("/motion/command", 1);

    signal_.status = Node::create_publisher<truck_msgs::msg::PurePursuitStatus>(
        "/motion/pure_pursuit/status", 1);
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
            msg.target.header = state_.localization_msg->header;
            msg.target.point = geom::msg::toPoint(*cmd.target);
        }

        msg.has_acceleration = true;

        return msg;
    };

    const auto now = this->now();

    const bool has_localization = state_.localization && state_.localization_msg &&
                                  (now - state_.localization_msg->header.stamp) < timeout_;

    if (!has_localization) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "missing localization");
        signal_.command->publish(toMsg(Command::stop()));
        signal_.status->publish(toNoLocalizationStatus(now));

        return;
    }

    const bool has_trajectory = state_.trajectory && state_.trajectory_msg &&
                                (now - state_.trajectory_msg->header.stamp) < timeout_;

    if (!has_trajectory) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "missing trajectory");
        signal_.command->publish(toMsg(Command::stop()));
        signal_.status->publish(toNoTrajectoryStatus(state_.localization_msg->header));
        return;
    }

    Result result{Error::kUnknown};
    try {
        result = controller_->command(*state_.localization, *state_.trajectory);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
    }

    if (!result) {
        const auto msg = toString(result.error());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", msg.data());
        signal_.command->publish(toMsg(Command::stop()));
        signal_.status->publish(toErrorStatus(state_.localization_msg->header, result.error()));
        return;
    }

    signal_.command->publish(toMsg(*result));
    signal_.status->publish(toOkStatus(state_.localization_msg->header));
}

void PurePursuitNode::handleTrajectory(truck_msgs::msg::Trajectory::SharedPtr trajectory) {
    state_.trajectory_msg = std::move(trajectory);
    state_.trajectory = motion::toTrajectory(*state_.trajectory_msg);
}

void PurePursuitNode::handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
    state_.localization_msg = (odometry);
    state_.localization = geom::toLocalization(*state_.localization_msg);
}

}  // namespace truck::pure_pursuit
