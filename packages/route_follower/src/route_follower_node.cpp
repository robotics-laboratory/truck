#include "route_follower/route_follower_node.h"

#include "geom/msg.h"
#include "model/model.h"
#include "motion/trajectory.h"
#include "speed/greedy_planner.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <functional>
#include <optional>

namespace truck::route_follower {

namespace {

Limits<double> velocityLimits(const model::Model& model, double velocity, double time) {
    const auto& vel = model.baseVelocityLimits();

    return {
        std::max(.0, vel.clamp(velocity) - model.baseMaxDeceleration() * time),
        std::min(vel.max, vel.clamp(velocity) + model.baseMaxAcceleration() * time)};
}

}  // namespace

RouteFollowerNode::RouteFollowerNode() : Node("route_follower") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));
    slot_.route = this->create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&RouteFollowerNode::onRoute, this, _1));

    signal_.trajectory = this->create_publisher<truck_msgs::msg::Trajectory>(
        "/motion/trajectory", 10);  // TODO: what does `10` mean?
}

void RouteFollowerNode::onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg) {
    motion::Trajectory trajectory;
    std::optional<geom::Vec2> prev = std::nullopt;

    for (const geometry_msgs::msg::Point& p : msg->data) {
        geom::Vec2 curr = geom::toVec2(p);

        if (prev.has_value()) {
            trajectory.states.emplace_back();
            trajectory.states.back().pose =
                geom::Pose{.pos = *prev, .dir = geom::AngleVec2::fromVector(curr - *prev)};
        }

        prev = curr;
    }

    trajectory.states.emplace_back();
    trajectory.states.back().pose =
        geom::Pose{.pos = *prev, .dir = trajectory.states.back().pose.dir};

    this->publishTrajectory(trajectory);
}

void RouteFollowerNode::publishTrajectory(motion::Trajectory& trajectory) {
    speed::GreedyPlanner(speed_params_, *model_)
        .setScheduledVelocity(state_.scheduled_velocity)
        .fill(trajectory);

    // dirty way to drop invalid trajectories and get error message
    try {
        trajectory.throwIfInvalid(motion::TrajectoryValidations::enableAll(), *model_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Drop invalid trajectory!");

        trajectory = motion::Trajectory{};
    }

    const double latency = params_.period.count();
    const auto limits = velocityLimits(*model_, state_.localization->velocity, latency);

    if (!trajectory.empty()) {
        const auto scheduled_state = trajectory.byTime(latency);
        state_.scheduled_velocity = limits.clamp(scheduled_state.velocity);
    } else {
        state_.scheduled_velocity = 0.0;
    }

    signal_.trajectory->publish(motion::msg::toTrajectory(state_.odometry->header, trajectory));
}
}  // namespace truck::route_follower