#include "waypoint_follower/waypoint_follower_node.h"

#include "geom/distance.h"
#include "geom/msg.h"
#include "geom/transform.h"
#include "model/model.h"
#include "motion/trajectory.h"
#include "speed/greedy_planner.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <functional>
#include <memory>
#include <optional>

namespace truck::waypoint_follower {

using namespace std::placeholders;

namespace {

Limits<double> velocityLimits(const model::Model& model, double velocity, double time) {
    const auto& vel = model.baseVelocityLimits();

    return {
        std::max(.0, vel.clamp(velocity) - model.baseMaxDeceleration() * time),
        std::min(vel.max, vel.clamp(velocity) + model.baseMaxAcceleration() * time)};
}

}  // namespace

WaypointFollowerNode::WaypointFollowerNode() : Node("waypoint_follower") {
    params_ = {
        .period = std::chrono::duration<double>(this->declare_parameter("period", 0.1)),
        .safety_margin = this->declare_parameter("safety_margin", 0.3),
    };

    RCLCPP_INFO(this->get_logger(), "period: %.2fs", params_.period.count());
    RCLCPP_INFO(this->get_logger(), "safety_margin: %.2fm", params_.safety_margin);

    speed_params_ = {
        Limits<double>{
            this->declare_parameter("acceleration/min", -0.5),
            this->declare_parameter("acceleration/max", 0.3),
        },
        this->declare_parameter("distance_to_obstacle", 0.7),
    };

    RCLCPP_INFO(
        this->get_logger(), "distance_to_obstacle: %.2fm", speed_params_.distance_to_obstacle);

    RCLCPP_INFO(
        this->get_logger(),
        "acceleration: [%.2f, %.2f]",
        speed_params_.acceleration.min,
        speed_params_.acceleration.max);

    // TODO: change service name
    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/reset_path", std::bind(&WaypointFollowerNode::onReset, this, _1, _2));

    slot_.odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", 1, std::bind(&WaypointFollowerNode::onOdometry, this, _1));

    slot_.waypoint = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&WaypointFollowerNode::onWaypoint, this, _1));

    slot_.grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&WaypointFollowerNode::onGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    TfCallback tf_call = std::bind(&WaypointFollowerNode::onTf, this, _1, false);
    TfCallback static_tf_callback = std::bind(&WaypointFollowerNode::onTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    timer_.main = this->create_wall_timer(
        params_.period, std::bind(&WaypointFollowerNode::publishFullState, this));

    signal_.waypoints = this->create_publisher<truck_msgs::msg::Waypoints>("/waypoints", 10);
    signal_.distance_transform =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid/distance_transform", 1);

    signal_.trajectory =
        this->create_publisher<truck_msgs::msg::Trajectory>("/motion/trajectory", 10);

    follower_ = std::make_unique<WaypointFollower>(WaypointFollower::Parameters{
        .resolution = this->declare_parameter("resolution", 0.05),
        .check_in_distance = this->declare_parameter("check_in_distance", 0.1)});

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    checker_ = std::make_unique<collision::StaticCollisionChecker>(model_->shape());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
}

void WaypointFollowerNode::onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    RCLCPP_INFO(this->get_logger(), "Reset path!");
    follower_->reset();
    publishFullState();
}

void WaypointFollowerNode::publishWaypoints() {
    if (!state_.odometry) {
        return;
    }

    truck_msgs::msg::Waypoints waypoints_msg;
    waypoints_msg.header = state_.odometry->header;

    for (const auto& waypoint : follower_->waypoints()) {
        waypoints_msg.waypoints.push_back(geom::msg::toPoint(waypoint.pos));
    }

    signal_.waypoints->publish(waypoints_msg);
}

namespace {

bool isStanding(const nav_msgs::msg::Odometry& odom) {
    const auto& twist = odom.twist.twist;
    const geom::Vec2 vel{twist.linear.x, twist.linear.y};
    return vel.lenSq() < squared(0.01);
}

motion::Trajectory makeTrajectory(const std::deque<LinkedPose>& path) {
    motion::Trajectory trajectory;

    for (const auto& lp : path) {
        const motion::State state{lp.pose};
        trajectory.states.push_back(state);
    }

    trajectory.fillDistance();

    return trajectory;
}

}  // namespace

void WaypointFollowerNode::publishGridCostMap() {
    if (!state_.distance_transform) {
        return;
    }

    if (!signal_.distance_transform->get_subscription_count()) {
        return;
    }

    constexpr double k_max_distance = 10.0;
    const auto msg = state_.distance_transform->makeCostMap(state_.grid->header, k_max_distance);
    signal_.distance_transform->publish(msg);
}

void WaypointFollowerNode::publishTrajectory() {
    if (!state_.odometry || !state_.distance_transform) {
        return;
    }

    const auto ego_pose = geom::toPose(*state_.odometry);
    follower_->update(ego_pose);

    if (follower_->isReadyToFinish(ego_pose) && isStanding(*state_.odometry)) {
        RCLCPP_INFO(this->get_logger(), "Path finished!");
        follower_->reset();
    }

    checker_->reset(*state_.distance_transform);

    motion::Trajectory trajectory = makeTrajectory(follower_->path());
    bool collision = false;
    for (auto& state : trajectory.states) {
        const double margin = checker_->distance(state.pose);
        collision |= margin < params_.safety_margin;

        state.collision = collision;
        state.margin = margin;
    }

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

void WaypointFollowerNode::publishFullState() {
    publishTrajectory();
    publishWaypoints();
    publishGridCostMap();
}

void WaypointFollowerNode::onOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
    state_.localization = geom::toLocalization(*odometry);
    state_.odometry = odometry;
}

std::optional<geom::Transform> WaypointFollowerNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        return geom::toTransform(tf_buffer_->lookupTransform(target, source, rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void WaypointFollowerNode::onWaypoint(geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (!state_.odometry) {
        RCLCPP_WARN(this->get_logger(), "Has no odometry, ignore waypoint!");
        return;
    }

    const auto tf_opt = getLatestTranform(msg->header.frame_id, state_.odometry->header.frame_id);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Can't lookup transform from '%s' to '%s', ignore waypoint!",
            msg->header.frame_id.c_str(),
            state_.odometry->header.frame_id.c_str());

        return;
    }

    const auto ego = geom::toPose(*state_.odometry);
    if (!follower_->hasWaypoints()) {
        // always start from current position
        RCLCPP_INFO(this->get_logger(), "Add ego waypoint (%f, %f)", ego.pos.x, ego.pos.y);
        follower_->addEgoWaypoint(ego);
    }

    const auto waypoint = tf_opt->apply(geom::toVec2(*msg));
    if (follower_->addWaypoint(waypoint)) {
        RCLCPP_INFO(this->get_logger(), "Waypoint (%f, %f) added!", waypoint.x, waypoint.y);
    } else {
        RCLCPP_INFO(this->get_logger(), "Waypoint (%f, %f) ignored!", waypoint.x, waypoint.y);
    }

    follower_->update(ego);
    publishFullState();
}

void WaypointFollowerNode::onGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!state_.odometry) {
        return;
    }

    const auto source = msg->header.frame_id;
    const auto target = state_.odometry->header.frame_id;

    const auto tf_opt = getLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    // simple hack to apply transform to grid
    msg->header.frame_id = target;
    msg->info.origin = geom::msg::toPose(tf_opt->apply(geom::toPose(msg->info.origin)));

    // distance transfor - cpu intensive operation
    state_.distance_transform = std::make_shared<collision::Map>(
        collision::distanceTransform(collision::Map::fromOccupancyGrid(*msg)));

    state_.grid = msg;
}

void WaypointFollowerNode::onTf(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

}  // namespace truck::waypoint_follower
