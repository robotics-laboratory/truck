#include "waypoint_follower/waypoint_follower_node.h"

#include "geom/distance.h"
#include "geom/msg.h"
#include "geom/transform.h"
#include "model/model.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <boost/assert.hpp>

#include <functional>
#include <optional>
#include <memory>

namespace truck::waypoint_follower {

using namespace std::placeholders;

constexpr size_t kThrotlleTime = 5000;

WaypointFollowerNode::WaypointFollowerNode(): Node("waypoint_follower") {
    params_ = {
        .period = std::chrono::milliseconds(this->declare_parameter("period", 100)),
        .safety_margin = this->declare_parameter("safety_margin", 0.3),
        .distance_before_obstacle = this->declare_parameter("distance_before_obstacle", 1.0),
    };

    // TODO: change service name
    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/control_demo/reset_path", std::bind(&WaypointFollowerNode::onReset, this, _1, _2));

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
        params_.period,
        std::bind(&WaypointFollowerNode::publishFullState, this));

    signal_.path = this->create_publisher<nav_msgs::msg::Path>("/motion/path", 10);
    signal_.waypoints = this->create_publisher<truck_interfaces::msg::Waypoints>("/waypoints", 10);
    signal_.distance_transform =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid/distance_transform", 1);

    follower_ = std::make_unique<WaypointFollower>(
        WaypointFollower::Parameters {
            .resolution = this->declare_parameter("resolution", 0.05),
            .check_in_distance = this->declare_parameter("check_in_distance", 0.1)
        }
    );

    const auto model = model::load(this->get_logger(), this->declare_parameter("model_config", ""));
    checker_ = std::make_unique<collision::StaticCollisionChecker>(model.shape());

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

    truck_interfaces::msg::Waypoints waypoints_msg;
    waypoints_msg.header = state_.odometry->header;

    for (const auto& waypoint : follower_->waypoints()) {
        waypoints_msg.waypoints.push_back(geom::msg::toPoint(waypoint.pos));
    }

    signal_.waypoints->publish(waypoints_msg);
}

namespace {

bool isStanding(const nav_msgs::msg::Odometry& odom) {
    const auto& twist = odom.twist.twist;
    return std::abs(twist.linear.x) < 0.1;
}

}  // namespace

void WaypointFollowerNode::publishGridCostMap() {
    if (!state_.distance_transform) {
        return;
    }

    if (!signal_.distance_transform->get_subscription_count()) {
        return;
    }

    constexpr double kMaxDistance = 10.0;
    const auto msg = state_.distance_transform->makeCostMap(state_.grid->header, kMaxDistance);
    signal_.distance_transform->publish(msg);
}

void WaypointFollowerNode::publishPath() {
    if (!state_.odometry || !state_.distance_transform) {
        return;
    }

    const auto ego_pose = geom::toPose(*state_.odometry);
    follower_->update(ego_pose);

    if (follower_->isReadyToFinish(ego_pose) && isStanding(*state_.odometry)) {
        follower_->reset();
    }

    checker_->reset(*state_.distance_transform);

    // cut collision along path
    const auto& path = follower_->path();
    auto end = std::find_if(path.begin(), path.end(), [&](const auto& lp) {
        return checker_->distance(lp.pose) < params_.safety_margin;
    });

    if (end != path.end()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), kThrotlleTime, "Unsafe path, cutting it!");

        double stop_margin = 0;
        while (end != path.begin()) {
            auto prev = end - 1;
            stop_margin += geom::distance(prev->pose.pos, end->pose.pos);
            if (stop_margin > params_.distance_before_obstacle) {
                break;
            }
            end = prev;
        }
    }

    // force check at ego pose
    if (checker_->distance(ego_pose) < params_.safety_margin) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), kThrotlleTime, "Collision at ego pose!");

        end = path.begin();
    }
    
    nav_msgs::msg::Path path_msg;
    path_msg.header = state_.odometry->header;

    for (auto it = path.begin(); it != end; ++it) {
        path_msg.poses.push_back(geom::msg::toPoseStamped(state_.odometry->header, it->pose));
    }

    signal_.path->publish(path_msg);
}

void WaypointFollowerNode::publishFullState() {
    publishPath();
    publishWaypoints();
    publishGridCostMap();
}

void WaypointFollowerNode::onOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
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
        RCLCPP_INFO(this->get_logger(), "Add ego waypoint: (%f, %f)", ego.pos.x, ego.pos.y);
        follower_->addEgoWaypoint(ego);
    }

    const auto waypoint = tf_opt->apply(geom::toVec2(*msg));
    RCLCPP_INFO(this->get_logger(), "Add waypoint: (%f, %f)", waypoint.x, waypoint.y);
    follower_->addWaypoint(waypoint);
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

} // namespace truck::waypoint_follower