#include "motion_planner/motion_planner_node.h"

#include "geom/msg.h"
#include "geom/motion_state.h"
#include "model/model.h"
#include "motion/trajectory.h"
#include "speed/greedy_planner.h"
#include "motion_planner/search.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <algorithm>
#include <functional>
#include <optional>

namespace truck::motion_planner {

namespace {

Limits<double> velocityLimits(const model::Model& model, double velocity, double time) {
    const auto& vel = model.baseVelocityLimits();

    return {
        std::max(.0, vel.clamp(velocity) - model.baseMaxDeceleration() * time),
        std::min(vel.max, vel.clamp(velocity) + model.baseMaxAcceleration() * time)};
}

motion::Trajectory convertToTrajectory(
    const geom::MotionStates& mstates, const geom::Transform& tf) {
    motion::Trajectory trajectory(mstates.size());

    std::transform(
        mstates.cbegin(),
        mstates.cend(),
        trajectory.states.begin(),
        [&tf](const geom::MotionState& mstate) -> motion::State {
            return motion::State{.pose = tf.apply(mstate.pose())};
        });

    trajectory.fillDistance();
    return trajectory;
}

template<typename It>
Reference convertToReference(It begin, It end, const geom::Transform& tf) {
    VERIFY(std::distance(begin, end) > 1);

    geom::Poses poses;

    for (auto curr = begin + 1; curr != end; ++curr) {
        const auto prev = curr - 1;
        const geom::Pose world_pose{
            .pos = geom::toVec2(*prev),
            .dir = geom::AngleVec2::fromVector(geom::toVec2(*curr) - geom::toVec2(*prev))};

        poses.push_back(tf.apply(world_pose));
    }

    const geom::Pose world_pose{.pos = geom::toVec2(*(end - 1)), .dir = poses.back().dir};

    poses.push_back(tf.apply(world_pose));

    return Reference{std::move(poses)};
}

RTree toRTree(const std::vector<geom::Vec2>& points) {
    RTree rtree;

    for (size_t i = 0; i < points.size(); i++) {
        rtree.insert(IndexPoint(points[i], i));
    }

    return rtree;
}

RTree toRTree(const hull::Nodes& nodes) {
    RTree rtree;

    for (const auto& node : nodes) {
        rtree.insert(IndexPoint(node.pose.pos, node.id));
    }

    return rtree;
}

size_t findNearestIndex(const RTree& rtree, const geom::Vec2& point) {
    IndexPoints rtree_indexed_points;

    rtree.query(bg::index::nearest(point, 1), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points[0].second;
}

}  // namespace

MotionPlannerNode::MotionPlannerNode() : Node("motion_planner") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));
    slot_.route = this->create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&MotionPlannerNode::onRoute, this, _1));

    signal_.trajectory =
        this->create_publisher<truck_msgs::msg::Trajectory>("/motion/trajectory", 10);

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

    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/reset_path", std::bind(&MotionPlannerNode::onReset, this, _1, _2));

    slot_.odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", 1, std::bind(&MotionPlannerNode::onOdometry, this, _1));

    slot_.grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&MotionPlannerNode::onGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    const TfCallback tf_call = std::bind(&MotionPlannerNode::onTf, this, _1, false);
    const TfCallback static_tf_callback = std::bind(&MotionPlannerNode::onTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    timer_.main = this->create_wall_timer(
        params_.period, std::bind(&MotionPlannerNode::publishFullState, this));

    signal_.distance_transform =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid/distance_transform", 1);

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    checker_ = std::make_unique<collision::StaticCollisionChecker>(model_->shape());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    // TODO: de-hardcode filepath
    std::string map_file_path = this->declare_parameter("map_file_path", "map/data/map_6.geojson");
    map_ = std::make_unique<map::Map>(map::Map::fromGeoJson(map_file_path));
}

void MotionPlannerNode::publishFullState() {
    publishTrajectory();
    publishGridCostMap();
}

void MotionPlannerNode::publishTrajectory() {
    if (!state_.graph) {
        RCLCPP_WARN(this->get_logger(), "Ignore grid, there's no graph!");
        return;
    }

    const auto source = std::string{"world"};
    const auto target = state_.odometry->header.frame_id;

    auto tf_opt = getLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    checker_->reset(*state_.distance_transform);

    NodeId node_from = findNearestIndex(cache_.node_pts, state_.localization->pose.pos);

    const auto node_occupancy =
        search::getNodeOccupancy(*state_.graph, *checker_, model_->shape().radius());
    const auto path =
        search::findShortestPath(*state_.graph, node_occupancy, node_from, cache_.finish_nodes);

    const geom::MotionStates spline = search::fitSpline(state_.graph->nodes, path);
    state_.trajectory = convertToTrajectory(spline, *tf_opt);

    bool collision = false;
    for (auto& state : state_.trajectory.states) {
        const double margin = checker_->distance(state.pose);
        collision |= margin < params_.safety_margin;

        state.collision = collision;
        state.margin = margin;
    }

    speed::GreedyPlanner(speed_params_, *model_)
        .setScheduledVelocity(state_.scheduled_velocity)
        .fill(state_.trajectory);

    // dirty way to drop invalid trajectories and get error message
    try {
        state_.trajectory.throwIfInvalid(motion::TrajectoryValidations::enableAll(), *model_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Drop invalid trajectory!");

        state_.trajectory = motion::Trajectory{};
    }

    const double latency = params_.period.count();
    const auto limits = velocityLimits(*model_, state_.localization->velocity, latency);

    if (!state_.trajectory.empty()) {
        const auto scheduled_state = state_.trajectory.byTime(latency);
        state_.scheduled_velocity = limits.clamp(scheduled_state.velocity);
    } else {
        state_.scheduled_velocity = 0.0;
    }

    signal_.trajectory->publish(
        motion::msg::toTrajectory(state_.odometry->header, state_.trajectory));
}

void MotionPlannerNode::publishGridCostMap() {
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

void MotionPlannerNode::onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg) {
    if (!state_.odometry || !state_.distance_transform) {
        return;
    }

    const auto source = std::string{"world"};
    const auto target = state_.odometry->header.frame_id;

    auto tf_opt = getLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    const auto reference = convertToReference(msg->data.begin(), msg->data.end(), *tf_opt);
    const auto [graph, context] = builder_->buildGraph(reference, map_->polygons().at(0));

    state_.graph = std::move(graph);
    cache_.node_pts = toRTree(graph.nodes);
    cache_.finish_nodes = std::set<NodeId>{
        context.milestone_nodes.back().cbegin(),
        context.milestone_nodes.back().cend(),
    };
}

void MotionPlannerNode::onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*unused*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*unused*/) {
    RCLCPP_INFO(this->get_logger(), "Reset path!");

    state_.trajectory = motion::Trajectory{};
    publishFullState();
}

void MotionPlannerNode::onOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
    state_.localization = geom::toLocalization(*odometry);
    state_.odometry = odometry;
}

void MotionPlannerNode::onGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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

void MotionPlannerNode::onTf(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string kAuthority;
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, kAuthority, is_static);
    }
}

std::optional<geom::Transform> MotionPlannerNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        return geom::toTransform(tf_buffer_->lookupTransform(target, source, rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}
}  // namespace truck::motion_planner
