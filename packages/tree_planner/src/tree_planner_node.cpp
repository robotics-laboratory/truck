#include "tree_planner/tree_planner_node.h"

#include <functional>

namespace truck::planner::visualization {

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace {

std_msgs::msg::ColorRGBA toColorRGBA(const std::vector<double>& vector) {
    VERIFY(vector.size() == 4);

    std_msgs::msg::ColorRGBA color;
    color.a = vector[0];
    color.r = vector[1];
    color.g = vector[2];
    color.b = vector[3];
    return color;
}

}  // namespace

TreePlannerNode::TreePlannerNode() : Node("tree_planner_node") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&TreePlannerNode::OnOdometry, this, _1));

    slot_.clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        bind(&TreePlannerNode::OnFinishPoint, this, _1));

    slot_.occupancy_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&TreePlannerNode::OnGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    TfCallback tf_call = std::bind(&TreePlannerNode::OnTf, this, _1, false);
    TfCallback static_tf_callback = std::bind(&TreePlannerNode::OnTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    signal_.tree = this->create_publisher<visualization_msgs::msg::Marker>("/tree", 10);

    signal_.path = this->create_publisher<visualization_msgs::msg::Marker>("/path", 10);

    signal_.finish = this->create_publisher<visualization_msgs::msg::Marker>("/finish", 10);

    planner::tree_planner::TreePlannerParams tree_planner_params = {
        .max_sampling_num =
            static_cast<int>(this->declare_parameter<int>("tree_planner.max_sampling_num", 10000)),
        .steering_dist = this->declare_parameter<double>("tree_planner.steering_dist", 0.2),
        .sampling_range = this->declare_parameter<double>("tree_planner.sampling_range", 4.0),
        .goal_region_radius =
            this->declare_parameter<double>("tree_planner.finish_area_radius", 0.2)};

    params_ = Parameters{
        .tree_planner_params = tree_planner_params,

        .node = Parameters::NodeParams{
            .z_lev = this->declare_parameter<double>("node.z_lev"),
            .scale = this->declare_parameter<double>("node.scale"),

            .base_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.base")),

            .start_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.start")),

            .goal_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.goal")),

            .optimal_goal_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.optimal_goal")),
        }};

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ = this->create_wall_timer(200ms, std::bind(&TreePlannerNode::DoPlanningLoop, this));

    planner_.SetParams(params_.tree_planner_params).SetShape(model_->shape());
}

void TreePlannerNode::OnGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!state_.odom) {
        RCLCPP_WARN(this->get_logger(), "No odom!");
        return;
    }

    const auto source = msg->header.frame_id;
    const auto target = state_.odom->header.frame_id;

    const auto tf_opt = GetLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    msg->header.frame_id = target;
    msg->info.origin = geom::msg::toPose(tf_opt->apply(geom::toPose(msg->info.origin)));

    state_.distance_transform = std::make_shared<collision::Map>(
        collision::distanceTransform(collision::Map::fromOccupancyGrid(*msg)));

    state_.occupancy_grid = msg;

    checker_->reset(*state_.distance_transform);

    if (!checker_->initialized()) {
        RCLCPP_WARN(this->get_logger(), "Checker not initizlized");
    }
}

void TreePlannerNode::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.odom = msg;
    state_.ego_pose = geom::toPose(*msg).pos;
    state_.planning_space = geom::BBox{
        .left_lower = geom::Vec2(
            state_.ego_pose->x - params_.tree_planner_params.sampling_range,
            state_.ego_pose->y - params_.tree_planner_params.sampling_range),
        .right_upper = geom::Vec2(
            state_.ego_pose->x + params_.tree_planner_params.sampling_range,
            state_.ego_pose->y + params_.tree_planner_params.sampling_range)};
}

void TreePlannerNode::OnFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    state_.finish_area = {
        .center = geom::toVec2(*msg), .radius = params_.tree_planner_params.goal_region_radius};
}

void TreePlannerNode::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

std_msgs::msg::ColorRGBA TreePlannerNode::GetNodeColor(const tree_planner::Node& node) const {
    std_msgs::msg::ColorRGBA node_color = params_.node.base_color;

    if (node.is_goal_node) {
        node_color = params_.node.goal_color;

        if (node.is_optimal_goal_node) {
            node_color = params_.node.optimal_goal_color;
        }
    }

    if (node.is_start_node) {
        node_color = params_.node.start_color;
    }

    return node_color;
}

void TreePlannerNode::PublishTree() const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom_ekf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.scale.x = params_.node.scale;
    marker.scale.y = params_.node.scale;
    marker.scale.z = params_.node.scale;
    marker.pose.position.z = params_.node.z_lev;

    const std::vector<tree_planner::Node>& nodes = planner_.GetNodes();

    for (const auto& node : nodes) {
        marker.points.push_back(geom::msg::toPoint(node.pose));
        marker.colors.push_back(GetNodeColor(node));
    }

    signal_.tree->publish(marker);
}

void TreePlannerNode::PublishPath() const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom_ekf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.scale.x = params_.node.scale;
    marker.scale.y = params_.node.scale;
    marker.scale.z = params_.node.scale;
    marker.pose.position.z = params_.node.z_lev;
    marker.color.a = 0.6;
    marker.color.r = 1.0;

    const geom::Poses& poses = planner_.GetPath();

    for (const auto& pose : poses) {
        marker.points.push_back(geom::msg::toPoint(pose.pos));
    }

    signal_.path->publish(marker);
}

void TreePlannerNode::PublishGoal() const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom_ekf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.scale.x = params_.tree_planner_params.goal_region_radius;
    marker.scale.y = params_.tree_planner_params.goal_region_radius;
    marker.scale.z = params_.node.z_lev * 2;

    marker.color = params_.node.goal_color;

    if (planner_.GetGoalNode() == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Goal node not found!");
        return;
    }

    geom::Vec2 position = planner_.GetGoalNode()->pose;

    RCLCPP_WARN(this->get_logger(), "Goal node pos '%f', '%f'", position.x, position.y);

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = params_.node.z_lev;

    signal_.finish->publish(marker);
}

void TreePlannerNode::Publish() const {
    PublishTree();
    PublishPath();
    // PublishGoal();
}

std::optional<geom::Transform> TreePlannerNode::GetLatestTranform(
    std::string_view source, std::string_view target) {
    try {
        return geom::toTransform(
            tf_buffer_->lookupTransform(target.data(), source.data(), rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void TreePlannerNode::DoPlanningLoop() {
    if (!checker_->initialized()) {
        RCLCPP_WARN(this->get_logger(), "No checker!");
        return;
    }
    if (!state_.ego_pose) {
        RCLCPP_WARN(this->get_logger(), "No ego_pose!");
        return;
    }
    if (!state_.finish_area) {
        RCLCPP_WARN(this->get_logger(), "No finish_area!");
        return;
    }
    if (!state_.planning_space) {
        RCLCPP_WARN(this->get_logger(), "No planning_space!");
    }

    planner_.Reset(*state_.planning_space, *state_.ego_pose, *state_.finish_area, checker_).Build();

    const tree_planner::Node* start_node = planner_.GetStartNode();

    if (!start_node) {
        RCLCPP_WARN(this->get_logger(), "No start_node!");
        return;
    }

    if (!planner_.FoundPath()) {
        RCLCPP_WARN(this->get_logger(), "Path not found!");
    }

    Publish();
}

}  // namespace truck::planner::visualization
