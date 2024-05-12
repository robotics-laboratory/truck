#include "trajectory_planner/node.h"

#include <functional>

namespace truck::trajectory_planner::visualization {

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

TrajectoryPlannerNode::TrajectoryPlannerNode() : rclcpp::Node("trajectory_planner_node") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&TrajectoryPlannerNode::OnOdometry, this, _1));

    slot_.clicked_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal",
        rclcpp::QoS(1).reliability(qos),
        bind(&TrajectoryPlannerNode::OnFinishPose, this, _1));

    slot_.occupancy_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&TrajectoryPlannerNode::OnGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    TfCallback tf_call = std::bind(&TrajectoryPlannerNode::OnTf, this, _1, false);
    TfCallback static_tf_callback = std::bind(&TrajectoryPlannerNode::OnTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    signal_.tree = this->create_publisher<visualization_msgs::msg::Marker>("/tree", 10);

    signal_.path = this->create_publisher<visualization_msgs::msg::Marker>("/path", 10);

    signal_.finish = this->create_publisher<visualization_msgs::msg::Marker>("/finish", 10);

    Planner::Params trajectory_planner_params =
        {.truck_state_params =
             {.min_dist_to_obstacle = this->declare_parameter<double>(
                  "trajectory_planner.truck_state_params.min_dist_to_obstacle", 0.1)},
         .state_space_params =
             {.longitude =
                  {.limits = Limits<double>(
                       this->declare_parameter<double>(
                           "trajectory_planner.state_space_params.longitude.negative_offset", -1.0),
                       this->declare_parameter<double>(
                           "trajectory_planner.state_space_params.longitude.positive_offset", 1.1)),
                   .total_states = static_cast<int>(this->declare_parameter<int>(
                       "trajectory_planner.state_space_params.longitude.discretization", 10))},
              .latitude =
                  {.limits = Limits<double>(
                       this->declare_parameter<double>(
                           "trajectory_planner.state_space_params.latitude.negative_offset", -0.5),
                       this->declare_parameter<double>(
                           "trajectory_planner.state_space_params.latitude.positive_offset", 0.6)),
                   .total_states = static_cast<int>(this->declare_parameter<int>(
                       "trajectory_planner.state_space_params.latitude.discretization", 10))},
              .total_forward_yaw_states = static_cast<int>(this->declare_parameter<int>(
                  "trajectory_planner.state_space_params.yaw.total_forward_states", 5)),
              .total_backward_yaw_states = static_cast<int>(this->declare_parameter<int>(
                  "trajectory_planner.state_space_params.yaw.total_backward_states", 3)),
              .velocity =
                  {.limits = Limits<double>(
                       this->declare_parameter<double>(
                           "trajectory_planner.tate_space_params.velocity.min", 0.0),
                       this->declare_parameter<double>(
                           "trajectory_planner.state_space_params.velocity.max", 0.8)),
                   .total_states = static_cast<int>(this->declare_parameter<int>(
                       "trajectory_planner.state_space_params.velocity.discretization", 10))}},
         .tree_params =
             {.total_heuristic_steps = static_cast<size_t>(this->declare_parameter<int>(
                  "trajectory_planner.tree_params.total_heuristic_steps", 3)),
              .step_resolution = this->declare_parameter<double>(
                  "trajectory_planner.tree_params.step_resolution", 0.01)},
         .planning_horizon =
             this->declare_parameter<double>("trajectory_planner.planning_horizon", 100.0),
         .batch_size =
             static_cast<int>(this->declare_parameter<int>("trajectory_planner.batch_size", 500)),
         .max_batches =
             static_cast<int>(this->declare_parameter<int>("trajectory_planner.max_batches", 300)),
         .max_edges =
             static_cast<int>(this->declare_parameter<int>("trajectory_planner.max_edges", 200000)),
         .radius_multiplier =
             this->declare_parameter<double>("trajectory_planner.radius_multiplier", 5.0)};

    params_ = Parameters{
        .planner_params = trajectory_planner_params,

        .node = Parameters::NodeParams{
            .z_lev = this->declare_parameter<double>("node.z_lev"),
            .scale = this->declare_parameter<double>("node.scale"),

            .base_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.base")),

            .start_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.start")),

            .finish_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.finish")),

            .optimal_finish_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.optimal_finish")),
        }};

    model_ = std::make_shared<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ =
        this->create_wall_timer(200ms, std::bind(&TrajectoryPlannerNode::DoPlanningLoop, this));

    planner_ = Planner(params_.planner_params);
    planner_.SetModel(model_).SetCollisionChecker(checker_);
}

void TrajectoryPlannerNode::OnGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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

void TrajectoryPlannerNode::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.odom = msg;
    state_.ego_state = geom::toLocalization(*msg);
}

void TrajectoryPlannerNode::OnFinishPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    state_.finish_area = {.base_state = {.pose = geom::toPose(*msg), .velocity = 0.0}};
}

void TrajectoryPlannerNode::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

std_msgs::msg::ColorRGBA TrajectoryPlannerNode::GetNodeColor(
    const trajectory_planner::Node& node) const {
    std_msgs::msg::ColorRGBA node_color = params_.node.base_color;

    if (node.type == trajectory_planner::Node::Type::FINISH) {
        node_color = params_.node.finish_color;

        if (planner_.GetFinishNode() == &node) {
            node_color = params_.node.optimal_finish_color;
        }
    }

    if (node.type == trajectory_planner::Node::Type::START) {
        node_color = params_.node.start_color;
    }

    return node_color;
}

std::optional<geom::Transform> TrajectoryPlannerNode::GetLatestTranform(
    std::string_view source, std::string_view target) {
    try {
        return geom::toTransform(
            tf_buffer_->lookupTransform(target.data(), source.data(), rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void TrajectoryPlannerNode::DoPlanningLoop() {
    if (!checker_->initialized()) {
        RCLCPP_WARN(this->get_logger(), "No checker!");
        return;
    }
    if (!state_.ego_state) {
        RCLCPP_WARN(this->get_logger(), "No ego_state!");
        return;
    }
    if (!state_.finish_area) {
        RCLCPP_WARN(this->get_logger(), "No finish_area!");
        return;
    }

    auto route =
        geom::Polyline{state_.ego_state->pose.pos, state_.finish_area->base_state.pose.pos};

    planner_.Clear().Build(*state_.ego_state, *state_.finish_area, route);

    if (planner_.GetPlan().empty()) {
        RCLCPP_WARN(this->get_logger(), "Path not found!");
    }

    // Publish();
}

}  // namespace truck::trajectory_planner::visualization