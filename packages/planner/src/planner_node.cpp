#include "planner/planner_node.h"

namespace truck::planner::visualization {

using namespace std::chrono_literals;
using namespace std::placeholders;

PlannerNode::PlannerNode() : Node("planner") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onOdometry, this, _1));

    slot_.clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerNode::onFinishPoint, this, _1));

    slot_.occupancy_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&PlannerNode::onGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    TfCallback tf_call = std::bind(&PlannerNode::onTf, this, _1, false);
    TfCallback static_tf_callback =
        std::bind(&PlannerNode::onTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    signal_.graph = this->create_publisher<visualization_msgs::msg::Marker>("/graph", 10);

    signal_.path = this->create_publisher<visualization_msgs::msg::Marker>("/path", 10);

    search::GridParams grid_params = search::GridParams{
        .width = this->declare_parameter<int>("grid/nodes/width"),
        .height = this->declare_parameter<int>("grid/nodes/height"),
        .resolution = this->declare_parameter<double>("grid/resolution"),
        .finish_area_radius = this->declare_parameter<double>("finish_area_radius"),
        .min_obstacle_distance = this->declare_parameter<double>("min_obstacle_distance"),
    };

    params_ = Parameters{
        .grid = grid_params,

        .node = Parameters::NodeParams {
            .z_lev = this->declare_parameter<double>("node/z-lev"),
            .scale = this->declare_parameter<double>("node/scale"),

            .base_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node/base/color")),

            .start_color =
                toColorRGBA(this->declare_parameter<std::vector<double>>("node/start/color")),

            .finish_base_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node/finish/base/color")),

            .finish_accent_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node/finish/accent/color")),

            .collision_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node/collision/color")),
        },

        .path = Parameters::PathParams {
            .z_lev = this->declare_parameter<double>("path/z-lev"),
            .scale = this->declare_parameter<double>("path/scale"),
            .color = toColorRGBA(this->declare_parameter<std::vector<double>>("path/color"))
        }
    };

    edge_geometry_cache_ = search::EdgeGeometryCache(this->declare_parameter<std::string>("primitives_config"));

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ = this->create_wall_timer(200ms, bind(&PlannerNode::doPlanningLoop, this));
}

Color PlannerNode::toColorRGBA(const std::vector<double>& vector) {
    Color color;
    color.a = vector[0];
    color.r = vector[1];
    color.g = vector[2];
    color.b = vector[3];
    return color;
}

void PlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!state_.odom) {
        return;
    }

    const auto source = msg->header.frame_id;
    const auto target = state_.odom->header.frame_id;

    const auto tf_opt = getLatestTranform(source, target);
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

    // update collision checker
    checker_->reset(*state_.distance_transform);
}

void PlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.odom = msg;
    state_.ego_pose = geom::toPose(*msg);
}

void PlannerNode::onFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    state_.finish_area = geom::Circle{
        .center = geom::toVec2(*msg), .radius = params_.grid.finish_area_radius};
}

void PlannerNode::onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

Color PlannerNode::getNodeColor(size_t node_index, const search::Grid& grid) const {
    const auto& node = grid.getNodes()[node_index];

    Color node_color = params_.node.base_color;

    if (node.is_finish) {
        node_color = params_.node.finish_base_color;

        if (node_index == grid.getEndNodeIndex()) {
            node_color = params_.node.finish_accent_color;
        }
    }

    if (node.collision) {
        node_color = params_.node.collision_color;
    }

    if (node_index == grid.getStartNodeIndex()) {
        node_color = params_.node.start_color;
    }

    return node_color;
}

void PlannerNode::publishGrid(const search::Grid& grid) {
    visualization_msgs::msg::Marker graph_nodes;
    graph_nodes.header.stamp = now();
    graph_nodes.header.frame_id = "odom_ekf";
    graph_nodes.id = 0;
    graph_nodes.type = visualization_msgs::msg::Marker::POINTS;
    graph_nodes.action = visualization_msgs::msg::Marker::ADD;
    graph_nodes.lifetime = rclcpp::Duration::from_seconds(0);

    graph_nodes.scale.x = params_.node.scale;
    graph_nodes.scale.y = params_.node.scale;
    graph_nodes.scale.z = params_.node.scale;
    graph_nodes.pose.position.z = params_.node.z_lev;

    const std::vector<search::Node>& nodes = grid.getNodes();

    for (size_t i = 0; i < nodes.size(); i++) {
        graph_nodes.points.push_back(geom::msg::toPoint(nodes[i].point));

        Color node_color = getNodeColor(i, grid);
        graph_nodes.colors.push_back(node_color);
    }

    signal_.graph->publish(graph_nodes);
}

void PlannerNode::publishPath(const search::Searcher& searcher) {
    visualization_msgs::msg::Marker path;
    path.header.stamp = now();
    path.header.frame_id = "odom_ekf";
    path.id = 0;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.lifetime = rclcpp::Duration::from_seconds(0);

    path.scale.x = params_.path.scale;
    path.scale.y = params_.path.scale;
    path.scale.z = params_.path.scale;
    path.pose.position.z = params_.path.z_lev;
    path.color = params_.path.color;

    for (const geom::Vec2& vertex : searcher.getPath()) {
        path.points.push_back(geom::msg::toPoint(vertex));
    }

    signal_.path->publish(path);
}

void PlannerNode::resetPath() {
    visualization_msgs::msg::Marker path;
    path.header.stamp = now();
    path.header.frame_id = "odom_ekf";
    path.id = 0;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.lifetime = rclcpp::Duration::from_seconds(0);

    signal_.path->publish(path);
}

std::optional<geom::Transform> PlannerNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        return geom::toTransform(tf_buffer_->lookupTransform(target, source, rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void PlannerNode::doPlanningLoop() {
    if (!checker_->initialized() ||
        !state_.ego_pose.has_value() ||
        !state_.finish_area.has_value()) {
        return;
    }

    // initialize grid
    search::Grid grid =
        search::Grid(params_.grid)
            .setEgoPose(state_.ego_pose)
            .setFinishArea(state_.finish_area)
            .setCollisionChecker(checker_)
            .build();

    // visualize grid
    publishGrid(grid);

    if (!grid.getEndNodeIndex().has_value()) {
        // need to reset visualization of last found path
        resetPath();
        return;
    }

    // initialize graph
    search::DynamicGraph graph =
        search::DynamicGraph()
            .setGrid(std::make_shared<search::Grid>(grid))
            .setEdgeGeometryCache(std::make_shared<search::EdgeGeometryCache>(edge_geometry_cache_));

    // initialize searcher
    search::Searcher searcher =
        search::Searcher()
            .setGraph(std::make_shared<search::DynamicGraph>(graph))
            .findPath();

    // visualize path
    publishPath(searcher);
}

}  // namespace truck::planner::visualization