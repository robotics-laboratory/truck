#include "planner/planner_node.h"

#include <tf2_ros/qos.hpp>

namespace truck::planner::visualization {
    
PlannerNode::PlannerNode() : Node("planner") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onOdometry, this, std::placeholders::_1));

    slot_.clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerNode::onFinishPoint, this, std::placeholders::_1));

    slot_.grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&PlannerNode::onGrid, this, std::placeholders::_1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    TfCallback tf_call = std::bind(&PlannerNode::onTf, this, std::placeholders::_1, false);
    TfCallback static_tf_callback =
        std::bind(&PlannerNode::onTf, this, std::placeholders::_1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    signal_.graph = this->create_publisher<visualization_msgs::msg::Marker>("/graph", 10);

    signal_.optimal_path = this->create_publisher<visualization_msgs::msg::Marker>("/path", 10);

    params_.width = this->declare_parameter<int>("width");
    params_.height = this->declare_parameter<int>("height");
    params_.resolution = this->declare_parameter<double>("resolution");
    params_.finish_area_radius = this->declare_parameter<double>("finish_area_radius");
    params_.json_path = this->declare_parameter<std::string>("json_path");

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ = this->create_wall_timer(250ms, bind(&PlannerNode::doPlanningLoop, this));
}

void PlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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

    msg->header.frame_id = target;
    msg->info.origin = geom::msg::toPose(tf_opt->apply(geom::toPose(msg->info.origin)));

    state_.distance_transform = std::make_shared<collision::Map>(
        collision::distanceTransform(collision::Map::fromOccupancyGrid(*msg)));

    state_.grid = msg;

    // update collision checker
    checker_->reset(*state_.distance_transform);

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /grid ...");
}

void PlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.odometry = msg;
    ego_pose_ = geom::toPose(*msg);

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /ekf/odometry/filtered ...");
}

void PlannerNode::onFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    end_point_ = geom::toVec2(*msg);

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /clicked_point ...");
}

void PlannerNode::onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /tf ...");
}

void PlannerNode::publishPath(const std::vector<geom::Pose>& path) { /** @todo */
}

void PlannerNode::publishGraph(const std::vector<search::Node>& nodes) {
    visualization_msgs::msg::Marker graph_nodes;
    graph_nodes.header.stamp = now();
    graph_nodes.header.frame_id = "odom_ekf";
    graph_nodes.id = 0;
    graph_nodes.type = visualization_msgs::msg::Marker::POINTS;
    graph_nodes.action = visualization_msgs::msg::Marker::ADD;
    graph_nodes.lifetime = rclcpp::Duration::from_seconds(0);
    graph_nodes.scale.x = graph_nodes.scale.y = 0.1;
    graph_nodes.pose.position.z = 0.1;

    for (int i = 0; i < nodes.size(); i++) {
        geometry_msgs::msg::Point p;
        p.x = nodes[i].point.x;
        p.y = nodes[i].point.y;

        graph_nodes.points.push_back(p);

        std_msgs::msg::ColorRGBA p_color;
        p_color.a = 1.0;

        /** @todo
         * need to add checker for a start node (BLUE colored)
         */
        if (nodes[i].is_finish) {
            // GREEN colored node - FINISH
            p_color.g = 1.0;
        } else if (nodes[i].collision) {
            // RED colored node - COLLISION
            p_color.r = 1.0;
        } else {
            // GRAY colored node - REGULAR
            p_color.r = p_color.g = p_color.b = 0.6;
        }

        graph_nodes.colors.push_back(p_color);
    }

    signal_.graph->publish(graph_nodes);

    // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph ...");
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
    if (!state_.distance_transform) {
        return;
    }

    // set grid params
    search::GridParams grid_params{
        .width = params_.width, .height = params_.height, .resolution = params_.resolution};

    // read data from json
    search::YawBins yaw_bins(params_.json_path);
    search::EdgeGeometryCache edge_geometry_cache(params_.json_path);

    // initialize grid
    search::GridBuilder grid_builder =
        search::GridBuilder(grid_params)
            .setEgoPose(ego_pose_)
            .setFinishArea(geom::Circle{end_point_, params_.finish_area_radius})
            .setCollisionChecker(checker_);

    // build grid
    search::Grid grid = grid_builder.build();

    // visualize grid nodes
    publishGraph(grid.nodes_);

    /** @todo add collision checker*/
    // initialize graph
    search::DynamicGraph graph =
        search::DynamicGraph()
            .setGrid(std::make_shared<search::Grid>(grid))
            .setEdgeGeometryCache(std::make_shared<search::EdgeGeometryCache>(edge_geometry_cache))
            .setYawBins(std::make_shared<search::YawBins>(yaw_bins));

    // initialize searcher
    search::Searcher searcher(std::make_shared<search::DynamicGraph>(graph));

    /** @todo search optimal path */
    // searcher.findPath();

    /** @todo get optimal path */
    // const auto path& = searcher.path_;

    /** @todo visualize optimal path */
    // publishPath(path);
}

}  // namespace truck::planner::visualization