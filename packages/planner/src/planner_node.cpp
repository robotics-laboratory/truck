#include "planner/planner_node.h"

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
    TfCallback static_tf_callback = std::bind(&PlannerNode::onTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    signal_.graph = this->create_publisher<visualization_msgs::msg::Marker>("/graph", 10);

    signal_.path = this->create_publisher<visualization_msgs::msg::Marker>("/path", 10);

    signal_.finish = this->create_publisher<visualization_msgs::msg::Marker>("/finish", 10);

    search::GridParams grid_params {
        .width = this->declare_parameter<int>("grid.width"),
        .height = this->declare_parameter<int>("grid.height"),
        .resolution = this->declare_parameter<double>("grid.resolution")
    };

    search::EdgeParams edge_params {
        .yaws_count = this->declare_parameter<int>("edge.yaws_count"),
        .type = this->declare_parameter<std::string>("edge.type"),

        .primitive = search::EdgeParams::PrimitiveParams {
            .json_path = this->declare_parameter<std::string>("edge.primitive.json_path")
        }
    };

    search::SearcherParams searcher_params {
        .max_vertices_count = this->declare_parameter<int>("searcher.max_vertices_count"),
        .finish_area_size = this->declare_parameter<double>("searcher.finish_area_size"),
        .min_obstacle_distance = this->declare_parameter<double>("searcher.min_obstacle_distance"),
        .max_node_position_error = this->declare_parameter<double>("searcher.max_node_position_error")
    };

    params_ = Parameters{
        .grid = grid_params,

        .edge = edge_params,

        .searcher = searcher_params,

        .node = Parameters::NodeParams{
            .z_lev = this->declare_parameter<double>("node.z_lev"),
            .scale = this->declare_parameter<double>("node.scale"),

            .base_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.base")),

            .ego_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.ego")),

            .finish_base_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.finish_base")),

            .finish_accent_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.finish_accent")),

            .collision_color = toColorRGBA(
                this->declare_parameter<std::vector<double>>("node.color_rgba.collision")),
        }
    };

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ = this->create_wall_timer(200ms, bind(&PlannerNode::doPlanningLoop, this));

    if (params_.edge.type == "primitive") {
        RCLCPP_INFO(this->get_logger(), "Edge type: primitive");

        edge_cache_ = std::make_shared<search::PrimitiveCache>(params_.edge);
    } else {
        RCLCPP_WARN(this->get_logger(), "Incorrect edge type!");
    }

    searcher_.setParams(params_.searcher);
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
    state_.finish_area =
        geom::Square{.center = geom::toVec2(*msg), .size = params_.searcher.finish_area_size};

    searcher_.reset();
}

void PlannerNode::onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

std_msgs::msg::ColorRGBA PlannerNode::getNodeColor(size_t node_index) const {
    const auto& node = state_.grid->getNodeByIndex(node_index);

    std_msgs::msg::ColorRGBA node_color = params_.node.base_color;

    if (state_.grid->getFinishAreaNodesIndices().find(node_index) !=
        state_.grid->getFinishAreaNodesIndices().end()) {
        node_color = params_.node.finish_base_color;

        if (node_index == state_.grid->getFinishNodeIndex()) {
            node_color = params_.node.finish_accent_color;
        }
    }

    if (node.collision) {
        node_color = params_.node.collision_color;
    }

    if (node_index == state_.grid->getEgoNodeIndex()) {
        node_color = params_.node.ego_color;
    }

    return node_color;
}

void PlannerNode::publishGrid() const {
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

    const std::vector<search::Node>& nodes = state_.grid->getNodes();

    for (size_t i = 0; i < nodes.size(); i++) {
        marker.points.push_back(geom::msg::toPoint(nodes[i].point));
        marker.colors.push_back(getNodeColor(i));
    }

    signal_.graph->publish(marker);
}

void PlannerNode::publishPath() const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom_ekf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position.z = params_.node.z_lev;
    marker.color.a = 0.6;
    marker.color.g = 1.0;

    const geom::Poses& poses = searcher_.getPath();

    for (size_t i = 0; i < poses.size(); i++) {
        marker.points.push_back(geom::msg::toPoint(poses[i].pos));
    }

    signal_.path->publish(marker);
}

void PlannerNode::publishFinish() const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom_ekf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker.scale.x = params_.searcher.finish_area_size;
    marker.scale.y = params_.searcher.finish_area_size;
    marker.scale.z = params_.node.z_lev * 2;

    marker.color = params_.node.finish_base_color;

    geom::Vec2 position = state_.grid->getNodeByIndex(state_.grid->getFinishNodeIndex()).point;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = params_.node.z_lev;

    signal_.finish->publish(marker);
}

void PlannerNode::publish() const {
    publishGrid();
    publishPath();
    publishFinish();
}

void PlannerNode::reset() const {
    visualization_msgs::msg::Marker marker;
    signal_.graph->publish(marker);
    signal_.path->publish(marker);
    signal_.finish->publish(marker);
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
    if (!checker_->initialized() || !state_.ego_pose.has_value() || !state_.finish_area.has_value()) {
        return;
    }

    state_.grid = std::make_shared<search::Grid>(
        search::Grid(params_.grid, model_->shape())
            .setEgoPose(state_.ego_pose.value())
            .setFinishArea(state_.finish_area.value())
            .setCollisionChecker(checker_)
    );

    if (!state_.grid->build()) {
        RCLCPP_WARN(this->get_logger(), "Finish point is out of grid bounds!");
        reset();
        return;
    }

    searcher_.setGrid(state_.grid);
    searcher_.setEdgeCache(edge_cache_);
    searcher_.setCollisionChecker(checker_);

    if (!searcher_.findPath()) {
        RCLCPP_WARN(this->get_logger(), "Path not found!");
    }

    publish();
}

}  // namespace truck::planner::visualization