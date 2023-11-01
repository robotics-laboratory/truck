#include "planner/planner_node.h"

namespace truck::planner::visualization {

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace {

std_msgs::msg::ColorRGBA toColorRGBA(const std::vector<double>& vector) {
    VERIFY(vector.size() == 4);

    std_msgs::msg::ColorRGBA color;
    color.r = vector[0];
    color.g = vector[1];
    color.b = vector[2];
    color.a = vector[3];
    return color;
}

} // namespace

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

    params_ = Parameters{
        .graph = Parameters::Graph{
            .type = this->declare_parameter<std::string>("graph.type"),

            .grid = Parameters::Graph::Grid{
                .width = this->declare_parameter<int>("graph.grid.width"),
                .height = this->declare_parameter<int>("graph.grid.height"),
                .resolution = this->declare_parameter<double>("graph.grid.resolution")
            }
        },

        .node = Parameters::Node{
            .z_lev = this->declare_parameter<double>("node.z_lev"),
            .scale = this->declare_parameter<double>("node.scale"),

            .color_rgba = Parameters::Node::ColorRGBA{
                .base = toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.base")),
                .ego = toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.ego")),
                .finish = toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.finish")),
                .finish_area = toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.finish_area")),
                .collision = toColorRGBA(this->declare_parameter<std::vector<double>>("node.color_rgba.collision"))
            }
        },

        .edge = Parameters::Edge{
            .type = this->declare_parameter<std::string>("edge.type"),

            .primitive = Parameters::Edge::Primitive{
                .json_path = this->declare_parameter<std::string>("edge.primitive.json_path")
            },

            .spline = Parameters::Edge::Spline{
                .yaws_count = this->declare_parameter<int>("edge.spline.yaws_count"),
                .sector_angle = this->declare_parameter<double>("edge.spline.sector_angle"),
                .sector_radius = this->declare_parameter<double>("edge.spline.sector_radius"),
            }
        },

        .searcher = Parameters::Searcher{
            .finish_area_radius = this->declare_parameter<double>("searcher.finish_area_radius"),
            .min_obstacle_distance = this->declare_parameter<double>("searcher.min_obstacle_distance")
        }
    };

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    if (!(params_.graph.type == "grid" || params_.graph.type == "adaptive")) {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid graph type: '%s'. Assign 'grid' or 'adaptive' only!",
            params_.graph.type.c_str());
        return;
    }

    if (!(params_.edge.type == "primitive" || params_.edge.type == "spline")) {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid edge type: '%s'. Assign 'primitive' or 'spline' only!",
            params_.edge.type.c_str());
        return;
    }

    if (params_.graph.type == "adaptive" && params_.edge.type == "primitive") {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot use edge type 'primitive' with graph type 'adaptive'!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Graph type: '%s'", params_.graph.type.c_str());
    RCLCPP_INFO(this->get_logger(), "Edge type: '%s'", params_.edge.type.c_str());

    timer_ = this->create_wall_timer(200ms, bind(&PlannerNode::doPlanningLoop, this));
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
        geom::Circle{.center = geom::toVec2(*msg), .radius = params_.searcher.finish_area_radius};
}

void PlannerNode::onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority = "";
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

std_msgs::msg::ColorRGBA PlannerNode::getNodeColor(size_t node_index) const {
    const auto& node = state_.grid->getNodeByIndex(node_index);

    std_msgs::msg::ColorRGBA node_color = params_.node.color_rgba.base;

    if (node.finish_area) {
        node_color = params_.node.color_rgba.finish_area;

        if (node.finish) {
            node_color = params_.node.color_rgba.finish;
        }
    }

    if (node.collision) {
        node_color = params_.node.color_rgba.collision;
    }

    if (node.ego) {
        node_color = params_.node.color_rgba.ego;
    }

    return node_color;
}

void PlannerNode::publishGrid() const {
    visualization_msgs::msg::Marker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom_ekf";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.lifetime = rclcpp::Duration::from_seconds(0);

    msg.scale.x = params_.node.scale;
    msg.scale.y = params_.node.scale;
    msg.scale.z = params_.node.scale;
    msg.pose.position.z = params_.node.z_lev;

    const std::vector<search::Node>& nodes = state_.grid->getNodes();

    for (size_t i = 0; i < nodes.size(); i++) {
        msg.points.push_back(geom::msg::toPoint(nodes[i].point));
        msg.colors.push_back(getNodeColor(i));
    }

    signal_.graph->publish(msg);
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
    if (!checker_->initialized() || !state_.ego_pose.has_value() ||
        !state_.finish_area.has_value()) {
        return;
    }

    search::GridParams grid_params_ {
        params_.graph.grid.width,
        params_.graph.grid.height,
        params_.graph.grid.resolution
    };

    // initialize grid
    state_.grid = std::make_shared<search::Grid>(
        search::Grid(grid_params_, model_->shape())
            .setEgoPose(state_.ego_pose.value())
            .setFinishArea(state_.finish_area.value())
            .setCollisionChecker(checker_)
            .build());

    // visualize grid
    publishGrid();
}

}  // namespace truck::planner::visualization
