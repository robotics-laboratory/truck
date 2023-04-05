#include "planner/planner_node.h"

using namespace std::chrono_literals;

namespace truck::planner::visualization {

PlannerNode::PlannerNode()
    : Node("planner")
    , model_(this->declare_parameter<std::string>("model_config", "/truck/packages/model/config/model.yaml"))
    , collision_checker_(model_.shape()) {
    
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerNode::handleOdometry, this, std::placeholders::_1)
    );

    slot_.clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerNode::handleClickedPoint, this, std::placeholders::_1)
    );

    slot_.grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid",
        1,
        std::bind(&PlannerNode::handleGrid, this, std::placeholders::_1)
    );

    signal_.graph = this->create_publisher<visualization_msgs::msg::Marker>(
        "/graph", 10
    );

    signal_.optimal_path = this->create_publisher<visualization_msgs::msg::Marker>(
        "/path", 10
    );

    timer_ = this->create_wall_timer(250ms, bind(&PlannerNode::doPlanningLoop, this));
}

void PlannerNode::handleGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid) {
    occupancy_grid_ = occupancy_grid;
    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /grid ...");
}

void PlannerNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
    ego_pose_ = geom::toPose(*odom);
    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /ekf/odometry/filtered ...");
}

void PlannerNode::handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_point) {
    end_point_ = geom::toVec2(*clicked_point);
    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /clicked_point ...");
}

void PlannerNode::publishPath(std::vector<geom::Pose>& path) { /** @todo */ }

void PlannerNode::publishGraph(std::vector<search::Node>& nodes) { /** @todo */ }

void PlannerNode::doPlanningLoop() {
    // update collision checker
    collision_checker_.reset(collision::Map::fromOccupancyGrid(*occupancy_grid_));

    // grid params
    search::GridParams params{10, 10, 0.2};

    // path to json with precalculated primitives and yaws
    std::string json_path = "/truck/packages/planner/data/data.json";

    // read data from json
    search::YawBins yaw_bins(json_path);
    search::EdgeGeometryCache edge_geometry_cache(json_path);

    // output
    std::vector<search::Node> nodes;
    std::vector<geom::Pose> path;
    
    // initialize grid
    search::GridBuilder grid_builder =
        search::GridBuilder(params)
            .setEgoPose(ego_pose_)
            .setFinishArea(geom::Circle{end_point_, 0.5})
            .setCollisionChecker(std::make_shared<collision::StaticCollisionChecker>(collision_checker_));

    // build grid nodes
    search::Grid grid = grid_builder.build();
    
    // get nodes
    nodes = grid.getNodes();
    
    // initialize graph
    search::DynamicGraph graph =
        search::DynamicGraph()
            .setGrid(std::make_shared<search::Grid>(grid))
            .setEdgeGeometryCache(std::make_shared<search::EdgeGeometryCache>(edge_geometry_cache))
            .setYawBins(std::make_shared<search::YawBins>(yaw_bins))
            .setCollisionChecker(std::make_shared<collision::StaticCollisionChecker>(collision_checker_));

    
    // initialize searcher
    search::Searcher searcher(std::make_shared<search::DynamicGraph>(graph));

    // search for optimal path
    searcher.findPath();

    /** @todo get optimal path */
    // path = searcher.getFoundPath();

    /** @todo visualize */
    // publishGraph(nodes);
    // publishPath(path);
}

} // namespace truck::planner::visualization