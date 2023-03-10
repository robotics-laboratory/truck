#include "planner/visualization.h"

namespace truck::planner::visualization {

PlannerVisualizationNode::PlannerVisualizationNode()
    : Node("planner_visualization_node") {

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerVisualizationNode::topicCallbackOdometry, this, std::placeholders::_1)
    );

    clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        bind(&PlannerVisualizationNode::topicCallbackClickedPoint, this, std::placeholders::_1)
    );

    graph_path_= this->create_publisher<visualization_msgs::msg::Marker>(
        "/graph/path", 10
    );

    graph_nodes_base_= this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/graph/nodes/base", 10
    );

    graph_nodes_accent_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/graph/nodes/accent", 10
    );

    timer_ = this->create_wall_timer(250ms, bind(&PlannerVisualizationNode::timerCallback, this));
}

void PlannerVisualizationNode::topicCallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
    ego_pose_ = geom::Vec2(
        odom->pose.pose.position.x,
        odom->pose.pose.position.y
    );

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /ekf/odometry/filtered ...");
}

void PlannerVisualizationNode::topicCallbackClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_point) {
    end_point_ = geom::Vec2(
        clicked_point->point.x,
        clicked_point->point.y
    );

    // RCLCPP_INFO(this->get_logger(), "Subscription to topic /clicked_point ...");
}

void PlannerVisualizationNode::publishGraph() {
    pcl::PointXYZRGB pcl_vertex;
    sensor_msgs::msg::PointCloud2 pointcloud2_base, pointcloud2_accent;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_base, pointcloud_accent;

    for (const search::Node& node : nodes_) {
        if (node.is_finish) {
            pcl_vertex = pcl::PointXYZRGB(255, 0, 0);

            pcl_vertex.x = node.point.x;
            pcl_vertex.y = node.point.y;
            pcl_vertex.z = 0.01;

            pointcloud_accent.points.push_back(pcl_vertex);
        } else {
            pcl_vertex = pcl::PointXYZRGB(0, 0, 255);

            pcl_vertex.x = node.point.x;
            pcl_vertex.y = node.point.y;
            pcl_vertex.z = 0.01;

            pointcloud_base.points.push_back(pcl_vertex);
        }
    }

    pcl::toROSMsg(pointcloud_base, pointcloud2_base);
    pcl::toROSMsg(pointcloud_accent, pointcloud2_accent);

    pointcloud2_base.header.stamp = pointcloud2_accent.header.stamp = this->now();
    pointcloud2_base.header.frame_id = pointcloud2_accent.header.frame_id = "odom_ekf";

    graph_nodes_base_->publish(pointcloud2_base);
    // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph/nodes/base ...");

    graph_nodes_accent_->publish(pointcloud2_accent);
    // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph/nodes/accent ...");
}

void PlannerVisualizationNode::publishPath() {
    /** @todo */
}

void PlannerVisualizationNode::timerCallback() {
    auto builder = search::GridBuilder(search::GridParams{10, 10, 0.2})
         .setEgoPose(ego_pose_)
         .setFinishArea(geom::Circle{end_point_, 0.5});
    
    nodes_ = builder.build();

    publishGraph();
    // publishPath();
}

}