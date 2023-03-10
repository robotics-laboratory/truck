#pragma once

#include "planner/search.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <vector>
#include <string>

using namespace std::chrono_literals;

namespace truck::planner::visualization {

class PlannerVisualizationNode : public rclcpp::Node {
    public:
        PlannerVisualizationNode();
    private:
        geom::Vec2 ego_pose_;
        geom::Vec2 end_point_;
        std::vector<geom::Vec2> path_;
        std::vector<search::Node> nodes_;

        rclcpp::TimerBase::SharedPtr timer_;

        // input
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_;

        // output
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_path_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graph_nodes_base_, graph_nodes_accent_;

        void timerCallback();
        void topicCallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);
        void topicCallbackClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_point);

        void publishGraph();
        void publishPath();
};

}