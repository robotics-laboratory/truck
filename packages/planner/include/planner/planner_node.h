#pragma once

#include "geom/msg.h"
#include "model/model.h"
#include "planner/search.h"
#include "collision/collision_checker.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace truck::planner::visualization {

class PlannerNode : public rclcpp::Node {
    public:
        PlannerNode();
    private:
        model::Model model_;
        collision::StaticCollisionChecker collision_checker_;
        
        geom::Vec2 end_point_;
        geom::Pose ego_pose_{geom::Vec2(), geom::Vec2()};
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

        rclcpp::TimerBase::SharedPtr timer_;

        // input
        struct Slots {
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
            rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point;
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid;
        } slot_;

        // output
        struct Signal {
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_path;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph;
        } signal_;

        void doPlanningLoop();
        void handleGrid(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid);
        void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);
        void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_point);

        void publishPath(std::vector<geom::Pose>& path);
        void publishGraph(std::vector<search::Node>& nodes);
};

} // namespace truck::planner::visualization