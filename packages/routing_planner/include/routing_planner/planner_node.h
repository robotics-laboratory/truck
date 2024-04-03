#pragma once

#include "routing_planner/search.h"

#include "model/model.h"
#include "geom/polyline.h"
#include "truck_msgs/msg/navigation_route.hpp"
#include "truck_msgs/msg/planner_mesh.hpp"
#include "truck_msgs/msg/planner_trajectory.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace truck::routing_planner::node {

struct RoutingMeshParams {
    double step;
    double width;
    double offset;
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg);

    void publishMesh();
    void publishTrajectory();

    void makePlannerTick();

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr finish = nullptr;
        rclcpp::Subscription<truck_msgs::msg::NavigationRoute>::SharedPtr route = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::PlannerMesh>::SharedPtr mesh = nullptr;
        rclcpp::Publisher<truck_msgs::msg::PlannerTrajectory>::SharedPtr trajectory = nullptr;
    } signals_;

    struct State {
        std::optional<geom::Pose> ego = std::nullopt;
        std::optional<geom::Vec2> finish = std::nullopt;
        std::optional<geom::Polyline> route = std::nullopt;
        std::optional<geom::Polyline> trajectory = std::nullopt;
    } state_;

    struct Params {
        std::string model_config;
        double radius;
        search::GraphParams graph;
        search::SearcherParams searcher;
        RoutingMeshParams routing_mesh;
    } params_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::shared_ptr<search::Graph> graph_ = nullptr;
    std::unique_ptr<search::Searcher> searcher_ = nullptr;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

}  // namespace truck::routing_planner::node