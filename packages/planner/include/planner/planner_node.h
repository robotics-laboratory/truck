#pragma once

#include "geom/polyline.h"
#include "truck_msgs/msg/navigation_route.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace truck::planner::node {

struct RoutingMeshParams {
    double width;
    double dist;
    double offset;
    double border_radius;
};

struct LatticeMeshParams {
    double dist;
    double radius;
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg);

    void makePlannerTick();

    void publishMesh() const;
    void publishTrajectory() const;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<truck_msgs::msg::NavigationRoute>::SharedPtr route = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory = nullptr;
    } signals_;

    struct State {
        std::optional<geom::Pose> ego = std::nullopt;
        std::optional<std::vector<geom::Vec2>> mesh = std::nullopt;
        std::optional<geom::Polyline> route = std::nullopt;
        std::optional<geom::Polyline> trajectory = std::nullopt;
    } state_;

    struct Params {
        enum class PlannerMode : uint8_t {
            routingMesh = 0,
            latticeMesh = 1
        } mode;

        RoutingMeshParams routing_mesh;
        LatticeMeshParams lattice_mesh;
    } params_;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

}  // namespace truck::planner::node