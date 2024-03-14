#pragma once

#include "geom/rtree.h"
#include "map/map.h"
#include "navigation/search.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"
#include "truck_msgs/msg/navigation_mesh.hpp"
#include "truck_msgs/msg/navigation_route.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::routing {

struct Route {
    Route();
    Route(const geom::Polyline& polyline);

    double distance(const geom::Vec2& point) const;
    size_t postfixIndex(const geom::Vec2& point, double postfix) const;

    geom::RTree rtree;
    geom::Polyline polyline;
};

struct Cache {
    Cache();
    Cache(const navigation::graph::Graph& graph);

    geom::Polyline findPath(const geom::Vec2& from, const geom::Vec2& to) const;

    geom::RTree rtree;
    navigation::graph::Graph graph;
};

class RoutingNode : public rclcpp::Node {
  public:
    RoutingNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void updateRoute();

    void makeMeshTick();
    void makeRouteTick();

    void publishMesh() const;
    void publishRoute() const;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr finish = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::NavigationMesh>::SharedPtr mesh = nullptr;
        rclcpp::Publisher<truck_msgs::msg::NavigationRoute>::SharedPtr route = nullptr;
    } signals_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr mesh = nullptr;
        rclcpp::TimerBase::SharedPtr route = nullptr;
    } timers_;

    struct State {
        std::optional<Route> route;
        std::optional<geom::Vec2> ego = std::nullopt;
        std::optional<geom::Vec2> finish = std::nullopt;
    } state_;

    struct Params {
        std::string map_config;

        struct Route {
            double max_ego_dist;
            double postfix_len;
            double spline_step;
        } route;

        navigation::mesh::MeshParams mesh;
        navigation::graph::GraphParams graph;
    } params_;

    Cache cache_;

    std::unique_ptr<map::Map> map_ = nullptr;
    std::unique_ptr<navigation::mesh::MeshBuilder> mesh_builder_ = nullptr;
    std::unique_ptr<navigation::graph::GraphBuilder> graph_builder_ = nullptr;
};

}  // namespace truck::routing