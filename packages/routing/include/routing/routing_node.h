#pragma once

#include "map/map.h"
#include "navigation/search.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"
#include "truck_msgs/msg/navigation_mesh.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <boost/geometry.hpp>

namespace truck::routing {

namespace bg = boost::geometry;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

class RoutingNode : public rclcpp::Node {
  public:
    RoutingNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void routingLoop();

    void publishRoute() const;
    void publishRouteSmooth() const;
    void publishNavigationMesh() const;

    void updateRoute();

    double polylineLength(const geom::Polyline& polyline) const;
    geom::Polyline polylineSmooth(const geom::Polyline& polyline) const;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr finish = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::NavigationMesh>::SharedPtr nav_mesh = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_smooth = nullptr;
    } signals_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr main = nullptr;
        rclcpp::TimerBase::SharedPtr nav_mesh = nullptr;
    } timers_;

    struct State {
        std::optional<geom::Vec2> ego = std::nullopt;
        std::optional<geom::Vec2> finish = std::nullopt;

        struct Route {
            geom::Polyline polyline;
            double length;
        } route;

        struct RouteSmooth {
            geom::Polyline polyline;
            double length;
            RTree rtree;
        } route_smooth;
    } state_;

    struct Cache {
        RTree rtree_nodes;
        navigation::graph::Graph graph;
    } cache_;

    struct Params {
        std::string map_config;

        struct Route {
            double max_ego_dist;
            double postfix_len;

            struct Spline {
                size_t degree;
                double step;
            } spline;
        } route;

        navigation::mesh::MeshParams mesh;
        navigation::graph::GraphParams graph;
    } params_;

    std::unique_ptr<map::Map> map_ = nullptr;
    std::unique_ptr<navigation::mesh::MeshBuilder> mesh_builder_ = nullptr;
    std::unique_ptr<navigation::graph::GraphBuilder> graph_builder_ = nullptr;
};

}  // namespace truck::routing