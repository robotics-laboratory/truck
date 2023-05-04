#pragma once

#include "planner/search.h"

#include "geom/msg.h"
#include "model/model.h"
#include "collision/collision_checker.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/buffer.h>

namespace truck::planner::visualization {

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    search::Color setColorFromVector(const std::vector<double>& vector) const;
    std_msgs::msg::ColorRGBA setColorRGBAfromColor(const search::Color& color) const;

    void doPlanningLoop();

    void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    std_msgs::msg::ColorRGBA setNodeColor(size_t node_index, search::Grid& grid) const;

    void publishGrid(search::Grid& grid);
    void publishPath(search::Searcher& searcher);
    
    void resetPath();

    std::optional<geom::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point = nullptr;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static = nullptr;
    } slot_;

    struct Signal {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path = nullptr;
    } signal_;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr odom = nullptr;
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid = nullptr;
        std::shared_ptr<collision::Map> distance_transform = nullptr;
    } state_;

    struct Parameters {
        search::GridParams grid_params;
        search::GraphParams graph_params;
        search::EdgeGeometryCache edge_geometry_cache;
        std::optional<geom::Pose> ego_pose = std::nullopt;
        std::optional<geom::Circle> finish_area = std::nullopt;
    } params_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

}  // namespace truck::planner::visualization