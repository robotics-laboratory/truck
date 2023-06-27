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

using Color = std_msgs::msg::ColorRGBA;

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    Color toColorRGBA(const std::vector<double>& vector);
    Color getNodeColor(size_t node_index, const search::Grid& grid) const;

    void publishGrid(const search::Grid& grid);

    std::optional<geom::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    void doPlanningLoop();

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point = nullptr;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static = nullptr;
    } slot_;

    struct Signal {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph = nullptr;
    } signal_;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr odom = nullptr;
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid = nullptr;
        std::shared_ptr<collision::Map> distance_transform = nullptr;
        std::optional<geom::Pose> ego_pose = std::nullopt;
        std::optional<geom::Circle> finish_area = std::nullopt;
    } state_;

    struct Parameters {
        search::GridParams grid;

        struct NodeParams {
            double z_lev;
            double scale;
            Color base_color;
            Color start_color;
            Color finish_base_color;
            Color finish_accent_color;
            Color collision_color;
        } node;
    } params_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

}  // namespace truck::planner::visualization