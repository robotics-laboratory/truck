#pragma once

#include "tree_planner/tree_planner.h"

#include "collision/collision_checker.h"
#include "geom/msg.h"
#include "model/model.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/qos.hpp>

namespace truck::planner::visualization {

class TreePlannerNode : public rclcpp::Node {
  public:
    TreePlannerNode();

  private:
    void OnGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void OnFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    std_msgs::msg::ColorRGBA GetNodeColor(const tree_planner::Node& node) const;

    void PublishTree() const;
    void PublishPath() const;
    void PublishGoal() const;

    void Publish() const;

    std::optional<geom::Transform> GetLatestTranform(
        std::string_view source, std::string_view target);

    void DoPlanningLoop();

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point = nullptr;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static = nullptr;
    } slot_;

    struct Signal {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr finish = nullptr;
    } signal_;

    struct State {
        std::shared_ptr<collision::Map> distance_transform = nullptr;

        nav_msgs::msg::Odometry::SharedPtr odom = nullptr;
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid = nullptr;

        std::optional<geom::Vec2> ego_pose = std::nullopt;
        std::optional<geom::BBox> planning_space = std::nullopt;
        std::optional<geom::Circle> finish_area = std::nullopt;
    } state_;

    struct Parameters {
        planner::tree_planner::TreePlannerParams tree_planner_params;

        struct NodeParams {
            double z_lev;
            double scale;
            std_msgs::msg::ColorRGBA base_color;
            std_msgs::msg::ColorRGBA start_color;
            std_msgs::msg::ColorRGBA goal_color;
            std_msgs::msg::ColorRGBA optimal_goal_color;
        } node;
    } params_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    tree_planner::TreePlanner planner_;
};

}  // namespace truck::planner::visualization
