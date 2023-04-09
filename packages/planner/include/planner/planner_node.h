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
#include <tf2_ros/buffer.h>

namespace truck::planner::visualization {

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void doPlanningLoop();

    void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinishPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    void publishPath(const std::vector<geom::Pose>& path);
    void publishGraph(const std::vector<search::Node>& nodes);

    std::optional<geom::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point = nullptr;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static = nullptr;
    } slot_;

    struct Signal {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_path = nullptr;
    } signal_;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr odometry = nullptr;
        nav_msgs::msg::OccupancyGrid::SharedPtr grid = nullptr;
        std::shared_ptr<collision::Map> distance_transform = nullptr;
    } state_;

    struct Parameters {
        int width;
        int height;
        float resolution;
        float finish_area_radius;
        std::string json_path;
    } params_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    geom::Vec2 end_point_;
    geom::Pose ego_pose_{geom::Vec2(), geom::Vec2()};
};

}  // namespace truck::planner::visualization