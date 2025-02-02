#pragma once

#include "common/math.h"
#include "geom/vector.h"

#include <image_geometry/pinhole_camera_model.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace truck::occupancy_grid {

class OccupancyGridNode : public rclcpp::Node {
  public:
    OccupancyGridNode();

  private:
    void handleCameraDepth(sensor_msgs::msg::Image::ConstSharedPtr image);

    void handlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

    void handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info);

    void publishOccupancyGrid();

    std::optional<tf2::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Params {
        std::string frame_id = "odom_ekf";
        double resolution = 0.1;
        double radius = 20;
        bool enable_lidar_grid = false;
        bool enable_lidar_cloud = false;
        bool enable_camera_grid = false;
        bool enable_camera_cloud = false;
        Limits<double> camera_view_height = {-0.165, 0.15};
        double camera_view_distance = 2;
        Limits<double> lidar_view_height = {-0.15, 0.05};
    } params_{};

    struct Slots {
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_cloud = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_cloud = nullptr;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid = nullptr;
    } signal_;

    struct State {
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info = nullptr;
        sensor_msgs::msg::PointCloud2::SharedPtr odom_camera_points = nullptr;
        sensor_msgs::msg::PointCloud2::SharedPtr odom_lidar_points = nullptr;
    } state_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

}  // namespace truck::occupancy_grid
