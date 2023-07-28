#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

namespace truck::occupancy_grid {

struct Params {
    double resolution = 0.1;
    double radius = 20;
    bool enable_lidar_grid = false;
    bool enable_camera_grid = false;
    bool enable_camera_cloud = false;
    double camera_view_hmin = -0.05;
    double camera_view_hmax = 0.05;
};

class OccupancyGridNode : public rclcpp::Node {
  public:
    OccupancyGridNode();

    ~OccupancyGridNode() = default;

  private:

    void handleCameraDepth(sensor_msgs::msg::Image::ConstSharedPtr image);

    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    
    void handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info);

    void publishOccupancyGrid();

    sensor_msgs::msg::PointCloud2::SharedPtr MakeCloud(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const image_geometry::PinholeCameraModel& model,
        const geometry_msgs::msg::TransformStamped& from_camera_to_lidar_tf_msg,
        double range_max = 0.0);

    Params params_{};

    struct Slots {
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar = nullptr;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera = nullptr;
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info = nullptr;    
    } slot_;
    
    struct Signals {
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_cloud = nullptr;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid = nullptr;
    } signal_;  
    
    struct State {
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info = nullptr;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_point = nullptr;
    sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_point = nullptr;
    } state_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

}  // namespace truck::occupancy_grid