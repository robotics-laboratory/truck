#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace truck::occupancy_grid {

struct Params {
    double resolution = 0.1;
    double radius = 20;
    bool enable_lidar_grid = false;
    bool enable_camera_grid = false;
    bool enable_camera_cloud = false;
};

class OccupancyGridNode : public rclcpp::Node {
  public:
    OccupancyGridNode();

    ~OccupancyGridNode() = default;

  private:

    void handleImageScan(sensor_msgs::msg::Image::ConstSharedPtr image);

    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    
    void handleCameraScan(sensor_msgs::msg::Image::ConstSharedPtr image);

    void handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info);

    Params params_{};

    struct Slots {
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_handle = nullptr;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_handle = nullptr;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_handle = nullptr;
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info = nullptr;
    } slot_;
    
    struct Signals {
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image = nullptr;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_cloud = nullptr;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid = nullptr;
    } signal_;  
    
    struct State{
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info = nullptr;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_output = std::make_shared<sensor_msgs::msg::PointCloud2>();
    } state_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

}  // namespace truck::occupancy_grid