//#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <rclcpp/rclcpp.hpp>

namespace truck::occupancy_grid {

struct Params {
    double resolution = 0.1;
    double radius = 20;
};

class OccupancyGridNode : public rclcpp::Node {
  public:
    OccupancyGridNode();

    ~OccupancyGridNode() = default;

  private:
    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    
    void handleCameraScan(sensor_msgs::msg::Image::ConstSharedPtr image);

    void cameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info);

    Params params_{};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_slot_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_signal_ = nullptr;
    
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_ = nullptr;
};

}  // namespace truck::occupancy_grid