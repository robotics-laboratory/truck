#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

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

    Params params_{};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_slot_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_signal_ = nullptr;
};

}  // namespace truck::occupancy_grid