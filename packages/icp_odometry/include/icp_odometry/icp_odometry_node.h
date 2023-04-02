#pragma once

#include "icp_odometry/common.h"
#include "truck_interfaces/msg/icp_odometry_stat.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::icp_odometry {

struct IcpResult {
    truck_interfaces::msg::IcpOdometryStat toOdometryStatMsg() const;

    nav_msgs::msg::Odometry toOdometryMsg() const;

    std_msgs::msg::Header header;
    std_msgs::msg::Header reference_header;

    rclcpp::Duration latency;
    TransformationParameters t;
};

class IcpOdometryNode: public rclcpp::Node {
  public:
    IcpOdometryNode();

    ~IcpOdometryNode() = default;

  private:
    IcpResult makeIcpStep(
        const std_msgs::msg::Header& header, const DataPoints& cloud,
        const std_msgs::msg::Header& refernece_header, const DataPoints& reference_cloud,
        const TransformationParameters& init);

    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

    void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom);

    bool visualize_ = false;
    std::unique_ptr<Matcher::ICP> icp_ = nullptr;

    nav_msgs::msg::Odometry::ConstSharedPtr odometry_ = nullptr;

    sensor_msgs::msg::LaserScan::ConstSharedPtr reference_scan_ = nullptr;
    std::shared_ptr<const DataPoints> reference_cloud_ = nullptr;
    nav_msgs::msg::Odometry::ConstSharedPtr reference_odometry_ = nullptr;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_slot_ = nullptr;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_slot_ = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_signal_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reference_cloud_signal_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_signal_ = nullptr;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_signal_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::IcpOdometryStat>::SharedPtr odometry_stat_signal_ = nullptr;
};

} // namespace truck::icp_odometry