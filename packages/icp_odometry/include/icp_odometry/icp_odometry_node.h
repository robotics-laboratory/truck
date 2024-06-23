#pragma once

#include "icp_odometry/common.h"
#include "truck_msgs/msg/icp_odometry_stat.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::icp_odometry {

struct IcpResult {
    truck_msgs::msg::IcpOdometryStat toOdometryStatMsg() const;

    nav_msgs::msg::Odometry toOdometryMsg() const;

    std_msgs::msg::Header header;
    std_msgs::msg::Header reference_header;

    rclcpp::Duration latency;
    TransformationParameters t;
};

class IcpOdometryNode : public rclcpp::Node {
  public:
    IcpOdometryNode();

    ~IcpOdometryNode() = default;

  private:
    IcpResult makeIcpStep(
        const std_msgs::msg::Header& header, const DataPoints& cloud,
        const std_msgs::msg::Header& reference_header, const DataPoints& reference_cloud,
        const TransformationParameters& init);

    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

    void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom);

    struct State {
        nav_msgs::msg::Odometry::ConstSharedPtr odometry = nullptr;
        sensor_msgs::msg::LaserScan::ConstSharedPtr reference_scan = nullptr;
        std::shared_ptr<const DataPoints> reference_cloud = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr reference_odometry = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } slot_;

    struct Signal {
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
        rclcpp::Publisher<truck_msgs::msg::IcpOdometryStat>::SharedPtr odometry_stat = nullptr;
    } signal_;

    std::unique_ptr<Matcher::ICP> icp_ = nullptr;
};

}  // namespace truck::icp_odometry
