#pragma once

#include "geom/pose.h"

#include <rclcpp/rclcpp.hpp>
#include <pointmatcher/PointMatcher.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <optional>

namespace truck::localization {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

struct LocalizationParams {
    std::string map_scan_path;
    std::string icp_config_path;

    double bbox_radius;
};

class LocalizationNode : public rclcpp::Node {
  public:
    LocalizationNode();

  private:
    void initializeParams();
    void initializeTransforms();
    void initializeTopicHandlers();

    void loadMap();

    void onReset(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    void publishMapScan();

    void makeTransformationICP();
    void publishTransform();
    void makeLocalizationTick();

    std::optional<tf2::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Map {
        DataPoints data_points;
        sensor_msgs::msg::PointCloud2 point_cloud;
    } map_;

    struct State {
        struct Transformations {
            tf2::Transform world_base;
            tf2::Transform ekf_base;
            tf2::Transform world_ekf;
            tf2::Transform base_lidar_link;
        } tf;

        std::optional<sensor_msgs::msg::LaserScan> scan = std::nullopt;
    } state_;

    struct Signals {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_scan = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_scan = nullptr;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transform = nullptr;
    } signals_;

    struct Slots {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose = nullptr;
    } slots_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr map_scan = nullptr;
        rclcpp::TimerBase::SharedPtr transform = nullptr;
    } timers_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    Matcher::ICP icp_;

    LocalizationParams params_;
};

}  // namespace truck::localization