#pragma once

#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace truck {
class OdometryTranslator : public rclcpp::Node {
  public:
    explicit OdometryTranslator();
    ~OdometryTranslator() noexcept;

  private:
    void warningThread();
    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    visualization_msgs::msg::Marker generateMarker(const nav_msgs::msg::Odometry& msg);
    bool checkOdomDiffSignificance();
    void republishOdometryPath();

    rmw_qos_profile_t odom_qos_profile_;
    nav_msgs::msg::Odometry latest_;
    std::deque<visualization_msgs::msg::Marker> markers_;
    bool callback_occured_;
    int count_;

    rclcpp::TimerBase::SharedPtr republish_odometry_path_;
    rclcpp::TimerBase::SharedPtr warning_thread_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curr_publisher_;

    // parameters
    std::string odom_qos_type_;
    std::string odom_topic_;
    std::string path_topic_;
    std::string curr_topic_;
    bool debug_;
    bool show_path_;
    int array_size_;
    double marker_lifetime_;
    double rate_in_seconds_;
    double dist_sensitivity_;
    double arrow_length_;
};
}  // namespace truck
