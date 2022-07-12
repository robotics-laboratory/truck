#include <cstdio>
#include <chrono>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <truck_foxglove/odom_translator.h>

using namespace std::placeholders;

namespace truck {

OdometryTranslator::OdometryTranslator()
    : Node("odom_translator")
    , callback_occured_(false)
    , odom_qos_type_("DEFAULT")
    , odom_topic_("odom")
    , curr_topic_("truck/odom_current")
    , path_topic_("truck/odom_path")
    , rate_in_seconds_(0.5)
    , arrow_length_(0.1)
    , dist_sensitivity_(0.03)
    , debug_(0)
    , array_size_(10)
    , marker_lifetime_(5)
    , show_path_(0)
    , count_(0)
    , latest_() {
    odom_qos_type_ = this->declare_parameter("qos", odom_qos_type_);
    odom_topic_ = this->declare_parameter("odom_topic", odom_topic_);
    path_topic_ = this->declare_parameter("path_topic", path_topic_);
    curr_topic_ = this->declare_parameter("current_topic", curr_topic_);
    rate_in_seconds_ = this->declare_parameter<double>("secondRate", rate_in_seconds_);
    arrow_length_ = this->declare_parameter<double>("arrow_length", arrow_length_);
    dist_sensitivity_ = this->declare_parameter<double>("distance_sensitivity", dist_sensitivity_);
    marker_lifetime_ = this->declare_parameter<double>("path_marker_lifetime", marker_lifetime_);
    array_size_ = this->declare_parameter<int>("markers_count", array_size_);
    debug_ = this->declare_parameter<bool>("debug", debug_);
    show_path_ = this->declare_parameter<bool>("show_path", show_path_);

    RCLCPP_INFO(this->get_logger(), "Input odometry topic: /%s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output odometry path topic: /%s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output odometry current pose topic: /%s", curr_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Rate of the publication in seconds: %f", rate_in_seconds_);
    RCLCPP_INFO(this->get_logger(), "Length of the arrow marker: %f", arrow_length_);
    RCLCPP_INFO(
        this->get_logger(), "Minimal difference between odometry markers: %f", dist_sensitivity_);
    RCLCPP_INFO(this->get_logger(), "The lifetime of a marker in seconds: %f", marker_lifetime_);
    RCLCPP_INFO(this->get_logger(), "Markers count: %d", array_size_);

    if (odom_qos_type_ == "DEFAULT") {
        odom_qos_profile_ = rmw_qos_profile_default;
    }
    else if (odom_qos_type_ == "SENSOR_DATA") {
        odom_qos_profile_ = rmw_qos_profile_default;
    }
    else if (odom_qos_type_ == "SYSTEM_DEFAULT") {
        odom_qos_profile_ = rmw_qos_profile_system_default;
    }
    else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error: Invalid value assigned to the QoS parameter. Setting QoS profile to default.");
        odom_qos_profile_ = rmw_qos_profile_default;
        odom_qos_type_ = "DEFAULT";
    }

    RCLCPP_INFO(this->get_logger(), "QoS settings profile: %s", odom_qos_type_.c_str());
    if(debug_) {
        //the other choice to suppress warning was to use a pragma directive.
        auto crutch = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    }
    
    RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Publish odometry path: %s", show_path_ ? "true" : "false");

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)),
        std::bind(&OdometryTranslator::callbackOdometry, this, _1));

    path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        path_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));

    curr_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        curr_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));

    if (show_path_) {
        republish_odometry_path_ = this->create_wall_timer(
            std::chrono::duration<double>(rate_in_seconds_),
            std::bind(&OdometryTranslator::republishOdometryPath, this));
    }

    warning_thread_ = this->create_wall_timer(
        std::chrono::duration<double>(5.0), std::bind(&OdometryTranslator::warningThread, this));
}

OdometryTranslator::~OdometryTranslator() {}

void OdometryTranslator::republishOdometryPath() {
    if (!(latest_ != nav_msgs::msg::Odometry() && checkOdomDiffSignificance())) {
        return;
    }

    visualization_msgs::msg::Marker trail = generateMarker(latest_);
    trail.id = count_ + 1;
    trail.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);

    markers_.push_back(trail);
    if (markers_.size() > 0 && markers_.size() > array_size_) {
        markers_.pop_front();
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers = { markers_.begin(), markers_.end() };

    path_publisher_->publish(markers);
    RCLCPP_INFO(
        this->get_logger(),
        "Odometry message added to the path and republished at %f, markers count: %d",
        this->now().seconds(),
        markers_.size());

    latest_ = nav_msgs::msg::Odometry();
    count_++;
}

void OdometryTranslator::warningThread() {
    if (!callback_occured_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Data from the topic /%s has not been received for more than 5 seconds."
            " Check that the input topic is being published correctly.",
            odom_topic_.c_str());
    }
    else if (path_publisher_->get_subscription_count() < 1) {
        RCLCPP_WARN(
            this->get_logger(),
            "There are no subscribers at /%s, thus the marker messages will not be published.",
            path_topic_.c_str());
    }
}

bool OdometryTranslator::checkOdomDiffSignificance() {
    if (markers_.size() == 0) {
        return true;
    }

    tf2::Quaternion qLast;
    geometry_msgs::msg::Pose lastPose = markers_.back().pose;
    tf2::fromMsg(lastPose.orientation, qLast);
    tf2::Quaternion qLatest;
    tf2::fromMsg(latest_.pose.pose.orientation, qLatest);

    tf2::Vector3 direction(1, 0, 0);
    tf2::Vector3 lastVector = tf2::quatRotate(qLast, direction);
    tf2::Vector3 latestVector = tf2::quatRotate(qLatest, direction);

    tf2::Vector3 lastPosition(lastPose.position.x, lastPose.position.y, lastPose.position.z);
    tf2::Vector3 latestPosition(
        latest_.pose.pose.position.x, latest_.pose.pose.position.y, latest_.pose.pose.position.z);

    tf2::Vector3 lastEndPos = lastPosition + lastVector * arrow_length_;
    tf2::Vector3 latestEndPos = latestPosition + latestVector * arrow_length_;

    double distance = lastEndPos.distance(latestEndPos);
    RCLCPP_DEBUG(
        this->get_logger(),
        "Info: distance between the latest and the last odometry is %f",
        distance);
    if (distance >= dist_sensitivity_) {
        return true;
    }
    return false;
}

void OdometryTranslator::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    callback_occured_ = true;
    if (path_publisher_->get_subscription_count() < 1) { 
        return;
    }

    if (msg->pose.pose.orientation.w == 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Null odometry has been received at %f, aborting translation",
            msg->header.stamp.sec);
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Odometry received at %f", msg->header.stamp.sec);
        latest_ = *msg;
    }

    visualization_msgs::msg::Marker marker = this->generateMarker(*msg);
    curr_publisher_->publish(marker);

    callback_occured_ = false;
}

visualization_msgs::msg::Marker OdometryTranslator::generateMarker(
    const nav_msgs::msg::Odometry& msg) {
    visualization_msgs::msg::Marker trail;
    trail.pose = latest_.pose.pose;
    trail.header = latest_.header;
    trail.ns = "odometry_path";
    trail.id = 0;
    trail.type = visualization_msgs::msg::Marker::ARROW;
    trail.action = visualization_msgs::msg::Marker::ADD;
    trail.lifetime = rclcpp::Duration::from_seconds(0);
    trail.scale.x = arrow_length_;
    trail.scale.y = 0.005;
    trail.scale.z = 0.005;
    trail.color.r = 255;
    trail.color.g = 0;
    trail.color.b = 255;
    trail.color.a = 1.0;

    return trail;
}
}  // namespace truck
