#include <cstdio>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <truck_foxglove/odom_translator.h>

using namespace std::placeholders;

namespace truck
{

    OdometryTranslator::OdometryTranslator() : 
    Node("odom_translator"), 
    _callbackOccured(false),
    _baseLink("camera_link"),
    _odomQoSType("DEFAULT"),
    _odomTopic("rtabmap/odom"),
    _outputTopic("odom_path"),
    _secondRate(0.5),
    _latest()
    {
        _baseLink = this->declare_parameter("base_frame_id", _baseLink);
        _odomQoSType = this->declare_parameter("qos", _odomQoSType);
        _odomTopic = this->declare_parameter("odom_topic", _odomTopic);
        _outputTopic = this->declare_parameter("output_topic", _outputTopic);
        _secondRate = this->declare_parameter<double>("secondRate", _secondRate);

        RCLCPP_INFO(this->get_logger(), "Input odometry topic: /%s", _odomTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output odometry topic: /%s", _outputTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", _baseLink.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", _baseLink.c_str());
        RCLCPP_INFO(this->get_logger(), "Rate of publication in seconds: %f", _secondRate);

        if (_odomQoSType == "DEFAULT") 
            _odomQoSProfile = rmw_qos_profile_default;
        else if (_odomQoSType == "SENSOR_DATA") 
            _odomQoSProfile = rmw_qos_profile_default;
        else if (_odomQoSType == "SYSTEM_DEFAULT") 
            _odomQoSProfile = rmw_qos_profile_system_default;
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error: Invalid value assigned to the QoS parameter. Setting QoS profile to default.");
            _odomQoSProfile = rmw_qos_profile_default;
            _odomQoSType = "DEFAULT";
        }

        RCLCPP_INFO(this->get_logger(), "QoS settings profile: %s", _odomQoSType.c_str());

        _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            _odomTopic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile)),
            std::bind(&OdometryTranslator::callbackOdometry, this, _1));

        _pathPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            _outputTopic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile))
            );

        _republishOdometry = this->create_wall_timer(
            std::chrono::duration<double>(_secondRate),
            std::bind(&OdometryTranslator::republishOdometry, this));

        _warningThread = this->create_wall_timer(
            std::chrono::duration<double>(5.0),
            std::bind(&OdometryTranslator::warningThread, this));
    }

    OdometryTranslator::~OdometryTranslator()
    {

    }

    void OdometryTranslator::republishOdometry() 
    {
        if (_latest != nav_msgs::msg::Odometry()) 
        {
            visualization_msgs::msg::Marker cumtrail;
            cumtrail.pose = _latest.pose.pose;
            cumtrail.header = _latest.header;
            cumtrail.ns = "odometry_path";
            cumtrail.id = _markers.size();
            cumtrail.type = visualization_msgs::msg::Marker::ARROW;
            cumtrail.action = visualization_msgs::msg::Marker::ADD;
            cumtrail.lifetime = rclcpp::Duration::from_seconds(0);
            cumtrail.scale.x = 0.1;
            cumtrail.scale.y = 0.005;
            cumtrail.scale.z = 0.005;
            cumtrail.color.r = 255;
            cumtrail.color.g = 0;
            cumtrail.color.b = 255;
            cumtrail.color.a = 1.0;

            _markers.push_back(cumtrail);
            visualization_msgs::msg::MarkerArray markers;
            markers.markers = _markers;

            _pathPublisher->publish(markers);
            RCLCPP_INFO(
                this->get_logger(),
                "Odometry message added to the path and republished at %f, markers count: %d",
                this->now().seconds(),
                _markers.size());
            _latest = nav_msgs::msg::Odometry();
        }
    }

    void OdometryTranslator::warningThread() 
    {
        if (!_callbackOccured)
            RCLCPP_WARN(
                this->get_logger(),
                "Data from the topic /%s has not been received for more than 5 seconds."
                " Check that the input topic is being published correctly.",
                _odomTopic.c_str());
        else if (_pathPublisher->get_subscription_count() < 1)
            RCLCPP_WARN(
                this->get_logger(),
                "There are no subscribers at /%s, thus the marker messages will not be published.",
                _outputTopic.c_str());
    }

    void OdometryTranslator::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _callbackOccured = true;
        if(_pathPublisher->get_subscription_count() < 1)
            return;

        if (msg->pose.pose.orientation.w == 0) 
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Null odometry has been received at %f, aborting translation",
                msg->header.stamp.sec);
            return;
        } 
        else
        {
            RCLCPP_INFO(this->get_logger(), "Odometry received at %f", msg->header.stamp.sec);
            _latest = *msg;
        }

        _callbackOccured = false;
    }
}