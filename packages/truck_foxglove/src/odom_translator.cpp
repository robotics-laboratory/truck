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

namespace truck
{

    OdometryTranslator::OdometryTranslator() : 
    Node("odom_translator"), 
    _callbackOccured(false),
    _baseLink("camera_link"),
    _odomQoSType("DEFAULT"),
    _odomTopic("rtabmap/odom"),
    _curTopic("truck/odom_current"),
    _pathTopic("truck/odom_path"),
    _secondRate(0.5),
    _arrowLength(0.1),
    _distSensitivity(0.03),
    _debug("false"),
    _arraySize(10),
    _markerLongevity(5),
    _showPath("false"),
    _count(0),
    _latest()
    {
        _baseLink = this->declare_parameter("base_frame_id", _baseLink);
        _odomQoSType = this->declare_parameter("qos", _odomQoSType);
        _odomTopic = this->declare_parameter("odom_topic", _odomTopic);
        _pathTopic = this->declare_parameter("path_topic", _pathTopic);
        _curTopic = this->declare_parameter("current_topic", _curTopic);
        _secondRate = this->declare_parameter<double>("secondRate", _secondRate);
        _arrowLength = this->declare_parameter<double>("arrow_length", _arrowLength);
        _distSensitivity = this->declare_parameter<double>("distance_sensitivity", _distSensitivity);
        _markerLongevity = this->declare_parameter<double>("path_marker_lifetime", _markerLongevity);
        _arraySize = this->declare_parameter<int>("markers_count", _arraySize);
        _debug = this->declare_parameter("debug", _debug);
        _showPath = this->declare_parameter("show_path", _showPath);

        RCLCPP_INFO(this->get_logger(), "Input odometry topic: /%s", _odomTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output odometry path topic: /%s", _pathTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output odometry current pose topic: /%s", _curTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", _baseLink.c_str());
        RCLCPP_INFO(this->get_logger(), "Rate of the publication in seconds: %f", _secondRate);
        RCLCPP_INFO(this->get_logger(), "Length of the arrow marker: %f", _arrowLength);
        RCLCPP_INFO(this->get_logger(), "Minimal difference between odometry markers: %f", _distSensitivity);
        RCLCPP_INFO(this->get_logger(), "The lifetime of a marker in seconds: %f", _markerLongevity);
        RCLCPP_INFO(this->get_logger(), "Markers count: %d", _arraySize);

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

        _debug = _debug == "true" || _debug == "false" ? _debug : "false";
        if(_debug == "true")
#pragma GCC diagnostic ignored "-Wunused-result"
            rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        RCLCPP_INFO(this->get_logger(), "Debug mode: %s", _debug.c_str());

        _showPath = _showPath == "true" || _showPath == "false" ? _showPath : "false";
        RCLCPP_INFO(this->get_logger(), "Publish odometry path: %s", _showPath.c_str());

        _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            _odomTopic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile)),
            std::bind(&OdometryTranslator::callbackOdometry, this, _1));

        _pathPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            _pathTopic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile))
            );

        _curPublisher = this->create_publisher<visualization_msgs::msg::Marker>(
            _curTopic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile))
            );

        if(_showPath == "true")
            _republishOdometryPath = this->create_wall_timer(
                std::chrono::duration<double>(_secondRate),
                std::bind(&OdometryTranslator::republishOdometryPath, this));

        _warningThread = this->create_wall_timer(
            std::chrono::duration<double>(5.0),
            std::bind(&OdometryTranslator::warningThread, this));
    }

    OdometryTranslator::~OdometryTranslator()
    {

    }

    void OdometryTranslator::republishOdometryPath() 
    {
        if (_latest != nav_msgs::msg::Odometry() && checkOdomDiffSignificance()) 
        {
            visualization_msgs::msg::Marker cumtrail = generateMarker(_latest);
            cumtrail.id = _count + 1;
            cumtrail.lifetime = rclcpp::Duration::from_seconds(_markerLongevity);

            _markers.push_back(cumtrail);
            if (_markers.size() > 0 && _markers.size() > _arraySize) 
                _markers.erase(_markers.begin());

            visualization_msgs::msg::MarkerArray markers;
            markers.markers = _markers;

            _pathPublisher->publish(markers);
            RCLCPP_INFO(
                this->get_logger(),
                "Odometry message added to the path and republished at %f, markers count: %d",
                this->now().seconds(),
                _markers.size());

            cumtrail.id = 0;
            cumtrail.lifetime = rclcpp::Duration::from_seconds(0);

            _curPublisher->publish(cumtrail);

            _latest = nav_msgs::msg::Odometry();
            _count++;
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
                _pathTopic.c_str());
    }

    bool OdometryTranslator::checkOdomDiffSignificance()
    {
        if (_markers.size() == 0) 
            return true;

        tf2::Quaternion qLast;
        geometry_msgs::msg::Pose lastPose = (*(_markers.end() - 1)).pose;
        tf2::fromMsg(lastPose.orientation, qLast);
        tf2::Quaternion qLatest;
        tf2::fromMsg(_latest.pose.pose.orientation, qLatest);

        tf2::Vector3 direction(1, 0, 0);
        tf2::Vector3 lastVector = tf2::quatRotate(qLast, direction);
        tf2::Vector3 latestVector = tf2::quatRotate(qLatest, direction);

        tf2::Vector3 lastPosition(lastPose.position.x, lastPose.position.y, lastPose.position.z);
        tf2::Vector3 latestPosition(
            _latest.pose.pose.position.x, _latest.pose.pose.position.y, _latest.pose.pose.position.z);

        tf2::Vector3 lastEndPos = lastPosition + lastVector * _arrowLength;
        tf2::Vector3 latestEndPos = latestPosition + latestVector * _arrowLength;

        double distance = lastEndPos.distance(latestEndPos);
        RCLCPP_DEBUG(
            this->get_logger(),
            "Info: distance between the latest and the last odometry is %f",
            distance);
        if(distance >= _distSensitivity)
            return true;
        return false;
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

        visualization_msgs::msg::Marker cumtrail = this->generateMarker(*msg);
        _curPublisher->publish(cumtrail);

        _callbackOccured = false;
    }

    visualization_msgs::msg::Marker OdometryTranslator::generateMarker(const nav_msgs::msg::Odometry msg)
    {
        visualization_msgs::msg::Marker cumtrail;
        cumtrail.pose = _latest.pose.pose;
        cumtrail.header = _latest.header;
        cumtrail.ns = "odometry_path";
        cumtrail.id = 0;
        cumtrail.type = visualization_msgs::msg::Marker::ARROW;
        cumtrail.action = visualization_msgs::msg::Marker::ADD;
        cumtrail.lifetime = rclcpp::Duration::from_seconds(0);
        cumtrail.scale.x = _arrowLength;
        cumtrail.scale.y = 0.005;
        cumtrail.scale.z = 0.005;
        cumtrail.color.r = 255;
        cumtrail.color.g = 0;
        cumtrail.color.b = 255;
        cumtrail.color.a = 1.0;

        return cumtrail;
    }
}
