#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

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
    _outputTopic("odom_path")
    {
        _baseLink = this->declare_parameter("base_frame_id", _baseLink);
        _odomQoSType = this->declare_parameter("qos", _odomQoSType);
        _odomTopic = this->declare_parameter("odom_topic", _odomTopic);
        _outputTopic = this->declare_parameter("output_topic", _outputTopic);

        RCLCPP_INFO(this->get_logger(), "Input odometry topic: %s", _odomTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output odometry topic: %s", _outputTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", _baseLink.c_str());

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

        _pathPublisher = this->create_publisher<nav_msgs::msg::Path>(
            _outputTopic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(_odomQoSProfile))
            );

        startWarningThread();
    }

    OdometryTranslator::~OdometryTranslator()
    {
        if(_warningThread)
        {
            _callbackOccured = true;
            _warningThread->join();
            delete _warningThread;
        }
    }

    void OdometryTranslator::startWarningThread()
    {
        _warningThread = new std::thread(
            [&]()
            {
                rclcpp::Rate r(1.0/5.0);
                while(!_callbackOccured)
                {
                    r.sleep();
                    if (!_callbackOccured)
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Data from the topic /%s has not been received for more than 5 seconds."
                            " Check that the input topic is being published correctly.",
                            _odomTopic.c_str()
                            );
                }
            }
        );
    }

    void OdometryTranslator::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _callbackOccured = true;
        if (msg->pose.pose.orientation.w == 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Null odometry has been received at %f, aborting translation",
                msg->header.stamp.sec);
            return;
        } else
            RCLCPP_INFO(this->get_logger(), "Odometry received at %f", msg->header.stamp.sec);

        geometry_msgs::msg::PoseStamped nigger;
        nigger.pose = msg->pose.pose;
        nigger.header = msg->header;
        _poses.push_back(nigger);

        nav_msgs::msg::Path cumtrail;
        cumtrail.poses = _poses;
        cumtrail.header = msg->header;

        _pathPublisher->publish(cumtrail);
        RCLCPP_INFO(this->get_logger(), "Odometry message added to the path and republished at %f", this->now().seconds());
        _callbackOccured = false;
    }
}