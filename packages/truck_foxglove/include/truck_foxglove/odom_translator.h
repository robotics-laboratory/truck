#pragma once

#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <rclcpp_components/register_node_macro.hpp>

//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>

namespace truck
{
    class OdometryTranslator : public rclcpp::Node
    {
      public:
        explicit OdometryTranslator();
        ~OdometryTranslator() noexcept;

      private:
        void startWarningThread();
        void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

        rmw_qos_profile_t _odomQoSProfile;
        std::vector<geometry_msgs::msg::PoseStamped> _poses;
        std::thread* _warningThread;
        bool _callbackOccured;

        //std::shared_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;
        //std::shared_ptr<tf2_ros::TransformListener> _tfListener;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;

        //parameters
        std::string _baseLink;         
        std::string _odomFrameId;
        std::string _odomQoSType;
        std::string _odomTopic;
        std::string _outputTopic;
    };
}

//RCLCPP_COMPONENTS_REGISTER_NODE(truck::OdometryTranslator)