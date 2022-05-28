#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace truck
{
    class OdometryTranslator : public rclcpp::Node
    {
      public:
        explicit OdometryTranslator();
        ~OdometryTranslator() noexcept;

      private:
        void warningThread();
        void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
        visualization_msgs::msg::Marker generateMarker(const nav_msgs::msg::Odometry msg);
        bool checkOdomDiffSignificance();
        void republishOdometryPath();

        rmw_qos_profile_t _odomQoSProfile;
        nav_msgs::msg::Odometry _latest;
        std::vector<visualization_msgs::msg::Marker> _markers;
        bool _callbackOccured;
        int _count;
        
        rclcpp::TimerBase::SharedPtr _republishOdometryPath;
        rclcpp::TimerBase::SharedPtr _warningThread;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pathPublisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _curPublisher;

        //parameters
        std::string _baseLink;         
        std::string _odomFrameId;
        std::string _odomQoSType;
        std::string _odomTopic;
        std::string _pathTopic;
        std::string _curTopic;
        std::string _debug;
        std::string _showPath;
        int _arraySize;
        double _markerLongevity;
        double _secondRate;
        double _distSensitivity;
        double _arrowLength;
    };
}
