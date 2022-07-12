#pragma once

<<<<<<< HEAD
#include <deque>
=======
#include <vector>
#include <thread>
>>>>>>> Added node for the Foxglove visualization of nav_msgs/Odometry messages

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

<<<<<<< HEAD
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
=======
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
>>>>>>> Added node for the Foxglove visualization of nav_msgs/Odometry messages
