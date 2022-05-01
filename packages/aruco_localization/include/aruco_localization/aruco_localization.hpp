#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "coordinator.hpp"

/**
 * node gets messages from two topics:
 *
 * /camera/color/camera_info [sensor_msgs/msg/CameraInfo]
 * /camera/color/image_raw [sensor_msgs/msg/Image]
 *
 * and publishes first marker coordinates to third topic:
 *
 * /truck/aruco_odometry [nav_msgs/msg/Odometry]
 */

namespace robolab {
namespace aruco  {


class ArucoLocalization : public rclcpp::Node {
   public:
    ArucoLocalization();

   private:
    void HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_raw_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_stamped_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf2_transform_;

    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
    cv::Ptr<cv::aruco::Dictionary> marker_dictionary_;

    Coordinator coordinator_;
};

}
}