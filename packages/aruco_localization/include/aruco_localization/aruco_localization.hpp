#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <nav_msgs/msg/odometry.hpp>


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

const static std::string kImageRawTopic = "/camera/color/image_raw";
const static std::string kCameraInfoTopic = "/camera/color/camera_info";
const static std::string kArucoLocalizationNodeName = "aruco_localization";
const static std::string kArucoOdometryTopic = "/truck/aruco_odometry";

const static int kCameraMatrixSize = 3;
const static int kDistCoeffsCount = 5;
const static int CV_64F_SIZE = 8;

class ArucoLocalization : public rclcpp::Node
{
public:
  ArucoLocalization();

private:
  void HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_raw_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry_;

  cv::Mat camera_matrix_, dist_coeffs_;

  std::vector<int> marker_ids_;
  std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Ptr<cv::aruco::Dictionary> marker_dictionary_;
};