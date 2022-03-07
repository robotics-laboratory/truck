#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>


//this node deal with two topics:
// /camera/color/camera_info [sensor_msgs/msg/CameraInfo]
// /camera/color/image_raw [sensor_msgs/msg/Image]

const static std::string kImageRawTopic = "/camera/color/image_raw";
const static std::string kCameraInfoTopic = "/camera/color/camera_info";
const static std::string kArucoLocalizationNodeName = "aruco_localization";

class ArucoLocalization : public rclcpp::Node
{
public:
  ArucoLocalization();

private:
  void HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_raw_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info_;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  std::shared_ptr<cv::Mat> camera_matrix, dist_coeffs;
};