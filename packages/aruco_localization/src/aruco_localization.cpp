#include "aruco_localization.hpp"

#include <chrono>
#include <thread>

using std::placeholders::_1;

ArucoLocalization::ArucoLocalization() : rclcpp::Node(kArucoLocalizationNodeName) {          
  subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
    kImageRawTopic, 10, std::bind(&ArucoLocalization::HandleImage, this, _1)
  );
  subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, 10, std::bind(&ArucoLocalization::UpdateCameraInfo, this, _1)
  );
}

void ArucoLocalization::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "HandleImage got message");
  auto cv_image = cv_bridge::toCvShare(msg);
  cv::aruco::detectMarkers(cv_image->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  RCLCPP_INFO(this->get_logger(), "HandleImage found %d markers", markerIds.size());

  auto current_camera_info = camera_info_;
  
  // RCLCPP_INFO(this->get_logger(), "rows = %d, cols = %d, CV_IS_MAT() = %d", 
  //   dist_coeffs.rows,
  //   dist_coeffs.cols, );
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camera_matrix, dist_coeffs, rvecs, tvecs);
  
  // RCLCPP_INFO(this->get_logger(), "HandleImage falling asleep %d", msg->header.stamp.nanosec);
  // std::this_thread::sleep_for(std::chrono::seconds(5));
  // RCLCPP_INFO(this->get_logger(), "HandleImage woke up %d", msg->header.stamp.nanosec);

  static int ct = 0;
  ct++;
  if (ct % 10 == 0) {
    cv::Mat outputImage = cv_image->image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    cv::aruco::drawAxis(outputImage, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1);
    cv::imwrite("/ws/detected_markers/image_" + std::to_string(ct) + ".png", outputImage);
    cv_bridge::CvImage cv_image();
  }
}

void ArucoLocalization::UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo got message");
  cv::Mat new_camera_matrix(3, 3, CV_64F);
  auto camera_matrix_array = msg->k;
  std::memcpy(new_camera_matrix.data, msg->k.data(), sizeof(double) * 9);

  const cv::Mat camera_matrix(3, 3, CV_64F, msg->k.data());
  const cv::Mat dist_coeffs(1, 5, CV_64F, msg->d.data());
  // printf("%s\n", msg->distortion_model);
  // RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo falling asleep %d", msg->header.stamp.nanosec);
  // std::this_thread::sleep_for(std::chrono::seconds(5));
  // RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo woke up %d", msg->header.stamp.nanosec);
  // RCLCPP_INFO(this->get_logger(), "distortion model : %s", msg->distortion_model);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoLocalization>());
  rclcpp::shutdown();
  return 0;
}
