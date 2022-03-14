#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "aruco_localization.hpp"

#include <chrono>
#include <thread>

using std::placeholders::_1;

ArucoLocalization::ArucoLocalization() 
: 
  rclcpp::Node(kArucoLocalizationNodeName),
  camera_matrix_(kCameraMatrixSize, kCameraMatrixSize, CV_64F),
  tmp_camera_matrix_(kCameraMatrixSize, kCameraMatrixSize, CV_64F),
  dist_coeffs_(1, kDistCoeffsCount, CV_64F),
  tmp_dist_coeffs_(1, kDistCoeffsCount, CV_64F),
  parameters_(cv::aruco::DetectorParameters::create()),
  dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)) {      

  subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
    kImageRawTopic, 10, std::bind(&ArucoLocalization::HandleImage, this, _1)
  );
  subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, 10, std::bind(&ArucoLocalization::UpdateCameraInfo, this, _1)
  );
  publisher_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
    kArucoOdometryTopic, 10
  );
}

void ArucoLocalization::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "HandleImage got message");
  auto cv_image = cv_bridge::toCvShare(msg);
  cv::aruco::detectMarkers(cv_image->image, dictionary_, marker_corners_, marker_ids_, parameters_, rejected_candidates_);
  RCLCPP_INFO(this->get_logger(), "HandleImage found %d markers", marker_ids_.size());

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(marker_corners_, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

  if (!rvecs.empty()) {
    auto message = nav_msgs::msg::Odometry();

    message.pose.pose.position.set__x(tvecs[0][0]);
    message.pose.pose.position.set__y(tvecs[0][1]);
    message.pose.pose.position.set__z(tvecs[0][2]);

    publisher_odometry_->publish(message);
  }
  
  // static int ct = 0;
  // ct++;
  // if (ct % 10 == 0) {
  //   cv::Mat outputImage = cv_image->image.clone();
  //   cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
  //   for (int i = 0; i < rvecs.size(); ++i) {
  //     auto rvec = rvecs[i];
  //     auto tvec = tvecs[i];
  //     cv::aruco::drawAxis(outputImage, camera_matrix, dist_coeffs, rvec, tvec, 0.1);
  //   }
  //   // cv::aruco::drawAxis(outputImage, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1);
  //   cv::imwrite("/ws/detected_markers/image_" + std::to_string(ct) + ".png", outputImage);
  //   cv_bridge::CvImage cv_image();
  // }
}

void ArucoLocalization::UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo got message");

  static_assert(std::tuple_size<decltype(msg->k)>::value == kCameraMatrixSize * kCameraMatrixSize);
  static_assert(std::is_same<double, decltype(msg->k)::value_type>::value);
  std::memcpy(tmp_camera_matrix_.data, msg->k.data(), kCameraMatrixSize * kCameraMatrixSize * sizeof(double));

  static_assert(std::is_same<double, decltype(msg->d)::value_type>::value);
  assert(msg->d.size() == kDistCoeffsCount);
  std::memcpy(tmp_dist_coeffs_.data, msg->d.data(), kDistCoeffsCount * sizeof(double));

  std::swap(camera_matrix_, tmp_camera_matrix_);
  std::swap(dist_coeffs_, tmp_dist_coeffs_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoLocalization>());
  rclcpp::shutdown();
  return 0;
}
