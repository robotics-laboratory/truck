#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aruco_localization.hpp"
#include "math_helpers.hpp"

#include <chrono>
#include <thread>

using std::placeholders::_1;

ArucoLocalization::ArucoLocalization() 
: 
  rclcpp::Node(kArucoLocalizationNodeName),
  camera_matrix_(kCameraMatrixSize, kCameraMatrixSize, CV_64F),
  dist_coeffs_(1, kDistCoeffsCount, CV_64F),
  detector_parameters_(cv::aruco::DetectorParameters::create()),
  marker_dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)) {  

  rclcpp::QoS qos(1);
  qos.reliable();
  qos.durability_volatile();
  
  subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
    kImageRawTopic, qos, std::bind(&ArucoLocalization::HandleImage, this, _1)
  );
  subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, qos, std::bind(&ArucoLocalization::UpdateCameraInfo, this, _1)
  );
  publisher_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
    kArucoOdometryTopic, qos
  );
  publisher_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    kArucoPoseTopic, qos
  );
  publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    kArucoMarkersTopic, qos
  );
}

void ArucoLocalization::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "HandleImage got message");
  auto cv_image = cv_bridge::toCvShare(msg);
  cv::aruco::detectMarkers(cv_image->image, marker_dictionary_, marker_corners_, marker_ids_, detector_parameters_, rejected_candidates_);
  RCLCPP_INFO(this->get_logger(), "HandleImage found %d markers", marker_ids_.size());

  auto it = find(marker_ids_.begin(), marker_ids_.end(), 0);

  if (it != marker_ids_.end()) {
    size_t ind = it - marker_ids_.begin();
    
    std::vector<cv::Vec3d> rvecs, tvecs;

    std::vector<std::vector<cv::Point2f>> zero_id_marker_corners = {marker_corners_[ind]};
    cv::aruco::estimatePoseSingleMarkers(zero_id_marker_corners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    auto rot_quat = MathHelpers::RotationVectorToQuaternion(rvecs[0]);
    auto camera_position = MathHelpers::RotateUsingQuaternion(-tvecs[0], rot_quat.inverse());

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position
      .set__x(camera_position[0])
      .set__y(camera_position[1])
      .set__z(camera_position[2]);
    pose_msg.set__orientation(tf2::toMsg(rot_quat));

    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.set__pose(pose_msg);
    pose_stamped_msg.header.set__stamp(rclcpp::Clock().now());
    publisher_pose_stamped_->publish(pose_stamped_msg);

    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.pose.set__pose(pose_msg);
    odometry_msg.header.set__stamp(rclcpp::Clock().now());
    publisher_odometry_->publish(odometry_msg);

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.points = {
      geometry_msgs::msg::Point().set__x(-0.5).set__y(-0.5),
      geometry_msgs::msg::Point().set__x(-0.5).set__y(0.5),
      geometry_msgs::msg::Point().set__x(0.5).set__y(0.5),
      geometry_msgs::msg::Point().set__x(0.5).set__y(-0.5),
      geometry_msgs::msg::Point().set__x(-0.5).set__y(-0.5),
    };
    marker_msg.color.a = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.scale.x = 1.0;

    visualization_msgs::msg::MarkerArray marker_array_msg;
    marker_array_msg.markers.push_back(marker_msg);
    publisher_marker_array_->publish(marker_array_msg);
  }
}

void ArucoLocalization::UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo got message");

  static_assert(std::tuple_size<decltype(msg->k)>::value == kCameraMatrixSize * kCameraMatrixSize);
  static_assert(sizeof(decltype(msg->k)::value_type) == CV_64F_SIZE);
  std::memcpy(camera_matrix_.data, msg->k.data(), kCameraMatrixSize * kCameraMatrixSize * CV_64F_SIZE);

  static_assert(sizeof(decltype(msg->d)::value_type) == CV_64F_SIZE);
  assert(msg->d.size() == kDistCoeffsCount);
  std::memcpy(dist_coeffs_.data, msg->d.data(), kDistCoeffsCount * CV_64F_SIZE);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoLocalization>());
  rclcpp::shutdown();
  return 0;
}
