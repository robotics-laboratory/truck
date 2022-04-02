#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/calib3d.hpp>

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
  publisher_resized_image_ = this->create_publisher<sensor_msgs::msg::Image>(
    kArucoResizedImageTopic, qos
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
    cv::aruco::estimatePoseSingleMarkers(zero_id_marker_corners, 1, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    auto rot_quat = MathHelpers::RotationVectorToQuaternion(rvecs[0]);
    auto camera_position = MathHelpers::RotateUsingQuaternion(-tvecs[0], rot_quat.inverse());

    // (void)camera_position;

    // cv::Mat rot_mat(3, 3, CV_64F), rot_mat_t(3, 3, CV_64F);
    // cv::Rodrigues(rvecs[0], rot_mat);
    // cv::transpose(rot_mat, rot_mat_t);
    // cv::Mat true_camera_position = rot_mat_t * -tvecs[0];

    geometry_msgs::msg::Pose pose_msg;
    // pose_msg.position
    //   .set__x(true_camera_position.at<double>(0))
    //   .set__y(true_camera_position.at<double>(1))
    //   .set__z(true_camera_position.at<double>(2));
    
    pose_msg.position
      .set__x(camera_position[0])
      .set__y(camera_position[1])
      .set__z(camera_position[2]);
    pose_msg.set__orientation(tf2::toMsg(rot_quat.inverse() * tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI / 2)));

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
      geometry_msgs::msg::Point().set__x(0).set__y(0),
      geometry_msgs::msg::Point().set__x(0).set__y(1),
      geometry_msgs::msg::Point().set__x(1).set__y(1),
      geometry_msgs::msg::Point().set__x(1).set__y(0),
      geometry_msgs::msg::Point().set__x(0).set__y(0),
    };
    marker_msg.color.a = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.scale.x = 0.1;

    


    // visualization_msgs::msg::Marker marker_z_msg;
    // marker_z_msg.header.stamp = rclcpp::Clock().now();
    // marker_z_msg.id = 1;
    // marker_z_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // marker_z_msg.action = visualization_msgs::msg::Marker::ADD;
    // marker_z_msg.points = {
    //   geometry_msgs::msg::Point().set__x(0).set__y(0).set__z(0),
    //   geometry_msgs::msg::Point().set__x(0).set__y(0).set__z(1),
    // };
    // marker_z_msg.color.a = 1.0;
    // marker_z_msg.color.r = 1.0;
    // marker_z_msg.scale.x = 0.1;

    // cv::Mat cam_dir_cam_frame(3, 1, CV_64F);
    // cam_dir_cam_frame.at<double>(0) = 0;
    // cam_dir_cam_frame.at<double>(1) = 0;
    // cam_dir_cam_frame.at<double>(2) = 1;

    // cv::Mat cam_dir_end_marker_frame(3, 1, CV_64F);
    // // cam_dir_end_marker_frame = rot_mat_t * (cam_dir_cam_frame - tvecs[0]);
    // cv::Mat tmp = (cam_dir_cam_frame - tvecs[0]);
    // cam_dir_end_marker_frame = MathHelpers::RotateUsingQuaternion(
    //   cv::Vec3d(tmp.at<double>(0), tmp.at<double>(1), tmp.at<double>(2)),
    //   rot_quat.inverse());

    // cv::Mat cam_dir_marker_frame = (cam_dir_end_marker_frame - true_camera_position);

    // double t = -true_camera_position.at<double>(2) / cam_dir_marker_frame.at<double>(2);

    // RCLCPP_INFO(this->get_logger(), "HandleImage: t = %lf, z0 = %lf, zv = %lf", 
    //   t,
    //   -true_camera_position.at<double>(2),
    //   cam_dir_marker_frame.at<double>(2));

    // cv::Mat cam_dir_intersection_with_XY = true_camera_position + cam_dir_marker_frame * t;

    // visualization_msgs::msg::Marker marker_cam_dir_msg;
    // marker_cam_dir_msg.header.stamp = rclcpp::Clock().now();
    // marker_cam_dir_msg.id = 2;
    // marker_cam_dir_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // marker_cam_dir_msg.action = visualization_msgs::msg::Marker::ADD;
    // marker_cam_dir_msg.points = {
    //   geometry_msgs::msg::Point()
    //     .set__x(true_camera_position.at<double>(0))
    //     .set__y(true_camera_position.at<double>(1))
    //     .set__z(true_camera_position.at<double>(2)),
    //   geometry_msgs::msg::Point()
    //     .set__x(cam_dir_intersection_with_XY.at<double>(0))
    //     .set__y(cam_dir_intersection_with_XY.at<double>(1))
    //     .set__z(cam_dir_intersection_with_XY.at<double>(2)),
    // };
    // marker_cam_dir_msg.color.a = 1.0;
    // marker_cam_dir_msg.color.b = 1.0;
    // marker_cam_dir_msg.scale.x = 0.1;


    // rot_quat.inverse() * tf2::Quaternion(tf2::Vector3(0, 1, 0), PI / 2)
    // tf2::Transform from_marker_to_cam_frame(rot_quat, tf2::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]));
    // tf2::Transform from_marker_to_cam_frame(rot_quat, tf2::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]));
    // tf2::Transform from_cam_to_marker_frame = from_marker_to_cam_frame.inverse();

    visualization_msgs::msg::MarkerArray marker_array_msg;

    // for (int i = 0; i < 3; i++) {
    //     visualization_msgs::msg::Marker axis_msg;
    //     axis_msg.header.stamp = rclcpp::Clock().now();
    //     axis_msg.id = 10 + i;
    //     axis_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     axis_msg.color.set__a(1.0);

    //     if (i == 0) {
    //       axis_msg.color.set__g(1.0);
    //     } else if (i == 1) {
    //       axis_msg.color.set__b(1.0);
    //     } else {
    //       axis_msg.color.set__r(1.0);
    //     }
        
    //     // cam_frame_msg.colors = {
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__g(1.0),
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__b(1.0),
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__r(1.0),
    //     // };
    //     axis_msg.action = visualization_msgs::msg::Marker::ADD;
    //     axis_msg.scale.x = 0.1;

    //     geometry_msgs::msg::Point origin;
    //     origin.set__x(camera_position[0])
    //       .set__y(camera_position[1])
    //       .set__z(camera_position[2]);
    //     tf2::Vector3 vec;
    //     for (int j = 0; j < 3; j++) {
    //       vec[j] = 0;
    //     }
    //     vec[i] = 1;
    //     tf2::Vector3 tf_vec = from_cam_to_marker_frame(vec);
    //     geometry_msgs::msg::Point end_point;
    //     end_point.set__x(tf_vec[0])
    //       .set__y(tf_vec[1])
    //       .set__z(tf_vec[2]);
    //     axis_msg.points.push_back(origin);
    //     axis_msg.points.push_back(end_point);

    //     marker_array_msg.markers.push_back(axis_msg);
    // }
    
    // for (int i = 0; i < 3; i++) {
    //     visualization_msgs::msg::Marker axis_msg;
    //     axis_msg.header.stamp = rclcpp::Clock().now();
    //     axis_msg.id = 20 + i;
    //     axis_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     axis_msg.color.set__a(1.0);

    //     if (i == 0) {
    //       axis_msg.color.set__g(1.0);
    //     } else if (i == 1) {
    //       axis_msg.color.set__b(1.0);
    //     } else {
    //       axis_msg.color.set__r(1.0);
    //     }
        
    //     // cam_frame_msg.colors = {
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__g(1.0),
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__b(1.0),
    //     //   std_msgs::msg::ColorRGBA().set__a(1.0).set__r(1.0),
    //     // };
    //     axis_msg.action = visualization_msgs::msg::Marker::ADD;
    //     axis_msg.scale.x = 0.1;

    //     geometry_msgs::msg::Point origin;
    //     origin.set__x(0)
    //       .set__y(0)
    //       .set__z(0);
    //     tf2::Vector3 vec;
    //     for (int j = 0; j < 3; j++) {
    //       vec[j] = 0;
    //     }
    //     vec[i] = 2;
    //     geometry_msgs::msg::Point end_point;
    //     end_point.set__x(vec[0])
    //       .set__y(vec[1])
    //       .set__z(vec[2]);
    //     axis_msg.points.push_back(origin);
    //     axis_msg.points.push_back(end_point);
    //     marker_array_msg.markers.push_back(axis_msg);
    // }

    
    marker_array_msg.markers.push_back(marker_msg);
    // marker_array_msg.markers.push_back(cam_frame_msg);
    // marker_array_msg.markers.push_back(marker_cam_dir_msg);
    // marker_array_msg.markers.push_back(marker_z_msg);
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
