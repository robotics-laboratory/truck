#include "aruco_localization.hpp"

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <opencv2/calib3d.hpp>
#include <thread>

#include "math_helpers.hpp"

using std::placeholders::_1;

const static std::string kImageRawTopic = "/camera/color/image_raw";
const static std::string kCameraInfoTopic = "/camera/color/camera_info";
const static std::string kArucoLocalizationNodeName = "aruco_localization";
const static std::string kArucoOdometryTopic = "/truck/aruco/odometry";
const static std::string kArucoPoseTopic = "/truck/aruco/pose";
const static std::string kArucoMarkersTopic = "/truck/aruco/vis/markers";
const static std::string kArucoTransformsTopic = "/truck/aruco/vis/transforms";

const static std::string kCameraFrameId = "camera";

const static int kCameraMatrixSize = 3;
const static int kDistCoeffsCount = 5;
const static int kCV_64FSize = 8;
const static int kMarkerCount = 250;

namespace robolab {
namespace aruco {

ArucoLocalization::ArucoLocalization()
    : rclcpp::Node(kArucoLocalizationNodeName),
      camera_matrix_(kCameraMatrixSize, kCameraMatrixSize, CV_64F),
      dist_coeffs_(1, kDistCoeffsCount, CV_64F),
      detector_parameters_(cv::aruco::DetectorParameters::create()),
      marker_dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
      coordinator_(kMarkerCount) {

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.durability_volatile();

    subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
        kImageRawTopic, qos, std::bind(&ArucoLocalization::HandleImage, this, _1));
    subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        kCameraInfoTopic, qos, std::bind(&ArucoLocalization::UpdateCameraInfo, this, _1));
    publisher_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(kArucoOdometryTopic, qos);
    publisher_pose_stamped_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(kArucoPoseTopic, qos);
    publisher_marker_array_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(kArucoMarkersTopic, qos);
    publisher_tf2_transform_ =
        this->create_publisher<tf2_msgs::msg::TFMessage>(kArucoTransformsTopic, qos);
}

void ArucoLocalization::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "HandleImage got message");
    auto cv_image = cv_bridge::toCvShare(msg);

    std::vector<int> marker_ids_;
    std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
    
    cv::aruco::detectMarkers(cv_image->image, marker_dictionary_, marker_corners_, marker_ids_,
                             detector_parameters_, rejected_candidates_);

    RCLCPP_INFO(this->get_logger(), "HandleImage detected %d markers", marker_ids_.size());

    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(marker_corners_, 1, camera_matrix_,
                                            dist_coeffs_, rvecs, tvecs);
    
    coordinator_.update(marker_ids_, rvecs, tvecs);

    auto pose = coordinator_.get_pose();

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = pose.point[0];
    pose_msg.position.y = pose.point[1];
    pose_msg.position.z = pose.point[2];

    /*
    added rotation around OY for PI/2 because opencv estimatePoseSingleMarkers() returns
    transformation where camera direction is OZ whereas default direction for odometry is OX

    added rotation around OY for PI/2 because Z axis must point up
    */

    auto orientation_msg = tf2::toMsg(pose.orientation);

    pose_msg.orientation = orientation_msg;

    if (publisher_pose_stamped_->get_subscription_count()) {
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.pose = pose_msg;
        pose_stamped_msg.header.stamp = msg->header.stamp;
        publisher_pose_stamped_->publish(pose_stamped_msg);
    }

    if (publisher_tf2_transform_->get_subscription_count()) {
        geometry_msgs::msg::TransformStamped transform_msg;

        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = "";
        transform_msg.child_frame_id = kCameraFrameId;

        geometry_msgs::msg::Vector3 translation_msg;
        translation_msg.x = pose.point[0];
        translation_msg.y = pose.point[1];
        translation_msg.z = pose.point[2];

        transform_msg.transform.rotation = orientation_msg;
        transform_msg.transform.translation = translation_msg;

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(transform_msg);

        publisher_tf2_transform_->publish(tf_msg);
    }

    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.pose.pose = pose_msg;
    odometry_msg.header.stamp = msg->header.stamp;
    publisher_odometry_->publish(odometry_msg);

    if (publisher_marker_array_->get_subscription_count()) {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.stamp = msg->header.stamp;
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

        visualization_msgs::msg::MarkerArray marker_array_msg;
        marker_array_msg.markers.push_back(marker_msg);
        publisher_marker_array_->publish(marker_array_msg);
    }
}

void ArucoLocalization::UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo got message");

    static_assert(std::tuple_size<decltype(msg->k)>::value ==
                  kCameraMatrixSize * kCameraMatrixSize);
    static_assert(sizeof(decltype(msg->k)::value_type) == kCV_64FSize);
    std::memcpy(camera_matrix_.data, msg->k.data(),
                kCameraMatrixSize * kCameraMatrixSize * kCV_64FSize);

    static_assert(sizeof(decltype(msg->d)::value_type) == kCV_64FSize);
    assert(msg->d.size() == kDistCoeffsCount);
    std::memcpy(dist_coeffs_.data, msg->d.data(), kDistCoeffsCount * kCV_64FSize);
}

}
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robolab::aruco::ArucoLocalization>());
    rclcpp::shutdown();
    return 0;
}
