#include "aruco_localization.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <opencv2/calib3d.hpp>
#include <thread>

#include "math_helpers.hpp"
#include "msgs_helpers.hpp"
#include "visualization_helpers.hpp"

using std::placeholders::_1;

const static std::string IMAGE_RAW_TOPIC = "/color/image_raw";
const static std::string CAMERA_INFO_TOPIC = "/color/camera_info";
const static std::string ARUCO_LOCALIZATION_NODE_NAME = "aruco_localization";
const static std::string ARUCO_ODOMETRY_TOPIC = "/truck/aruco/odometry";
const static std::string ARUCO_POSE_TOPIC = "/truck/aruco/pose";
const static std::string ARUCO_MARKERS_TOPIC = "/truck/aruco/vis/markers";
const static std::string ARUCO_TRANSFORMS_TOPIC = "/truck/aruco/vis/transforms";

const static std::string CAMERA_FRAME_ID = "camera";

const static int CAMERA_MATRIX_SIZE = 3;
const static int DIST_COEFFS_COUNT = 5;
const static int CV_64_F_SIZE = 8;
const static int MARKER_COUNT = 250;

namespace rosaruco {

ArucoLocalization::ArucoLocalization() :
    rclcpp::Node(ARUCO_LOCALIZATION_NODE_NAME),
    camera_matrix_(CAMERA_MATRIX_SIZE, CAMERA_MATRIX_SIZE, CV_64F),
    dist_coeffs_(1, DIST_COEFFS_COUNT, CV_64F),
    detector_parameters_(cv::makePtr<cv::aruco::DetectorParameters>()),
    marker_dictionary_(cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))),
    coordinator_(MARKER_COUNT) {
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.durability_volatile();

    subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_RAW_TOPIC, qos, std::bind(&ArucoLocalization::handleImage, this, _1));
    subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        CAMERA_INFO_TOPIC, qos, std::bind(&ArucoLocalization::updateCameraInfo, this, _1));
    publisher_odometry_ =
        this->create_publisher<nav_msgs::msg::Odometry>(ARUCO_ODOMETRY_TOPIC, qos);
    publisher_pose_stamped_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(ARUCO_POSE_TOPIC, qos);
    publisher_marker_array_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(ARUCO_MARKERS_TOPIC, qos);
    publisher_tf2_transform_ =
        this->create_publisher<tf2_msgs::msg::TFMessage>(ARUCO_TRANSFORMS_TOPIC, qos);
}

void ArucoLocalization::handleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "handleImage got message");
    auto cv_image = cv_bridge::toCvShare(msg);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;

    cv::aruco::detectMarkers(
        cv_image->image,
        marker_dictionary_,
        marker_corners,
        marker_ids,
        detector_parameters_,
        rejected_candidates);

    RCLCPP_INFO(this->get_logger(), "handleImage detected %ld markers", marker_ids.size());

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    cv::aruco::estimatePoseSingleMarkers(
        marker_corners, 1, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    std::vector<Transform> from_cam_to_marker;
    from_cam_to_marker.reserve(rvecs.size());

    for (size_t i = 0; i < rvecs.size(); i++) {
        from_cam_to_marker.push_back(getTransform(rvecs[i], tvecs[i]));
    }

    coordinator_.update(marker_ids, from_cam_to_marker);

    auto pose = coordinator_.getPose();

    geometry_msgs::msg::Pose pose_msg;

    setPoint(pose.point, pose_msg.position);

    auto orientation_msg = tf2::toMsg(pose.orientation);

    pose_msg.orientation = orientation_msg;

    if (publisher_pose_stamped_->get_subscription_count()) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose_msg;
        pose_stamped.header.stamp = msg->header.stamp;
        publisher_pose_stamped_->publish(pose_stamped);
    }

    if (publisher_tf2_transform_->get_subscription_count()) {
        geometry_msgs::msg::TransformStamped transform_msg;

        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = "";
        transform_msg.child_frame_id = CAMERA_FRAME_ID;

        geometry_msgs::msg::Vector3 translation_msg;
        setPoint(pose.point, translation_msg);

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
        visualization_msgs::msg::MarkerArray marker_array;

        const std::unordered_set<int> visible_markers(marker_ids.begin(), marker_ids.end());

        for (size_t i = 0; i < MARKER_COUNT; i++) {
            auto to_anchor = coordinator_.getTransformToAnchor(i);
            if (to_anchor) {
                addLabeledMarker(
                    marker_array.markers, *to_anchor, i, 1.0, visible_markers.count(i));
            }
        }

        for (auto& marker : marker_array.markers) {
            marker.header.stamp = msg->header.stamp;
        }

        publisher_marker_array_->publish(marker_array);
    }
}

void ArucoLocalization::updateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "updateCameraInfo got message");

    static_assert(
        std::tuple_size<decltype(msg->k)>::value == CAMERA_MATRIX_SIZE * CAMERA_MATRIX_SIZE);
    static_assert(sizeof(decltype(msg->k)::value_type) == CV_64_F_SIZE);
    std::memcpy(
        camera_matrix_.data, msg->k.data(), CAMERA_MATRIX_SIZE * CAMERA_MATRIX_SIZE * CV_64_F_SIZE);

    static_assert(sizeof(decltype(msg->d)::value_type) == CV_64_F_SIZE);
    assert(msg->d.size() == DIST_COEFFS_COUNT);
    std::memcpy(dist_coeffs_.data, msg->d.data(), DIST_COEFFS_COUNT * CV_64_F_SIZE);
}

}  // namespace rosaruco

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rosaruco::ArucoLocalization>());
    rclcpp::shutdown();
    return 0;
}  // namespace rosaruco
