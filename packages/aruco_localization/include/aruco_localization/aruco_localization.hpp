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
const static std::string kArucoOdometryTopic = "/truck/aruco/odometry";
const static std::string kArucoPoseTopic = "/truck/aruco/pose";
const static std::string kArucoMarkersTopic = "/truck/aruco/vis/markers";
const static std::string kArucoResizedImageTopic = "/truck/aruco/resized_image";

const static int kCameraMatrixSize = 3;
const static int kDistCoeffsCount = 5;
const static int CV_64F_SIZE = 8;

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

    cv::Mat camera_matrix_, dist_coeffs_;

    std::vector<int> marker_ids_;
    std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
    cv::Ptr<cv::aruco::Dictionary> marker_dictionary_;
};