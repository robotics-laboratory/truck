#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>

#include <functional>
#include <memory>
#include <string>

class CameraView: public rclcpp::Node {
public:
    const std::string camera_topic = "/camera/color/image_raw";
    const std::string camera_view_topic = "/camera/color/image_view";

    CameraView() : Node("CameraView") {
        const auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data);

        slot_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, qos,
            std::bind(&CameraView::Resize, this, std::placeholders::_1));

        signal_camera_view_ =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(camera_view_topic, qos);
    }

private:
    void Resize(sensor_msgs::msg::Image::ConstSharedPtr msg) const {
        auto cv_image = cv_bridge::toCvShare(msg);
        cv_bridge::CvImage cv_resized{cv_image->header, cv_image->encoding};

        const cv::Size size = cv_image->image.size();

        const int width = 320;
        const int height = width * size.height / size.width;

        cv::resize(cv_image->image, cv_resized.image, {width, height}, 0, 0, cv::INTER_NEAREST);

        auto result = cv_resized.toCompressedImageMsg();
        signal_camera_view_->publish(*result);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr slot_camera_{};
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr signal_camera_view_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraView>());
    rclcpp::shutdown();
    return 0;
}