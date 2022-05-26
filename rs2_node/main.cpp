#include <chrono>
#include <memory>
#include <vector>

#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher()
    : Node("image_publisher"), count_(0)
    {
      cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16); // size and format from rs-viewer
      colorize_.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
      colorize_.set_option(RS2_OPTION_COLOR_SCHEME, 9.0f);
      pipe_.start(cfg_);
      publisher_img_ = this->create_publisher<sensor_msgs::msg::Image>("imgtopic", 10);
      publisher_compr_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("comprtopic", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&ImagePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      rs2::frameset data = pipe_.wait_for_frames();
      rs2::frame depth = data.get_depth_frame().apply_filter(colorize_);
      cv::Mat img(cv::Size(848, 480), CV_8UC3, (void*)depth.get_data()); // depth - CV_16U, colorized - CV_8UC3
      about_img_.header.stamp = rclcpp::Node::now();
      about_img_.header.frame_id = "depth_frame";
      sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(about_img_.header, "bgr8", img).toImageMsg(); // depth - mono16, colorized - bgr8
      about_compr_.header.stamp = rclcpp::Node::now();
      about_compr_.header.frame_id = "depth_frame_compressed";
      sensor_msgs::msg::CompressedImage::SharedPtr compr_msg = cv_bridge::CvImage(about_compr_.header, "bgr8", img).toCompressedImageMsg(cv_bridge::Format::PNG); // change here for JPEG/PNG compression
      std::vector<rclcpp::TopicEndpointInfo> img_sub = rclcpp::Node::get_subscriptions_info_by_topic("imgtopic");
      std::vector<rclcpp::TopicEndpointInfo> compr_sub = rclcpp::Node::get_subscriptions_info_by_topic("comprtopic");
      if (img_sub.size() > 0)
      {
        publisher_img_->publish(*img_msg.get());
      }
      if (compr_sub.size() > 0)
      {
        publisher_compr_->publish(*compr_msg.get());
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_img_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compr_;
    rs2::config cfg_;
    rs2::pipeline pipe_;
    size_t count_;
    sensor_msgs::msg::Image about_img_;
    sensor_msgs::msg::CompressedImage about_compr_;
    rs2::colorizer colorize_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

