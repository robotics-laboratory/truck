#include <chrono>
#include <memory>

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
      cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16); // size and format from rs-viewer
      pipe.start(cfg);
      publisher_img_ = this->create_publisher<sensor_msgs::msg::Image>("imgtopic", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&ImagePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      rs2::frameset data = pipe.wait_for_frames();
      rs2::depth_frame depth = data.get_depth_frame();
      cv::Mat img(cv::Size(depth.get_width(), depth.get_height()), CV_16U, (void*)depth.get_data());
      sensor_msgs::msg::Image about;
      about.header.stamp = rclcpp::Node::now();
      std::string frame = "depth_frame";
      about.header.frame_id = frame.c_str();
      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(about.header, "mono16", img).toImageMsg();
      publisher_img_->publish(*msg.get());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_img_;
    rs2::config cfg;
    rs2::pipeline pipe;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

