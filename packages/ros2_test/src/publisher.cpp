#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class CubePublisher : public rclcpp::Node
{
public:
    CubePublisher()
        : Node("cube_publisher"), count_(0)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CubePublisher::publish_marker, this));
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/cube/test", 10);
    }

private:
    void publish_marker()
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cube";
        marker.id = count_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        if (count_ % 2 == 0) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
           
            RCLCPP_INFO(this->get_logger(), "I'm a cube of color blue and I was published!");
        } else {
            marker.color.r = 1.0; 
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            RCLCPP_INFO(this->get_logger(), "I'm a cube of color red and I was published!");
        }

        marker.pose.position.x = 3.0; 
        marker.pose.position.y = 3.0; 
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;

        marker.scale.x = 6;
        marker.scale.y = 6;
        marker.scale.z = 6;

        publisher_->publish(marker);
        count_++;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubePublisher>());
    rclcpp::shutdown();
    return 0;
}