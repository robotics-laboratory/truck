#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node {
    public:
        DemoNode();
        void timerCallback();

    private:
        std::chrono::duration<double> period_ = 250ms;
        visualization_msgs::msg::Marker sphere_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_ = nullptr;
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};