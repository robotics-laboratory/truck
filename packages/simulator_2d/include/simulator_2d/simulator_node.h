#include <visualization_msgs/msg/marker.hpp>
#include "truck_msgs/msg/control.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

using namespace std::chrono_literals;

class SimulatorNode : public rclcpp::Node {
    public:
        SimulatorNode();
        void timerCallback();

    private:
        std::chrono::duration<double> period_ = 250ms;
        visualization_msgs::msg::Marker sphere_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Signals {
            rclcpp::Publisher<truck_msgs::msg::Control>::SharedPtr command = nullptr;
            rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback = nullptr;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization = nullptr;
        } signal_;
};