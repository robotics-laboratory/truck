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
        std::chrono::duration<double> period_ = 250ms, simulation_tick_ = 10ms;
        visualization_msgs::msg::Marker sphere_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct State {
            truck_msgs::msg::Control::SharedPtr control = nullptr;
            nav_msgs::msg::Odometry::ConstShared odom = nullptr;
        } state_;

        struct Slots {
            rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        } slot_;

        struct Signals {
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
            //rclcpp::Subscription<rclcpp::Clock>::SharedPtr clock = nullptr;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization = nullptr;
        } signal_;
};