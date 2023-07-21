#include "truck_msgs/msg/control.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

using namespace std::chrono_literals;

class SimulatorNode : public rclcpp::Node {
    public:
        SimulatorNode();
        void timerCallback();

    private:
        void createTruckMarker();
        void updateTruckMarker();

        void handleControl(truck_msgs::msg::Control::ConstSharedPtr control);
        void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom);

        std::chrono::duration<double> period_ = 250ms, simulation_tick_ = 10ms;
        visualization_msgs::msg::Marker truck_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct State {
            truck_msgs::msg::Control::ConstSharedPtr control = nullptr;
            nav_msgs::msg::Odometry::ConstSharedPtr odom = nullptr;
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