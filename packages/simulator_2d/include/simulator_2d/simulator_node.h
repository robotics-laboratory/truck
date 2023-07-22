#include "model/model.h"
#include "truck_msgs/msg/control.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

using namespace std::chrono_literals;

namespace truck::simulator {

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
        std::unique_ptr<model::Model> model_ = nullptr;
        visualization_msgs::msg::Marker truck_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Parameters {
            rclcpp::Duration ttl = rclcpp::Duration::from_seconds(1.0);

            double ego_z_lev = 0.0;
            double ego_height = 0.0;

            double ego_track_width = 0.06;
            double ego_track_height = 0.01;
            rclcpp::Duration ego_track_ttl = rclcpp::Duration::from_seconds(2.0);
            size_t ego_track_rate = 5;

            double arc_z_lev = 0.0;
            double arc_width = 0.0;
            double arc_length = 1.0;

            double waypoints_z_lev = 0.0;
            double waypoints_radius = 0.0;

            double trajectory_z_lev = 0.0;
            double trajectory_width = 0.0;
        } params_{};

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

} // namespace truck::simulator