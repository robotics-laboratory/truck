#include "model/model.h"
#include "truck_msgs/msg/control.hpp"

#include "geom/angle.h"
#include "geom/pose.h"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

class SimulatorNode : public rclcpp::Node {
    public:
        SimulatorNode();
        void timerCallback();

    private:
        void createTruckMarker();
        void updateTruckMarker();
        void updateState();

        void handleControl(truck_msgs::msg::Control::ConstSharedPtr control);

        std::unique_ptr<model::Model> model_ = nullptr;
        visualization_msgs::msg::Marker truck_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr, test_timer_ = nullptr;

        struct Parameters {
            double ego_height = 0.0;
            double simulation_tick = 0.01;
        } params_{};

        struct State {
            // The coordinates and the yaw.
            geom::Pose pose;
            // The position of the virtual corner in the middle (bicycle model).
            geom::Angle steering;
            //double velocity = 0.0;
        } state_;

        struct Control {
            double velocity = 0.0;
            double acceleration = 0.0;
            double curvature = 0.0;
        } control_;

        struct Slots {
            rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        } slot_;

        struct Signals {
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
            //rclcpp::Publisher<rclcpp::Clock>::SharedPtr clock = nullptr;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization = nullptr;
        } signal_;
};

} // namespace truck::simulator