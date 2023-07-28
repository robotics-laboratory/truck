#include "simulator_2d/simulator_engine.h"

#include "truck_msgs/msg/control.hpp"

#include "geom/angle.h"
#include "geom/pose.h"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

namespace truck::simulator {

using namespace std::chrono_literals;

class SimulatorNode : public rclcpp::Node {
    public:
        SimulatorNode();
        ~SimulatorNode();

    private:
        void handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) const;
        void createTruckMarker();
        void publishTruckMarker(const geom::Pose pose, geom::Angle steering);
        void createOdometryMessage();
        void publishOdometryMessage(const geom::Pose pose, geom::Angle steering);
        void publishSignals();

        SimulatorEngine *engine_ = nullptr;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Parameters {
            double ego_height = 0.01;
            std::chrono::duration<double> update_period = 250ms;
        } params_;

        struct Slots {
            rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        } slots_;

        struct Messages {
            visualization_msgs::msg::Marker truck;
            nav_msgs::msg::Odometry odometry;
        } msgs_;

        struct Signals {
            //rclcpp::Publisher<rclcpp::Clock>::SharedPtr clock = nullptr;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization = nullptr;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        } signals_;
};

} // namespace truck::simulator