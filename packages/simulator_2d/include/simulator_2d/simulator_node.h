#include "truck_msgs/msg/control.hpp"

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
        void handleControl(truck_msgs::msg::Control::ConstSharedPtr control);
        void createTruckMarker();
        void publishTruckMarker(geom::Pose pose, geom::Angle steering);
        void createOdometryMessage();
        void publishOdometryMessage(geom::Pose pose, geom::Angle steering);
        void publishSignals();

        SimulatorEngine *engine_ = nullptr;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Parameters {
            double ego_height = 0.0;
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