#include "simulator_2d/simulator_engine.h"

#include "geom/pose.h"
#include "geom/vector.h"

#include "truck_msgs/msg/control.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono> 

namespace truck::simulator {

class SimulatorNode : public rclcpp::Node {
    public:
        SimulatorNode();

    private:
        void handleControl(const truck_msgs::msg::Control::ConstSharedPtr control);
        void createOdometryMessage();
        void createTransformMessage();
        void publishOdometryMessage(const geom::Pose &pose, const geom::Vec2 &linearVelocity, 
            const geom::Vec2 &angularVelocity);
        void publishTransformMessage(const geom::Pose &pose);
        void publishSignals();

        SimulatorEngine engine_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Parameters {
            double ego_height;
            float ego_red;
            float ego_green;
            float ego_blue;
            std::chrono::duration<double> update_period;
        } params_;

        struct Slots {
            rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        } slots_;

        struct Messages {
            nav_msgs::msg::Odometry odometry;
            geometry_msgs::msg::TransformStamped odom_to_base_transform;
        } msgs_;

        struct Signals {
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
            rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher = nullptr;
        } signals_;
};

} // namespace truck::simulator