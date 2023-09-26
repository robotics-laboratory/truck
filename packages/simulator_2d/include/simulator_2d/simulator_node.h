#include "simulator_2d/simulator_engine.h"

#include "geom/pose.h"
#include "geom/vector.h"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"
#include "truck_msgs/msg/simulation_state.hpp"

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
    void publishOdometryMessage(
        const rclcpp::Time &time, const geom::Pose &pose, const geom::Vec2 &linearVelocity,
        const geom::Vec2 &angularVelocity);
    void publishTransformMessage(const rclcpp::Time &time, const geom::Pose &pose);
    void publishTelemetryMessage(const rclcpp::Time &time, const geom::Angle &steering);
    void publishSimulationStateMessage(const rclcpp::Time &time, 
        const double speed, const geom::Angle &steering);
    void publishSignals();

    SimulatorEngine engine_;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    struct Parameters {
        double update_period;
        double precision;
    } params_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher = nullptr;
        rclcpp::Publisher<truck_msgs::msg::HardwareTelemetry>::SharedPtr telemetry = nullptr;
        rclcpp::Publisher<truck_msgs::msg::SimulationState>::SharedPtr state = nullptr;
    } signals_;
};

}  // namespace truck::simulator
