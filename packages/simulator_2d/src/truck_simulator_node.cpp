#include "simulator_2d/truck_simulator_node.h"

#include "simulator2d/status_code.h"

#include "geom/msg.h"

#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

using namespace std::placeholders;

TruckSimulatorNode::TruckSimulatorNode() : simulator2d::SimulatorNode() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        get_parameter("qos").as_int());

    slots_.control = create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&TruckSimulatorNode::handleControl, this, _1));

    signals_.telemetry = create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.state = create_publisher<truck_msgs::msg::SimulationState>(
        "/simulator/state", rclcpp::QoS(1).reliability(qos));
}

void TruckSimulatorNode::handleControl(
    const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        setControl(control->velocity, control->curvature, control->acceleration);
    } else {
        setControl(control->velocity, control->curvature);
    }
}

void TruckSimulatorNode::publishTelemetryMessage(
    const simulator2d::TruckState& truck_state) {
    truck_msgs::msg::HardwareTelemetry msg;
    msg.header.frame_id = "base";
    msg.header.stamp = truck_state.time();

    const auto current_steering = truck_state.currentSteering();
    msg.current_left_steering  = current_steering.left.radians();
    msg.current_right_steering = current_steering.right.radians();

    const auto target_steering = truck_state.targetSteering();
    msg.target_left_steering  = target_steering.left.radians();
    msg.target_right_steering = target_steering.right.radians();

    msg.current_rps = truck_state.currentMotorRps();
    msg.target_rps  = truck_state.targetMotorRps();

    const auto wv = truck_state.wheelVelocity();
    msg.rear_left_wheel_velocity   = wv.rear_left.radians();
    msg.rear_right_wheel_velocity  = wv.rear_right.radians();
    msg.front_left_wheel_velocity  = wv.front_left.radians();
    msg.front_right_wheel_velocity = wv.front_right.radians();

    // battery ??, оставляем 0.0
    signals_.telemetry->publish(msg);
}

void TruckSimulatorNode::publishSimulationStateMessage(
    const simulator2d::TruckState& truck_state) {
    truck_msgs::msg::SimulationState msg;
    msg.header.frame_id = "base";
    msg.header.stamp = truck_state.time();

    msg.speed    = truck_state.baseTwist().velocity;
    msg.steering = truck_state.currentSteering().middle.radians();
    msg.collision = truck_state.status() == simulator2d::StatusCode::COLLISION;
    msg.pose     = geom::msg::toPose(truck_state.odomBasePose());

    const auto angular_velocity = truck_state.gyroAngularVelocity();
    msg.gyro_angular_velocity.x = angular_velocity.x;
    msg.gyro_angular_velocity.y = angular_velocity.y;
    msg.gyro_angular_velocity.z = angular_velocity.z;

    const auto acceleration = truck_state.accelLinearAcceleration();
    msg.accel_linear_acceleration.x = acceleration.x;
    msg.accel_linear_acceleration.y = acceleration.y;
    msg.accel_linear_acceleration.z = acceleration.z;

    signals_.state->publish(msg);
}

void TruckSimulatorNode::onSimulationTick(
    const simulator2d::TruckState& truck_state) {
    publishTelemetryMessage(truck_state);
    publishSimulationStateMessage(truck_state);
}

}  // namespace truck::simulator