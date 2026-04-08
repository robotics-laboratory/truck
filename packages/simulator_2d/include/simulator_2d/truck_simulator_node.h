#pragma once

#include "simulator2d/simulator_node.h"

#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"
#include "truck_msgs/msg/simulation_state.hpp"

#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

class TruckSimulatorNode : public simulator2d::SimulatorNode {
  public:
    TruckSimulatorNode();

  private:
    // Конвертация truck_msgs::Control -> setControl()
    void handleControl(const truck_msgs::msg::Control::ConstSharedPtr control);

    // Публикация truck-специфичных топиков
    void publishTelemetryMessage(const simulator2d::TruckState& truck_state);
    void publishSimulationStateMessage(const simulator2d::TruckState& truck_state);

    // Переопределяем хук из SimulatorNode - вызывается каждый тик
    void onSimulationTick(const simulator2d::TruckState& truck_state) override;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::HardwareTelemetry>::SharedPtr telemetry = nullptr;
        rclcpp::Publisher<truck_msgs::msg::SimulationState>::SharedPtr state = nullptr;
    } signals_;
};

}  // namespace truck::simulator