#pragma once

#include "model/model.h"

#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/hardware_status.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <utility>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <memory>
#include <string>
#include <cstring>

namespace truck::hardware_node {

class HardwareNode : public rclcpp::Node {
  public:
    HardwareNode();

  private:
    uint8_t prevMode_ = truck_msgs::msg::ControlMode::OFF;

    std::unique_ptr<model::Model> model_ = nullptr;

    void initializePtrFields();

    void initializeParams();

    void initializeTopicHandlers();

    void initializeTimers();

    void initializeOdrive();

    void initializeSocketCan();

    void modeCallback(const truck_msgs::msg::ControlMode& msg);

    void commandCallback(const truck_msgs::msg::Control& msg);

    void sendFrame(uint32_t cmdId, uint8_t can_dlc, const void* data);

    can_frame readFrame();

    void enableMotor();

    void disableMotor();

    void pushStatus();

    void pushTelemetry();

    struct SocketCan {
        int socket;
        struct ifreq ifr;
        struct sockaddr_can addr;
    } can_{};

    struct Params {
        uint16_t nodeId;
        std::string odriveAxis;
        std::string interface;
        std::chrono::milliseconds odriveTimeout;
        std::chrono::duration<double> statusReportRate;
        std::chrono::duration<double> telemetryReportRate;
    } params_{};

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::ControlMode>::SharedPtr mode = nullptr;
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr command = nullptr;
    } slots_{};

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::HardwareTelemetry>::SharedPtr telemetry = nullptr;
        rclcpp::Publisher<truck_msgs::msg::HardwareStatus>::SharedPtr status = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } signals_{};

    struct Timers {
        rclcpp::TimerBase::SharedPtr statusTimer = nullptr;
        rclcpp::TimerBase::SharedPtr telemetryTimer = nullptr;
    } timers_{};
};
}  // namespace truck::hardware_node
