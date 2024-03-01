#pragma once

#include "model/model.h"
#include "nav_msgs/msg/odometry.hpp"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/hardware_status.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"

#include <cstring>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <stack>
#include <string>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unordered_map>
#include <unistd.h>
#include <utility>


namespace truck::hardware_node {

class HardwareNode : public rclcpp::Node {
  public:
    HardwareNode();

  private:
    uint8_t prevMode_ = truck_msgs::msg::ControlMode::OFF;

    std::unordered_map<uint32_t, std::pair<can_frame, struct timeval>> framesCache;

    std::unique_ptr<model::Model> model_ = nullptr;

    void initializePtrFields();

    void initializeParams();

    void initializeTopicHandlers();

    void initializeTimers();

    void initializeOdrive();

    void initializeSocketCan();

    void modeCallback(const truck_msgs::msg::ControlMode& msg);

    void commandCallback(const truck_msgs::msg::Control& msg);

    void readFromSocket();

    void sendFrame(uint32_t cmdId, uint8_t can_dlc, const void* data);

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
        rclcpp::TimerBase::SharedPtr socketRead = nullptr;
    } timers_{};
};
}  // namespace truck::hardware_node
