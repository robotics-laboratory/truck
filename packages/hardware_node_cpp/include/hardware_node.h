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

using namespace truck_msgs::msg;

namespace truck::hardware_node {

enum CmdId : uint32_t {
    kHeartbeat = 0x001,             // ControllerStatus  - publisher
    kGetMotorError = 0x003,         // SystemStatus      - publisher
    kGetEncoderError = 0x004,       // SystemStatus      - publisher
    kGetSensorlessError = 0x005,    // SystemStatus      - publisher
    kGetControllerError = 0x1d,     // SystemStatus      - publisher
    kSetAxisState = 0x007,          // SetAxisState      - service
    kGetEncoderEstimates = 0x009,   // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,     // ControlMessage    - subscriber
    kSetInputPos,                   // ControlMessage    - subscriber
    kSetInputVel,                   // ControlMessage    - subscriber
    kSetInputTorque,                // ControlMessage    - subscriber
    kGetIq = 0x014,                 // ControllerStatus  - publisher
    kGetTemp,                       // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017,  // SystemStatus      - publisher
    kClearErrors = 0x018,           // SystemStatus      - publisher
    kGetTorques = 0x01c,            // ControllerStatus  - publisher
};

class HardwareNode : public rclcpp::Node {
  public:
    HardwareNode();

  private:
    uint8_t curMode = ControlMode::OFF;

    std::unordered_map<CmdId, std::pair<can_frame, std::chrono::system_clock::time_point>>
        canFramesCache;

    std::unique_ptr<model::Model> model_ = nullptr;

    void initializePtrFields();

    void initializeParams();

    void initializeTopicHandlers();

    void initializeTimers();

    void initializeOdrive();

    void initializeSocketCan();

    void modeCallback(const ControlMode& msg);

    void commandCallback(const Control& msg);

    void readFromSocket();

    void sendFrame(uint32_t cmdId, uint8_t can_dlc, const void* data);

    void enableMotor();

    void disableMotor();

    void pushStatus();

    void pushTelemetry();

    struct SocketCan {
        int socket;
        const char* device = "vxcan1";
        struct ifreq ifr;
        struct sockaddr_can addr;
    } can_{};

    struct Params {
        uint16_t nodeId;
        std::string odriveAxis;
        std::string interface;
        std::chrono::milliseconds odriveTimeout;
        double statusReportRate;
        double telemetryReportRate;
        double readReportRate;
    } params_{};

    struct Slots {
        rclcpp::Subscription<ControlMode>::SharedPtr mode = nullptr;
        rclcpp::Subscription<Control>::SharedPtr command = nullptr;
    } slots_{};

    struct Signals {
        rclcpp::Publisher<HardwareTelemetry>::SharedPtr telemetry = nullptr;
        rclcpp::Publisher<HardwareStatus>::SharedPtr status = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } signals_{};

    struct Timers {
        rclcpp::TimerBase::SharedPtr statusTimer = nullptr;
        rclcpp::TimerBase::SharedPtr telemetryTimer = nullptr;
        rclcpp::TimerBase::SharedPtr socketRead = nullptr;
    } timers_{};
};
}  // namespace truck::hardware_node
