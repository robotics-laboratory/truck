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

// https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
enum CmdId : uint32_t {
    HEARTBEAT = 0x001,               // ControllerStatus  - publisher
    GET_MOTOR_ERROR = 0x003,         // SystemStatus      - publisher
    GET_ENCODER_ERROR = 0x004,       // SystemStatus      - publisher
    GET_SENSORLESS_ERROR = 0x005,    // SystemStatus      - publisher
    GET_CONTROLLER_ERROR = 0x1d,     // SystemStatus      - publisher
    SET_AXIS_STATE = 0x007,          // SetAxisState      - service
    GET_ENCODER_ESTIMATES = 0x009,   // ControllerStatus  - publisher
    SET_CONTROLLER_MODE = 0x00b,     // ControlMessage    - subscriber
    SET_INPUT_POS = 0x00C,           // ControlMessage    - subscriber
    SET_INPUT_VEL = 0x00D,           // ControlMessage    - subscriber
    SET_INPUT_TORQUE = 0x00E,        // ControlMessage    - subscriber
    GET_IQ = 0x014,                  // ControllerStatus  - publisher
    GET_BUS_VOLTAGE_CURRENT = 0x017, // SystemStatus      - publisher
    CLEAR_ERRORS = 0x018,            // SystemStatus      - publisher
    GET_TORQUES = 0x01c,             // ControllerStatus  - publisher
};

enum AxisState : uint8_t {
    AXIS_STATE_UNDEFINED,
    AXIS_STATE_IDLE,
    AXIS_STATE_STARTUP_SEQUENCE,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_LOCKIN_SPIN,
    AXIS_STATE_ENCODER_DIR_FIND,
    AXIS_STATE_HOMING,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION,
};

class HardwareNode : public rclcpp::Node {
  public:
    using CanFrameTimePair = std::pair<can_frame, std::chrono::system_clock::time_point>;
    HardwareNode();

  private:
    uint8_t curMode = ControlMode::OFF;
    std::unordered_map<CmdId, CanFrameTimePair> canFramesCache;
    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<model::Steering> steering_ = nullptr;

    void initializePtrFields();
    void initializeParams();
    void initializeTopicHandlers();
    void initializeTimers();
    void initializeOdrive();
    void initializeSocketCan();
    void initializeTeensy();
    void readFromSocket();
    void enableMotor();
    void disableMotor();
    void pushStatus();
    void pushTelemetry();
    void modeCallback(const ControlMode& msg);
    void commandCallback(const Control& msg);
    void sendFrame(uint32_t cmdId, uint8_t can_dlc, const void* data);

    std::optional<can_frame> checkCanFrame(const CmdId& command, const double reportRate);
    
    struct SocketCan {
        int socket;
        struct ifreq ifr;
        struct sockaddr_can addr;
    } can_{};

    struct Params {
        uint16_t odriveCanId;
        uint32_t teensySerialSpeed;
        std::string odriveAxis;
        std::string interface;
        std::string teensySerialPort;
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
