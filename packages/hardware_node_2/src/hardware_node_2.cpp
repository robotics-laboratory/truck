#include "hardware_node_2.h"

#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/qos.hpp>

using namespace std::placeholders;

namespace truck::hardware_node {

enum CmdId : uint32_t {
    kHeartbeat = 0x001,             // ControllerStatus  - publisher
    kGetError = 0x003,              // SystemStatus      - publisher
    kSetAxisState = 0x007,          // SetAxisState      - service
    kGetEncoderEstimates = 0x009,   // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,     // ControlMessage    - subscriber
    kSetInputPos,                   // ControlMessage    - subscriber
    kSetInputVel,                   // ControlMessage    - subscriber
    kSetInputTorque,                // ControlMessage    - subscriber
    kGetIq = 0x014,                 // ControllerStatus  - publisher
    kGetTemp,                       // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017,  // SystemStatus      - publisher
    kGetTorques = 0x01c,            // ControllerStatus  - publisher
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

HardwareNode::HardwareNode() : Node("hardware_node") {
    initializePtrFields();
    initializeParams();
    initializeTopicHandlers();
    initializeTimers();
    initializeOdrive();
    RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::initializePtrFields() {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));
    RCLCPP_INFO(this->get_logger(), "curvature: %f", model_->baseMaxAbsCurvature());
}

void HardwareNode::initializeParams() {
    params_.nodeId = this->declare_parameter("node_id", 39);
    RCLCPP_INFO(this->get_logger(), "node_id: %d", params_.nodeId);
    params_.odriveAxis = this->declare_parameter("odrive_axis", "axis1");
    RCLCPP_INFO(this->get_logger(), "axis: %s", params_.odriveAxis.c_str());
    params_.interface = this->declare_parameter("interface", "vxcan1");
    RCLCPP_INFO(this->get_logger(), "interface: %s", params_.interface.c_str());
    params_.odriveTimeout =
        std::chrono::milliseconds(this->declare_parameter<long int>("odrive_timeout", 250));
    RCLCPP_INFO(this->get_logger(), "odrive timeout: %d", params_.odriveTimeout);
    params_.statusReportRate =
        std::chrono::duration<double>(this->declare_parameter("status_report_rate", 1.0));
    RCLCPP_INFO(this->get_logger(), "status report rate: %.2f", params_.statusReportRate.count());
    params_.telemetryReportRate =
        std::chrono::duration<double>(this->declare_parameter("telemetry_report_rate", 20.0));
    RCLCPP_INFO(
        this->get_logger(), "telemetry report rate: %.2f", params_.telemetryReportRate.count());
}

void HardwareNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    slots_.mode = Node::create_subscription<truck_msgs::msg::ControlMode>(
        "/control/mode",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&HardwareNode::modeCallback, this, _1));

    slots_.command = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&HardwareNode::commandCallback, this, _1));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.status = Node::create_publisher<truck_msgs::msg::HardwareStatus>(
        "/hardware/status", rclcpp::QoS(1).reliability(qos));

    signals_.odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/hardware/odom", rclcpp::QoS(1).reliability(qos));
}

void HardwareNode::initializeTimers() {
    timers_.statusTimer = this->create_wall_timer(
        params_.statusReportRate, std::bind(&HardwareNode::pushStatus, this));

    timers_.telemetryTimer = this->create_wall_timer(
        params_.telemetryReportRate, std::bind(&HardwareNode::pushTelemetry, this));
}

void HardwareNode::modeCallback(const truck_msgs::msg::ControlMode& msg) {
    if (msg.mode == truck_msgs::msg::ControlMode::OFF) {
        return;
    }
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF &&
        msg.mode != truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "mode change: OFF -> ANY - enabling motor");
        enableMotor();
        return;
    }
    if (prevMode_ != truck_msgs::msg::ControlMode::OFF &&
        msg.mode == truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "mode change: ANY -> OFF - disabling motor");
    }
    prevMode_ = msg.mode;
    timers_.statusTimer.reset();
    pushStatus();
}

void HardwareNode::send(uint32_t can_id, uint8_t can_dlc, const void* data = nullptr) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    if (data != nullptr) {
        frame.can_id = can_id;
        frame.can_dlc = can_dlc;
        memcpy(frame.data, data, can_dlc);
        std::ostringstream logMsg;
        logMsg << "sent packet: can_id = 0x" << std::hex << can_id << ", dlc = " << std::dec
               << (int)can_dlc << ", data = ";
        for (int i = 0; i < can_dlc; ++i) {
            logMsg << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", logMsg.str().c_str());
    }
    canInterface.sendCanFrame(frame);
}

void HardwareNode::initializeOdrive() {
    auto accelMps = model_->baseMaxAcceleration();
    auto accelRps = model_->linearVelocityToMotorRPS(accelMps);
    RCLCPP_INFO(this->get_logger(), "acceleration: %.1f m/s^2, %.1f turns/s^2", accelMps, accelRps);
    uint32_t id = (params_.nodeId << 5) | CmdId::kSetAxisState;
    send(id, sizeof(accelRps), &accelRps);
    disableMotor();
}

void HardwareNode::commandCallback(const truck_msgs::msg::Control& msg) {
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF) {
        disableMotor();
    }
    auto rpm = model_->linearVelocityToMotorRPS(msg.velocity);
    uint32_t id = (params_.nodeId << 5) | CmdId::kSetInputVel;
    send(id, 8, &rpm);
    truck::model::Twist twist = truck::model::Twist{msg.curvature, msg.velocity};
    twist = model_->baseToRearTwist(twist);
    auto steering = model_->rearTwistToSteering(twist);
    RCLCPP_INFO(this->get_logger(), "center curv: %.2f", msg.curvature);
    RCLCPP_INFO(this->get_logger(), "rear curv: %.2f", twist.curvature);
}

void HardwareNode::enableMotor() {
    double rpm = 0.0;
    uint32_t id1 = (params_.nodeId << 5) | CmdId::kSetInputVel;
    send(id1, sizeof(rpm), &rpm);

    uint8_t state = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    uint32_t id2 = (params_.nodeId << 5) | CmdId::kSetAxisState;
    send(id2, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "motor enabled");
}

void HardwareNode::disableMotor() {
    uint32_t id = (params_.nodeId << 5) | CmdId::kSetAxisState;
    uint8_t state = CmdId::kSetAxisState;
    send(id, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "motor disabled");
}

void HardwareNode::pushTelemetry() {
    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "base";
    auto telemetry = truck_msgs::msg::HardwareTelemetry();
    telemetry.header = header;
    auto st1 = CmdId::kGetEncoderEstimates;

    struct can_frame frame;
    canInterface.processReceivedFrame(frame);  // write(socket, &frame, sizeof(frame));

    /*
    std::ostringstream logMsg;
        logMsg << "recieved packet: can_id = 0x" << std::hex << frame.can_id << ", dlc = " <<
    std::dec << (int)frame.can_dlc << ", data = "; for (int i = 0; i < frame.can_dlc; ++i) { logMsg
    << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[i] << " ";
        }
    RCLCPP_INFO(this->get_logger(), "%s", logMsg.str().c_str());
    */

    /*
    telemetry = HardwareTelemetry(
        header=header,
        current_rps=self._axis.encoder.vel_estimate,
        target_rps=self._axis.controller.input_vel,

        battery_voltage=self._odrive.vbus_voltage, (params_nodeId << 5) | kGetBusVoltageCurrent;
        battery_current=self._odrive.ibus,
    )
    */
    signals_.telemetry->publish(telemetry);
}

void HardwareNode::pushStatus() {
    auto status = truck_msgs::msg::HardwareStatus();
    status.header.stamp = now();
    signals_.status->publish(status);
}

}  // namespace truck::hardware_node