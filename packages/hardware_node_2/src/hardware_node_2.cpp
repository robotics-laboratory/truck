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

enum AxisState {
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
    initializeOdrive();
    RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::initializePtrFields() {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));
    RCLCPP_INFO(this->get_logger(), "Curvature: %f", model_->baseMaxAbsCurvature());
}

void HardwareNode::initializeParams() {
    params_.nodeId = this->declare_parameter("node_id", 1);
    params_.modelConfig = this->declare_parameter("model_config", "");
    params_.odriveAxis = this->declare_parameter("odrive_axis", "axis1");
    params_.interface = this->declare_parameter("interface", "vxcan1");
    params_.odriveTimeout =
        std::chrono::milliseconds(this->declare_parameter<long int>("odrive_timeout", 250));
    params_.statusReportRate =
        std::chrono::duration<double>(this->declare_parameter("status_report_rate", 1.0));
    params_.telemetryReportRate =
        std::chrono::duration<double>(this->declare_parameter("telemetry_report_rate", 20.0));
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
        std::chrono::duration<double>(1 / params_.statusReportRate.count()),
        std::bind(&HardwareNode::pushStatus, this));
    timers_.telemetryTimer = this->create_wall_timer(
        std::chrono::duration<double>(1 / params_.telemetryReportRate.count()),
        std::bind(&HardwareNode::pushTelemetry, this));
}

void HardwareNode::modeCallback(const truck_msgs::msg::ControlMode& msg) {
    if (msg.mode == truck_msgs::msg::ControlMode::OFF) {
        return;
    }
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF &&
        msg.mode != truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: OFF -> ANY - Enabling motor");
        enableMotor();
        return;
    }
    if (prevMode_ != truck_msgs::msg::ControlMode::OFF &&
        msg.mode == truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: ANY -> OFF - Disabling motor");
    }
    prevMode_ = msg.mode;
    timers_.statusTimer.reset();
    pushStatus();
}

inline bool HardwareNode::verifyLength(const std::string& name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(this->get_logger(), "received %s", name.c_str());
    if (!valid)
        RCLCPP_WARN(
            this->get_logger(),
            "Incorrect %s frame length: %d != %d",
            name.c_str(),
            length,
            expected);
    return valid;
}

void HardwareNode::initializeOdrive() {
    auto accelMps = model_->baseMaxAcceleration();
    auto accelRps = model_->linearVelocityToMotorRPS(accelMps);
    RCLCPP_INFO(
        this->get_logger(), "Max acceleration: %.1f m/s^2 | %.1f turns/s^2", accelMps, accelRps);
    // self._axis.controller.config.vel_ramp_rate = accel_rps ?
    disableMotor();
}

void HardwareNode::commandCallback(const truck_msgs::msg::Control& msg) {
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF) {
        disableMotor();
    }
    auto rpm = model_->linearVelocityToMotorRPS(msg.velocity);
    // self._axis.controller.input_vel = rpm ?
    truck::model::Twist twist = truck::model::Twist{msg.curvature, msg.velocity};
    twist = model_->baseToRearTwist(twist);
    auto steering = model_->rearTwistToSteering(twist);
    RCLCPP_DEBUG(this->get_logger(), "Center curvature: %.2f", msg.curvature);
    RCLCPP_DEBUG(this->get_logger(), "Rear curvature: %.2f", twist.curvature);
}

void HardwareNode::enableMotor() {
    // self._axis.controller.input_vel = 0 ?
    struct can_frame frame;
    frame.can_id = (params_.nodeId << 5) | CmdId::kSetAxisState;
    frame.can_dlc = 4;
    frame.data[0] = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    canInterface.sendCanFrame(frame);
}

void HardwareNode::disableMotor() {
    struct can_frame frame;
    frame.can_id = (params_.nodeId << 5) | CmdId::kSetAxisState;
    frame.can_dlc = 4;
    frame.data[0] = AxisState::AXIS_STATE_IDLE;
    canInterface.sendCanFrame(frame);
}

void HardwareNode::pushTelemetry() {
    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "base";
    auto telemetry = truck_msgs::msg::HardwareTelemetry();
    telemetry.header = header;
    /*
    telemetry = HardwareTelemetry(
        header=header,
        current_rps=self._axis.encoder.vel_estimate,
        target_rps=self._axis.controller.input_vel, ?

        battery_voltage=self._odrive.vbus_voltage,
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