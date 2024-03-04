#include "hardware_node.h"

#include <chrono>
#include <fcntl.h>
#include <linux/sockios.h>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/qos.hpp>
#include <unistd.h>

using namespace std::placeholders;

using namespace std::chrono_literals;

using namespace truck_msgs::msg;

namespace truck::hardware_node {

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
    initializeSocketCan();
    initializeParams();
    initializeTopicHandlers();
    initializeTimers();
    initializeOdrive();
    RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::initializeSocketCan() {
    can_.socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_.ifr.ifr_name, can_.device);
    ioctl(can_.socket, SIOCGIFINDEX, &can_.ifr);
    can_.addr.can_family = AF_CAN;
    can_.addr.can_ifindex = can_.ifr.ifr_ifindex;
    bind(can_.socket, reinterpret_cast<struct sockaddr*>(&can_.addr), sizeof(can_.addr));
    int flags = fcntl(can_.socket, F_GETFL, 0);
    fcntl(can_.socket, F_SETFL, flags | O_NONBLOCK);
    int enable = 1;
    setsockopt(can_.socket, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));
}

void HardwareNode::initializeParams() {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    params_.nodeId = this->declare_parameter("node_id", 39);

    params_.odriveAxis = this->declare_parameter("odrive_axis", "axis1");

    params_.interface = this->declare_parameter("interface", "vxcan1");

    params_.odriveTimeout =
        std::chrono::milliseconds(this->declare_parameter<long int>("odrive_timeout", 250));

    params_.statusReportRate = this->declare_parameter("status_report_rate", 20.0);

    params_.telemetryReportRate = this->declare_parameter("telemetry_report_rate", 20.0);

    params_.readReportRate = this->declare_parameter("read_report_rate", 20.0);

    RCLCPP_INFO(this->get_logger(), "curvature: %f", model_->baseMaxAbsCurvature());
    RCLCPP_INFO(this->get_logger(), "node_id: %d", params_.nodeId);
    RCLCPP_INFO(this->get_logger(), "axis: %s", params_.odriveAxis.c_str());
    RCLCPP_INFO(this->get_logger(), "interface: %s", params_.interface.c_str());
    RCLCPP_INFO(this->get_logger(), "odrive timeout: %d", params_.odriveTimeout);
    RCLCPP_INFO(this->get_logger(), "status RR: %.2f", params_.statusReportRate);
    RCLCPP_INFO(this->get_logger(), "telemetry RR: %.2f", params_.telemetryReportRate);
    RCLCPP_INFO(this->get_logger(), "read from can socket RR: %.2f", params_.readReportRate);
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
    timers_.telemetryTimer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.telemetryReportRate)),
        std::bind(&HardwareNode::pushTelemetry, this));

    timers_.statusTimer = Node::create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.statusReportRate)),
        std::bind(&HardwareNode::pushStatus, this));

    timers_.socketRead = Node::create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.readReportRate)),
        std::bind(&HardwareNode::readFromSocket, this));
}

void HardwareNode::readFromSocket() {
    can_frame frame;
    struct iovec iov;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
    struct msghdr msg;

    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);

    msg.msg_name = nullptr;
    msg.msg_namelen = 0;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrlmsg;
    msg.msg_controllen = sizeof(ctrlmsg);
    msg.msg_flags = 0;

    for (;;) {
        ssize_t nbytes = recvmsg(can_.socket, &msg, 0);
        if (nbytes < 0) {
            break;
        }

        std::chrono::system_clock::time_point rcv_time;
        for (auto cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                struct timeval tv = *(struct timeval*)CMSG_DATA(cmsg);
                rcv_time = std::chrono::system_clock::from_time_t(tv.tv_sec) +
                           std::chrono::microseconds(tv.tv_usec);
                break;
            }
        }

        uint32_t cmd = frame.can_id & 0x1f;
        CmdId cmdId = static_cast<CmdId>(cmd);

        auto rcv_time_t = std::chrono::system_clock::to_time_t(rcv_time);
        auto rcv_time_us =
            std::chrono::duration_cast<std::chrono::microseconds>(rcv_time.time_since_epoch()) %
            1000000;

        canFramesCache[cmdId] = std::make_pair(frame, rcv_time);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&rcv_time_t), "%F %T") << "." << std::setw(6)
           << std::setfill('0') << rcv_time_us.count();

        RCLCPP_INFO(this->get_logger(), "Read cmd_id: %d. Time: %s", cmdId, ss.str().c_str());
    }
}
void HardwareNode::modeCallback(const ControlMode& msg) {
    if (msg.mode == curMode) {
        return;
    }
    if (curMode == ControlMode::OFF && msg.mode != ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: OFF -> ANY - enabling motor");
        enableMotor();
        return;
    }
    if (curMode != ControlMode::OFF && msg.mode == ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: ANY -> OFF - disabling motor");
        disableMotor();
    }
    curMode = msg.mode;
    timers_.statusTimer.reset();
    pushStatus();
}

void HardwareNode::sendFrame(uint32_t cmd_id, uint8_t can_dlc, const void* data = nullptr) {
    auto id = (params_.nodeId << 5) | cmd_id;
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = can_dlc;
    if (data != nullptr) {
        memcpy(frame.data, data, can_dlc);
    }
    ssize_t nbytes = write(can_.socket, &frame, sizeof(frame));
    if (nbytes < 0 || nbytes != sizeof(frame)) {
        rclcpp::shutdown();
    }
}

void HardwareNode::initializeOdrive() {
    auto accelMps = model_->baseMaxAcceleration();
    auto accelRps = model_->linearVelocityToMotorRPS(accelMps);
    RCLCPP_INFO(this->get_logger(), "Acceleration: %.1f m/s^2, %.1f turns/s^2", accelMps, accelRps);
    sendFrame(CmdId::kSetInputVel, sizeof(accelRps), &accelRps);
    disableMotor();
}

void HardwareNode::commandCallback(const Control& msg) {
    if (curMode == ControlMode::OFF) {
        disableMotor();
        return;
    }
    auto rpm = model_->linearVelocityToMotorRPS(msg.velocity);
    sendFrame(CmdId::kSetInputVel, sizeof(rpm), &rpm);
    truck::model::Twist twist = truck::model::Twist{msg.curvature, msg.velocity};
    twist = model_->baseToRearTwist(twist);
    auto steering = model_->rearTwistToSteering(twist);
    RCLCPP_INFO(this->get_logger(), "Center curv: %.2f", msg.curvature);
    RCLCPP_INFO(this->get_logger(), "Rear curv: %.2f", twist.curvature);
}

void HardwareNode::enableMotor() {
    double rpm = 0.0;
    sendFrame(CmdId::kSetInputVel, sizeof(rpm), &rpm);

    uint8_t state = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    sendFrame(CmdId::kSetAxisState, sizeof(state), &state);

    RCLCPP_INFO(this->get_logger(), "Motor enabled");
}

void HardwareNode::disableMotor() {
    uint8_t state = AxisState::AXIS_STATE_IDLE;
    sendFrame(CmdId::kSetAxisState, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "Motor disabled");
}

void HardwareNode::pushTelemetry() {
    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "base";

    auto telemetry = truck_msgs::msg::HardwareTelemetry();
    telemetry.header = header;
    auto voltageFrame = canFramesCache[CmdId::kGetBusVoltageCurrent];

    float voltage = 0.0, current = 0.0, rps = 0.0;

    memcpy(&voltage, voltageFrame.first.data, sizeof(voltage));
    memcpy(&current, voltageFrame.first.data + 4, sizeof(current));

    auto encoderFrame = canFramesCache[CmdId::kGetEncoderEstimates];
    memcpy(&rps, encoderFrame.first.data + 4, sizeof(rps));

    telemetry.battery_voltage = voltage;

    telemetry.battery_current = current;

    telemetry.current_rps = rps;

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing telemetry = {.current_rps = %f, .voltage_battery = %f, .voltage_current = %f}",
        telemetry.current_rps,
        telemetry.battery_voltage,
        telemetry.battery_current);
    signals_.telemetry->publish(telemetry);

    auto odometry = nav_msgs::msg::Odometry();
    odometry.header = header;
    auto velocity = model_->motorRPStoLinearVelocity(telemetry.current_rps);
    odometry.twist.twist.linear.x = velocity;
    odometry.twist.twist.linear.y = 0.0;
    odometry.twist.twist.linear.z = 0.0;
    std::array<double, 36> covariance_matrix;
    covariance_matrix[0] = 0.0001;
    odometry.twist.covariance = covariance_matrix;

    RCLCPP_INFO(
        this->get_logger(),
        "odometry.twist.linear = {.x = %f, .y = %f, .z = %f}",
        odometry.twist.twist.linear.x,
        odometry.twist.twist.linear.y,
        odometry.twist.twist.linear.z);

    signals_.odometry->publish(odometry);
}

void HardwareNode::pushStatus() {
    auto status = truck_msgs::msg::HardwareStatus();

    auto heartbeatFrame = canFramesCache[CmdId::kHeartbeat];
    auto motorFrame = canFramesCache[CmdId::kGetMotorError];
    auto encoderFrame = canFramesCache[CmdId::kGetEncoderError];

    uint64_t motorError = 0;
    uint32_t axisState = 0, axisError = 0, encoderError = 0;
    uint8_t motorFlag = 0, encoderFlag = 0;

    memcpy(&axisError, heartbeatFrame.first.data, sizeof(axisError));
    memcpy(&axisState, heartbeatFrame.first.data + 4, sizeof(axisState));
    memcpy(&motorFlag, heartbeatFrame.first.data + 5, sizeof(motorFlag));
    memcpy(&encoderFlag, heartbeatFrame.first.data + 6, sizeof(encoderFlag));

    if (axisError) {
        auto msg = "Axis Error Detected: " + std::to_string(axisError);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        status.errors.push_back(msg);
    }

    auto check = [](const std::chrono::system_clock::time_point& msg,
                    std::chrono::milliseconds& reportRate) -> bool {
        auto curTime = std::chrono::system_clock::now();
        return (msg >= curTime - 2 * reportRate);
    };

    auto delay = std::chrono::milliseconds(static_cast<int>(1000.0 / params_.statusReportRate));

    if (motorFlag) {
        if (check(motorFrame.second, delay)) {
            memcpy(&motorError, motorFrame.first.data, sizeof(motorError));
            auto msg = "Motor Error Detected: " + std::to_string(motorError);
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            status.errors.push_back(msg);
        } else {
            sendFrame(CmdId::kGetMotorError, 0);
        }
    }
    if (encoderFlag) {
        if (check(encoderFrame.second, delay)) {
            memcpy(&encoderError, encoderFrame.first.data, sizeof(encoderError));
            auto msg = "Encoder Error Detected: " + std::to_string(encoderError);
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            status.errors.push_back(msg);
        } else {
            sendFrame(CmdId::kGetEncoderError, 0);
        }
    }

    bool armed = (axisState != AxisState::AXIS_STATE_IDLE);

    status.header.stamp = now();
    status.armed = armed;
    signals_.status->publish(status);
}
}  // namespace truck::hardware_node