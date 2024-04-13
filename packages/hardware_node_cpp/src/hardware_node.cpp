#include "hardware_node.h"

#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <linux/sockios.h>
#include <netinet/tcp.h>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/qos.hpp>
#include <unistd.h>

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace truck_msgs::msg;

namespace truck::hardware_node {

HardwareNode::HardwareNode() : Node("hardware_node") {
    initializeParams();
    initializeSocketCan();
    initializeTopicHandlers();
    initializeTimers();
    initializeOdrive();
    initializeTeensy();
    RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::initializeSocketCan() {
    can_.socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_.ifr.ifr_name, params_.interface.c_str());
    ioctl(can_.socket, SIOCGIFINDEX, &can_.ifr);
    can_.addr.can_family = AF_CAN;
    can_.addr.can_ifindex = can_.ifr.ifr_ifindex;
    if (bind(can_.socket, reinterpret_cast<struct sockaddr*>(&can_.addr), sizeof(can_.addr)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Unnable connect to CAN-socket");
        rclcpp::shutdown();
    }
    int flags = fcntl(can_.socket, F_GETFL, 0);
    fcntl(can_.socket, F_SETFL, flags | O_NONBLOCK);
    int enable = 1;
    setsockopt(can_.socket, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));
    sleep(1);
    readFromSocket();
}

void HardwareNode::initializeOdrive() {
    auto accelMps = static_cast<float>(model_->baseMaxAcceleration());
    auto accelRps = static_cast<float>(model_->linearVelocityToMotorRPS(accelMps));
    RCLCPP_INFO(this->get_logger(), "Acceleration: %.1f m/s^2, %.1f turns/s^2", accelMps, accelRps);
    disableMotor();
}

void HardwareNode::initializeTeensy() {
    steeringControl_ = std::make_unique<servo::SteeringControl>(params_.steeringPath);
    RCLCPP_INFO(this->get_logger(), "Teensy initialized");
}

void HardwareNode::initializeParams() {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));
    params_.steeringPath = this->declare_parameter("steering_path", "../resource/steering.csv");
    params_.odriveCanId = this->declare_parameter("node_id", 39);
    params_.servoHomeAngleLeft = this->declare_parameter("steering_base_left", 90.0);
    params_.servoHomeAngleRight = this->declare_parameter("steering_base_right", 90.0);
    params_.odriveAxis = this->declare_parameter("odrive_axis", "axis1");
    params_.interface = this->declare_parameter("interface", "vcan0");
    params_.statusReportRate = this->declare_parameter("status_report_rate", 20.0);
    params_.telemetryReportRate = this->declare_parameter("telemetry_report_rate", 20.0);
    params_.readReportRate = this->declare_parameter("read_report_rate", 20.0);
    /*
    params_.teensySerialPort = this->declare_parameter("teensy_serial_port", "/dev/ttyTHS0");
    params_.teensySerialSpeed = this->declare_parameter("teensy_serial_speed", 500000);
    */
    params_.odriveTimeout =
        std::chrono::milliseconds(this->declare_parameter<long int>("odrive_timeout", 250));

    RCLCPP_INFO(this->get_logger(), "curvature: %f", model_->baseMaxAbsCurvature());
    RCLCPP_INFO(this->get_logger(), "node_id: %d", params_.odriveCanId);
    RCLCPP_INFO(this->get_logger(), "axis: %s", params_.odriveAxis.c_str());
    RCLCPP_INFO(this->get_logger(), "interface: %s", params_.interface.c_str());
    RCLCPP_INFO(this->get_logger(), "odrive timeout: %ld", params_.odriveTimeout.count());
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

namespace {
struct __attribute__((packed)) InputVel {
    float input_vel;
    float torq;
};

float interpolate(const std::vector<std::pair<float, float>>& data, float x) noexcept {
    auto comp = [](const std::pair<float, float>& elem, float value) { return elem.first < value; };
    auto it = std::lower_bound(data.begin(), data.end(), x, comp);
    if (it == data.end()) {
        return (it - 1)->second;
    }
    if (it == data.begin()) {
        return it->second;
    }
    auto itPrev = it - 1;
    float x0 = itPrev->first, y0 = itPrev->second;
    float x1 = it->first, y1 = it->second;
    if (x1 == x0) {
        return y0;
    }
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

float rad2degree(float rad) noexcept { return rad * (180.0f / M_PI); }
}  // namespace

void HardwareNode::pushTeensy(float left_wheel_angle, float right_wheel_angle) {
    auto left_servo_angle = interpolate(steeringControl_->steering_, -left_wheel_angle);
    auto right_servo_angle = interpolate(steeringControl_->steering_, right_wheel_angle);
    left_servo_angle = params_.servoHomeAngleLeft + left_servo_angle;
    right_servo_angle = params_.servoHomeAngleRight - right_servo_angle;
    servo::Angles data = {rad2degree(left_servo_angle), rad2degree(right_servo_angle)};
    RCLCPP_INFO(
        this->get_logger(),
        "Servo angles: {left = %.3f, right = %.3f}",
        data.left_angle,
        data.right_angle);
    static_assert(sizeof(servo::Angles) == 8, "Padding issues with servo angles struct");
    sendFrame(CmdId::SET_SERVO_ANGLE, sizeof(servo::Angles), &data);
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

    // https://github.com/linux-can/can-utils/blob/master/candump.c
    for (;;) {
        ssize_t nbytes = recvmsg(can_.socket, &msg, 0);
        if (nbytes < 0) {
            break;
        }
        std::chrono::system_clock::time_point rcv_time;
        bool flag = false;
        for (auto cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                struct timeval tv = *(struct timeval*)CMSG_DATA(cmsg);
                rcv_time = std::chrono::system_clock::from_time_t(tv.tv_sec) +
                           std::chrono::microseconds(tv.tv_usec);
                flag = true;
                break;
            }
        }
        if (!flag) {
            RCLCPP_DEBUG(this->get_logger(), "No timestamp header");
            rclcpp::shutdown();
        }
        uint32_t cmd = frame.can_id & 0x1f;
        auto cmdId = static_cast<CmdId>(cmd);
        canFramesCache[cmdId] = std::make_pair(frame, rcv_time);
    }
}

void HardwareNode::modeCallback(const ControlMode& msg) {
    if (msg.mode == curMode) {
        return;
    }
    if (curMode == ControlMode::OFF && msg.mode != ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: OFF -> ANY - enabling motor");
        enableMotor();
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
    canid_t id = (params_.odriveCanId << 5) | cmd_id;
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = can_dlc;
    if (data != nullptr) {
        std::memcpy(frame.data, data, can_dlc);
    }
    RCLCPP_INFO(this->get_logger(), "COMMAND: %u", cmd_id);
    std::stringstream ss;
    for (int i = 0; i < can_dlc; ++i) {
        ss >> frame.data[i];
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    ssize_t nbytes = write(can_.socket, &frame, sizeof(frame));
    if (nbytes < 0 || nbytes != sizeof(frame)) {
        RCLCPP_INFO(this->get_logger(), "Unnable to write to socket");
        rclcpp::shutdown();
    }
}

void HardwareNode::commandCallback(const Control& msg) {
    if (curMode == ControlMode::OFF) return;
    auto rpm = static_cast<float>(model_->linearVelocityToMotorRPS(msg.velocity));
    InputVel ss{rpm, 0.0};
    static_assert(sizeof(ss) == 8, "Padding issues with InputVel struct");
    sendFrame(CmdId::SET_INPUT_VEL, sizeof(InputVel), &ss);
    truck::model::Twist twist = truck::model::Twist{msg.curvature, msg.velocity};
    twist = model_->baseToRearTwist(twist);
    auto steering = model_->rearTwistToSteering(twist);
    RCLCPP_DEBUG(this->get_logger(), "Center curv: %.2f", msg.curvature);
    RCLCPP_DEBUG(this->get_logger(), "Rear curv: %.2f", twist.curvature);
    auto left = static_cast<float>(steering.left.radians());
    auto right = static_cast<float>(steering.right.radians());
    pushTeensy(left, right);
}

void HardwareNode::enableMotor() {
    sendFrame(CmdId::CLEAR_ERRORS, 0);
    double rpm = 0.0;
    sendFrame(CmdId::SET_INPUT_VEL, sizeof(rpm), &rpm);
    auto state = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    sendFrame(CmdId::SET_AXIS_STATE, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "Motor enabled");
}

void HardwareNode::disableMotor() {
    auto state = AxisState::AXIS_STATE_IDLE;
    sendFrame(CmdId::SET_AXIS_STATE, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "Motor disabled");
}

std::optional<can_frame> HardwareNode::checkCanFrame(
    const CmdId& command, const double reportRate) {
    auto it = canFramesCache.find(command);
    auto checkTime = [](const std::chrono::system_clock::time_point& msgTime,
                        const double reportRate) -> bool {
        std::chrono::milliseconds delay =
            std::chrono::milliseconds(static_cast<int>(1000.0 / reportRate));
        std::chrono::system_clock::time_point curTime = std::chrono::system_clock::now();
        auto time = msgTime - curTime;
        return time <= 2 * delay;
    };
    if (it != canFramesCache.end()) {
        const auto& [frame, timestamp] = it->second;
        if (checkTime(timestamp, reportRate)) {
            return frame;
        }
    }
    return {};
}

void HardwareNode::pushTelemetry() {
    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "base";
    auto telemetry = truck_msgs::msg::HardwareTelemetry();
    telemetry.header = header;

    auto telem = params_.telemetryReportRate;
    auto voltageFrame = checkCanFrame(CmdId::GET_BUS_VOLTAGE_CURRENT, telem);
    auto encoderFrame = checkCanFrame(CmdId::GET_ENCODER_ESTIMATES, telem);

    if (!voltageFrame || !encoderFrame) {
        RCLCPP_WARN(this->get_logger(), "ERROR. GOT OLD FRAME WITH TELEMETRY");
        rclcpp::shutdown();
    }
    float voltage = 0.0, current = 0.0, rps = 0.0;

    std::memcpy(&voltage, voltageFrame->data, sizeof(voltage));
    std::memcpy(&current, voltageFrame->data + 4, sizeof(current));
    std::memcpy(&rps, encoderFrame->data + 4, sizeof(rps));
    telemetry.battery_voltage = voltage;
    telemetry.battery_current = current;
    telemetry.current_rps = rps;
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
    signals_.odometry->publish(odometry);

    RCLCPP_INFO(
        this->get_logger(),
        "odometry.twist.linear = {.x = %f, .y = %f, .z = %f}",
        odometry.twist.twist.linear.x,
        odometry.twist.twist.linear.y,
        odometry.twist.twist.linear.z);

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing telemetry = {.current_rps = %f, .voltage_battery = %f, .voltage_current = %f}",
        telemetry.current_rps,
        telemetry.battery_voltage,
        telemetry.battery_current);
}

static int prevMotorFlag = 0;
static int prevEncoderFlag = 0;

void HardwareNode::pushStatus() {
    auto status = truck_msgs::msg::HardwareStatus();
    uint64_t motorError = 0;
    uint32_t axisState = 0, axisError = 0, encoderError = 0;
    uint8_t motorFlag = 0, encoderFlag = 0;
    auto heartbeatFrame = checkCanFrame(CmdId::HEARTBEAT, params_.statusReportRate);
    auto motorFrame = checkCanFrame(CmdId::GET_MOTOR_ERROR, params_.statusReportRate);
    auto encoderFrame = checkCanFrame(CmdId::GET_ENCODER_ERROR, params_.statusReportRate);
    if (!heartbeatFrame || !motorFrame || !encoderFrame) {
        RCLCPP_WARN(this->get_logger(), "Too old status frame");
        rclcpp::shutdown();
    }
    std::memcpy(&axisError, heartbeatFrame->data, sizeof(axisError));
    std::memcpy(&axisState, heartbeatFrame->data + 4, sizeof(axisState));
    std::memcpy(&motorFlag, heartbeatFrame->data + 5, sizeof(motorFlag));
    std::memcpy(&encoderFlag, heartbeatFrame->data + 6, sizeof(encoderFlag));
    if (axisError) {
        auto msg = "Axis Error Detected: " + std::to_string(axisError);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        status.errors.push_back(msg);
    }
    if (prevMotorFlag) {
        std::memcpy(&motorError, motorFrame->data, sizeof(motorError));
        auto msg = "Motor Error Detected: " + std::to_string(motorError);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        status.errors.push_back(msg);
    }
    if (prevEncoderFlag) {
        std::memcpy(&encoderError, encoderFrame->data, sizeof(encoderError));
        auto msg = "Encoder Error Detected: " + std::to_string(encoderError);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        status.errors.push_back(msg);
    }
    if (motorFlag) {
        sendFrame(CmdId::GET_MOTOR_ERROR, 0);
        prevMotorFlag = 1;
    }
    if (encoderFlag) {
        sendFrame(CmdId::GET_ENCODER_ERROR, 0);
        prevEncoderFlag = 1;
    }
    prevMotorFlag = 0;
    prevEncoderFlag = 0;
    bool armed = (axisState != AxisState::AXIS_STATE_IDLE);
    status.header.stamp = now();
    status.armed = armed;
    signals_.status->publish(status);
}
}  // namespace truck::hardware_node