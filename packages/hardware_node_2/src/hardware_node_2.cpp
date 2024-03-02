#include "hardware_node_2.h"

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
    initializeSocketCan();
    initializePtrFields();
    initializeParams();
    initializeTopicHandlers();
    initializeTimers();
    initializeOdrive();
    RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::initializeSocketCan() {
    can_.socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_.ifr.ifr_name, "vxcan1");
    ioctl(can_.socket, SIOCGIFINDEX, &can_.ifr);
    can_.addr.can_family = AF_CAN;
    can_.addr.can_ifindex = can_.ifr.ifr_ifindex;
    bind(can_.socket, reinterpret_cast<struct sockaddr*>(&can_.addr), sizeof(can_.addr));
    int flags = fcntl(can_.socket, F_GETFL, 0);
    fcntl(can_.socket, F_SETFL, flags | O_NONBLOCK);
    int enable = 1;
    setsockopt(can_.socket, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));
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
    params_.statusReportRate = std::chrono::duration<double>(
        this->declare_parameter("status_report_rate", 20.0));
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
    timers_.telemetryTimer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.telemetryReportRate.count())),
        std::bind(&HardwareNode::pushTelemetry, this));

    timers_.statusTimer = Node::create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.statusReportRate.count())),
        std::bind(&HardwareNode::pushStatus, this));

    timers_.socketRead =
        Node::create_wall_timer(50ms, std::bind(&HardwareNode::readFromSocket, this));
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
        if (nbytes > 0) {
            struct timeval tv;
            for (auto cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
                if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                    tv = *(struct timeval*)CMSG_DATA(cmsg);
                    break;
                }
            }
            auto cmdId = frame.can_id & 0x1f;
            RCLCPP_INFO(
                this->get_logger(),
                "Read cmd_id: %d. Time: (%010llu.%06llu)",
                cmdId,
                (unsigned long long)tv.tv_sec,
                (unsigned long long)tv.tv_usec);
            framesCache[cmdId] = std::make_pair(frame, tv);
            continue;
        }
        break;
    }
}

void HardwareNode::modeCallback(const truck_msgs::msg::ControlMode& msg) {
    if (msg.mode == truck_msgs::msg::ControlMode::OFF) {
        return;
    }
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF &&
        msg.mode != truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: OFF -> ANY - enabling motor");
        enableMotor();
        return;
    }
    if (prevMode_ != truck_msgs::msg::ControlMode::OFF &&
        msg.mode == truck_msgs::msg::ControlMode::OFF) {
        RCLCPP_INFO(this->get_logger(), "Mode change: ANY -> OFF - disabling motor");
    }
    prevMode_ = msg.mode;
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
        RCLCPP_WARN(this->get_logger(), "Failed to write to socket");
    }
}

void HardwareNode::initializeOdrive() {
    auto accelMps = model_->baseMaxAcceleration();
    auto accelRps = model_->linearVelocityToMotorRPS(accelMps);
    RCLCPP_INFO(this->get_logger(), "Acceleration: %.1f m/s^2, %.1f turns/s^2", accelMps, accelRps);
    sendFrame(CmdId::kSetAxisState, sizeof(accelRps), &accelRps);
    disableMotor();
}

void HardwareNode::commandCallback(const truck_msgs::msg::Control& msg) {
    if (prevMode_ == truck_msgs::msg::ControlMode::OFF) {
        disableMotor();
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
    uint8_t state = CmdId::kSetAxisState;
    sendFrame(CmdId::kSetAxisState, sizeof(state), &state);
    RCLCPP_INFO(this->get_logger(), "Motor disabled");
}

void HardwareNode::pushTelemetry() {
    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "base";

    auto telemetry = truck_msgs::msg::HardwareTelemetry();
    telemetry.header = header;
    auto voltageFrame = framesCache[CmdId::kGetBusVoltageCurrent];

    float voltage = 0.0, current = 0.0, rps = 0.0;

    memcpy(&voltage, voltageFrame.first.data, sizeof(voltage));
    memcpy(&current, voltageFrame.first.data + 4, sizeof(current));

    auto encoderFrame = framesCache[CmdId::kGetEncoderEstimates];
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

    auto heartbeatFrame = framesCache[CmdId::kHeartbeat];
    auto motorFrame = framesCache[CmdId::kGetMotorError];
    auto encoderFrame = framesCache[CmdId::kGetEncoderError];

    uint32_t axisState = 0;

    uint64_t motorError = 0;
    uint32_t axisError = 0, encoderError = 0;
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

    auto cmp = [](const struct timeval& t1, const struct timeval& t2) {
        if (t1.tv_sec < t2.tv_sec) {
            return true;
        } else if (t1.tv_sec == t2.tv_sec) {
            return t1.tv_usec <= t2.tv_usec;
        } else {
            return false;
        }
    };

    struct timeval currTime;
    ioctl(can_.socket, SIOCGSTAMP, &currTime);
    RCLCPP_INFO(
                this->get_logger(),
                "PushStatus time: (%010llu.%06llu)",
                (unsigned long long)currTime.tv_sec,
                (unsigned long long)currTime.tv_usec);    

    // cached[cmId].time * 2 <= cur_time ?

    auto tmp = motorFrame.second;
    tmp.tv_sec *= 2; // ?
    tmp.tv_usec *= 2; // ?
    if (motorFlag) {
        if (cmp(tmp, currTime)) {
            memcpy(&motorError, motorFrame.first.data, sizeof(motorError));
            auto msg = "Motor Error Detected: " + std::to_string(motorError);
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            status.errors.push_back(msg);
        } else {
            sendFrame(CmdId::kGetMotorError, 0);
        }
    }

    tmp = encoderFrame.second;
    tmp.tv_sec *= 2;
    tmp.tv_usec *= 2;
    if (encoderFlag) {
        if (cmp(tmp, currTime)) {
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