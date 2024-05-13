#include "icp_odometry/icp_odometry_node.h"
#include "icp_odometry/conversion.h"

#include "common/math.h"
#include "geom/msg.h"

#include <fstream>
#include <string>

namespace truck::icp_odometry {

namespace {

std::unique_ptr<Matcher::ICP> makeICP(rclcpp::Logger logger, const std::string& config_path) {
    std::ifstream yaml(config_path);

    auto icp = std::make_unique<Matcher::ICP>();

    if (!yaml.is_open()) {
        RCLCPP_ERROR(logger, "failed to open icp config: '%s'", config_path.c_str());
        icp->setDefault();
    } else {
        RCLCPP_INFO(logger, "load icp config: '%s'", config_path.c_str());
        icp->loadFromYaml(yaml);
    }

    return icp;
}

TransformationParameters guessTransformation(
    const nav_msgs::msg::Odometry::ConstSharedPtr& reference_odometry,
    const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) {
    TransformationParameters t(3, 3);
    t.setIdentity();

    if (!reference_odometry || !odometry) {
        return t;
    }

    const float dx = odometry->pose.pose.position.x - reference_odometry->pose.pose.position.x;
    const float dy = odometry->pose.pose.position.y - reference_odometry->pose.pose.position.y;

    const geom::Angle dtheta = geom::toYawAngle(odometry->pose.pose.orientation) -
                               geom::toYawAngle(reference_odometry->pose.pose.orientation);

    // rotation
    t(0, 0) = cos(dtheta);
    t(0, 1) = -sin(dtheta);
    t(1, 0) = sin(dtheta);
    t(1, 1) = cos(dtheta);

    // transform
    t(0, 2) = dx;
    t(1, 2) = dy;

    return t;
}

}  // namespace

void IcpOdometryNode::handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
    state_.odometry = odometry;
}

IcpOdometryNode::IcpOdometryNode() : Node("icp_odometry") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    slot_.scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&IcpOdometryNode::handleLaserScan, this, std::placeholders::_1));

    slot_.odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&IcpOdometryNode::handleOdometry, this, std::placeholders::_1));

    signal_.odometry_stat =
        this->create_publisher<truck_msgs::msg::IcpOdometryStat>("/odometry/stat", 10);

    signal_.odometry = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/odom", 10);

    icp_ = makeICP(this->get_logger(), this->declare_parameter<std::string>("icp_config"));
}

truck_msgs::msg::IcpOdometryStat IcpResult::toOdometryStatMsg() const {
    truck_msgs::msg::IcpOdometryStat msg;

    msg.header = header;

    msg.latency = latency;
    msg.x = t(0, 2);
    msg.y = t(1, 2);
    msg.theta = std::atan2(t(1, 0), t(0, 0));

    return msg;
}

nav_msgs::msg::Odometry IcpResult::toOdometryMsg() const {
    nav_msgs::msg::Odometry msg;

    msg.header = header;
    msg.child_frame_id = header.frame_id;

    const rclcpp::Time start = header.stamp;
    const rclcpp::Time end = reference_header.stamp;
    const float dt = (end - start).seconds();

    const float x = t(0, 2);
    const float y = t(1, 2);

    const float v = std::copysign(std::sqrt(x * x + y * y) / dt, -x);

    msg.twist.twist.linear.x = v;
    msg.twist.twist.angular.z = std::atan2(t(1, 0), t(0, 0)) / dt;

    const double distance_error = 0.04;
    const double yaw_error = 0.01;

    msg.twist.covariance[0] = squared(distance_error);
    msg.twist.covariance[7] = 0;
    msg.twist.covariance[14] = 0;
    msg.twist.covariance[21] = 0;
    msg.twist.covariance[28] = 0;
    msg.twist.covariance[35] = squared(yaw_error);

    return msg;
}

IcpResult IcpOdometryNode::makeIcpStep(
    const std_msgs::msg::Header& header, const DataPoints& cloud,
    const std_msgs::msg::Header& reference_header, const DataPoints& reference_cloud,
    const TransformationParameters& init) {
    const auto icp_start_stamp = this->now();
    const auto t = (*icp_)(reference_cloud, cloud, init);
    const auto icp_end_stamp = this->now();

    return IcpResult{
        .header = header,
        .reference_header = reference_header,
        .latency = (icp_end_stamp - icp_start_stamp),
        .t = t,
    };
}

std::string toStr(const Eigen::Matrix3f& m) {
    std::stringstream ss;
    ss << m;
    return ss.str();
}

void IcpOdometryNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    auto cloud = std::make_shared<DataPoints>(toDataPoints(*scan));
    auto odometry = state_.odometry;

    if (state_.reference_cloud) {
        const auto init = guessTransformation(state_.reference_odometry, odometry);

        const auto result = makeIcpStep(
            scan->header, *cloud, state_.reference_scan->header, *state_.reference_cloud, init);

        signal_.odometry_stat->publish(result.toOdometryStatMsg());
        signal_.odometry->publish(result.toOdometryMsg());
    }

    state_.reference_scan = std::move(scan);
    state_.reference_cloud = std::move(cloud);
    if (odometry) {
        state_.reference_odometry = std::move(odometry);
    }
}

}  // namespace truck::icp_odometry
