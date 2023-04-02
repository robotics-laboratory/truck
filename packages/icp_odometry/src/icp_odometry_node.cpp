#include "icp_odometry/icp_odometry_node.h"
#include "icp_odometry/conversion.h"

#include "common/math.h"
#include "geom/transform.h"

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

    icp->setDefault();

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
    odometry_ = odometry;
}

IcpOdometryNode::IcpOdometryNode() : Node("icp_odometry_node") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    scan_slot_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&IcpOdometryNode::handleLaserScan, this, std::placeholders::_1));

    cloud_signal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/odometry/cloud", 1);

    reference_cloud_signal_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/odometry/reference", 1);

    transformed_cloud_signal_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/odometry/transformed", 10);

    odometry_stat_signal_ =
        this->create_publisher<truck_interfaces::msg::IcpOdometryStat>("/odometry/stat", 10);

    odometry_signal_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/odom", 10);

    odometry_slot_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&IcpOdometryNode::handleOdometry, this, std::placeholders::_1));

    icp_ = makeICP(this->get_logger(), this->declare_parameter<std::string>("icp_config"));
    visualize_ = this->declare_parameter<bool>("visualize", false);
}

truck_interfaces::msg::IcpOdometryStat IcpResult::toOdometryStatMsg() const {
    truck_interfaces::msg::IcpOdometryStat msg;

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

    const float v = std::copysign(std::sqrt(x*x + y*y) / dt, -x);

    msg.twist.twist.linear.x = v;
    msg.twist.twist.angular.z = std::atan2(t(1, 0), t(0, 0)) / dt;

    constexpr double sigma = 2e-2;
    constexpr double var = sigma * sigma;

    msg.twist.covariance[0] = var;
    msg.twist.covariance[7] = 0;
    msg.twist.covariance[14] = 0;
    msg.twist.covariance[21] = 0;
    msg.twist.covariance[28] = 0;
    msg.twist.covariance[35] = var;

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
    auto odometry = odometry_;

    if (reference_cloud_) {
        const auto init = guessTransformation(reference_odometry_ , odometry);

        // RCLCPP_INFO(this->get_logger(), "Init matrix\n: %s", toStr(init).c_str());

        const auto result = makeIcpStep(
            scan->header, *cloud,
            reference_scan_->header, *reference_cloud_,
            init);

        // RCLCPP_INFO(this->get_logger(), "Result matrix\n: %s", toStr(result.t).c_str());

        odometry_stat_signal_->publish(result.toOdometryStatMsg());
        odometry_signal_->publish(result.toOdometryMsg());

        // if (visualize_) {
        //     DataPoints transformed_cloud(*reference_cloud_);
        //     icp_->transformations.apply(transformed_cloud, result.t);

        //     cloud_signal_->publish(toPointCloud2(scan->header, icp_->getReadingFiltered()));
        //     reference_cloud_signal_->publish(toPointCloud2(scan->header, *reference_cloud_));
        //     transformed_cloud_signal_->publish(toPointCloud2(scan->header, transformed_cloud));
        // }
    }

    reference_scan_ = std::move(scan);
    reference_cloud_ = std::move(cloud);
    if (odometry) {
        reference_odometry_ = std::move(odometry);
    }
}

}  // namespace truck::icp_odometry