#include "localization/localization_node.h"
#include "localization/conversion.h"

#include "geom/msg.h"

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::localization {

using namespace std::placeholders;
using namespace std::chrono_literals;

LocalizationNode::LocalizationNode() : Node("localization") {
    initializeParams();
    initializeTransforms();
    initializeTopicHandlers();

    loadMap();

    std::ifstream icp_config(params_.icp_config_path);
    icp_.loadFromYaml(icp_config);
}

void LocalizationNode::initializeParams() {
    params_ = LocalizationParams {
        .map_scan_path = this->declare_parameter<std::string>("lidar_map_config"),
        .icp_config_path = this->declare_parameter<std::string>("icp_config"),
        .bbox_radius = this->declare_parameter<double>("bbox_radius")
    };
}

void LocalizationNode::initializeTransforms() {
    tf2::fromMsg(geom::msg::toPose({}), state_.tf.world_base);
    tf2::fromMsg(geom::msg::toPose({}), state_.tf.ekf_base);
    tf2::fromMsg(geom::msg::toPose({}), state_.tf.world_ekf);
}

void LocalizationNode::initializeTopicHandlers() {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    signals_ = Signals{
        .map_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/map", rclcpp::QoS(1).reliability(qos)),

        .local_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/local", rclcpp::QoS(1).reliability(qos)),

        .transform = this->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::QoS(1).reliability(qos))};

    slots_ = Slots{
        .scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar/scan",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&LocalizationNode::handleLaserScan, this, _1)),

        .pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&LocalizationNode::onReset, this, _1))};

    timers_.map_scan =
       this->create_wall_timer(1000ms, std::bind(&LocalizationNode::publishMapScan, this));

    timers_.transform =
        this->create_wall_timer(100ms, std::bind(&LocalizationNode::makeLocalizationTick, this));
}

void LocalizationNode::loadMap() {
    /** @todo: search for topic msg with a specific name */

    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(params_.map_scan_path);

    if (!reader->has_next()) {
        RCLCPP_ERROR(this->get_logger(), "no data in mcap, stop parsing!");
        return;
    }

    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    sensor_msgs::msg::PointCloud2::SharedPtr ros_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>();

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    serialization.deserialize_message(&serialized_msg, ros_msg.get());
    map_.point_cloud = *ros_msg;
    map_.data_points = toDataPoints(*ros_msg);
}

std::optional<tf2::Transform> LocalizationNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        const auto tf_msg = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        return tf;
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void LocalizationNode::onReset(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    tf2::fromMsg(msg->pose, state_.tf.world_base);
    state_.tf.world_ekf = state_.tf.world_base * state_.tf.ekf_base.inverse();
}

void LocalizationNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    state_.scan = *msg;
}

void LocalizationNode::makeLocalizationTick() {
    const auto tf_ekf_base = getLatestTranform("base", "odom_ekf");

    if (tf_ekf_base.has_value()) {
        state_.tf.ekf_base = *tf_ekf_base;
    }

    const auto tf_base_lidar_link = getLatestTranform("lidar_link", "base");

    if (tf_base_lidar_link.has_value()) {
        state_.tf.base_lidar_link = *tf_base_lidar_link;
    }

    makeTransformationICP();
    publishTransform();
}

namespace {

Eigen::Matrix3f transformationMatrix(const geom::Pose& pose) {
    const double dtheta = pose.dir.angle().radians();
    const double cos_dtheta = std::cos(dtheta);
    const double sin_dtheta = std::sin(dtheta);

    Eigen::Matrix3f tf_matrix = Eigen::Matrix3f::Identity();

    // Rotation
    tf_matrix(0, 0) = cos_dtheta;
    tf_matrix(0, 1) = -1.0 * sin_dtheta;
    tf_matrix(1, 0) = sin_dtheta;
    tf_matrix(1, 1) = cos_dtheta;

    // Translation
    tf_matrix(0, 2) = pose.pos.x;
    tf_matrix(1, 2) = pose.pos.y;

    return tf_matrix;
}

geom::Pose toPose(const Eigen::Matrix3f& tf_matrix) {
    const double tx = tf_matrix(0, 2);
    const double ty = tf_matrix(1, 2);
    const double dtheta = std::atan2(tf_matrix(1, 0), tf_matrix(0, 0));
    return {geom::Vec2(tx, ty), geom::AngleVec2(dtheta)};
}

}  // namespace

void LocalizationNode::makeTransformationICP() {
    if (!state_.scan.has_value()) {
        return;
    }

    tf2::Transform tf_world_lidar_link = state_.tf.world_ekf * state_.tf.ekf_base * state_.tf.base_lidar_link;
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf_world_lidar_link, pose);
    Eigen::Matrix3f tf_matrix_world_lidar_link = transformationMatrix(geom::toPose(pose));

    DataPoints scan_lidar_link = toDataPoints(*state_.scan);     
    DataPoints scan_world = scan_lidar_link;
    scan_world.features = tf_matrix_world_lidar_link * scan_world.features;

    const double radius = params_.bbox_radius;
    const geom::Vec2 ego = geom::Transform(state_.tf.world_ekf * state_.tf.ekf_base).t();

    PointMatcherSupport::Parametrizable::Parameters params;
    params["xMin"] = std::to_string(ego.x - radius);
    params["xMax"] = std::to_string(ego.x + radius);
    params["yMin"] = std::to_string(ego.y - radius);
    params["yMax"] = std::to_string(ego.y + radius);
    params["zMin"] = std::to_string(-0.15);
    params["zMax"] = std::to_string(0.15);
    params["removeInside"] = "0";

    std::shared_ptr<Matcher::DataPointsFilter> bounding_box_gilter =
        Matcher::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
    
    scan_world = bounding_box_gilter->filter(scan_world);

    auto map_data_points_masked = bounding_box_gilter->filter(map_.data_points);
    
    /*
    std_msgs::msg::Header header;
    header.frame_id = "world";
    header.stamp = this->now();
    signals_.map_scan->publish(toPointCloud2(header, map_data_points_masked));
    */

    {
        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = now();
        signals_.local_scan->publish(toPointCloud2(header, scan_world));
    }

    try {
        auto tf = icp_(scan_world, map_data_points_masked);
        geom::Pose diff = toPose(tf);
        std::string log = "diff\n pos: (" + std::to_string(diff.pos.x) + ", " + std::to_string(diff.pos.y) + "), dir: " + std::to_string(diff.dir.angle().radians()) + "\n";
        RCLCPP_INFO(this->get_logger(), log.c_str());

        geometry_msgs::msg::Pose world_ekf_pose;
        tf2::toMsg(state_.tf.world_ekf, world_ekf_pose);

        geom::Pose tmp = geom::toPose(world_ekf_pose);
        tmp.pos += diff.pos;
        tmp.dir += diff.dir;

        tf2::fromMsg(geom::msg::toPose(tmp), state_.tf.world_ekf);
    } catch(const std::exception& e) {
        RCLCPP_INFO(this->get_logger(), "Localization error");
    }
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped tf_msg_world_ekf;
    tf_msg_world_ekf.header.frame_id = "world";
    tf_msg_world_ekf.child_frame_id = "odom_ekf";
    tf_msg_world_ekf.header.stamp = now();
    tf2::toMsg(state_.tf.world_ekf, tf_msg_world_ekf.transform);

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tf_msg_world_ekf);
    signals_.transform->publish(tf_msg);
}

void LocalizationNode::publishMapScan() {
    std_msgs::msg::Header header;
    header.frame_id = "world";
    header.stamp = this->now();

    signals_.map_scan->publish(toPointCloud2(header, map_.data_points));
}

}  // namespace truck::localization