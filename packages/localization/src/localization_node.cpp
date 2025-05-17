#include "localization/localization_node.h"

#include "geom/msg.h"
#include "localization/conversion.h"

#include <chrono>
#include <fstream>
#include <rosbag2_cpp/reader.hpp>

namespace truck::localization {

using namespace std::placeholders;

LocalizationNode::LocalizationNode() : Node("localization") {
    initializeParams();
    initializeTopicHandlers();
    loadScanGlobal();

    std::ifstream icp_config(params_.icp_config);
    icp_.loadFromYaml(icp_config);

    tf2::fromMsg(geom::msg::toPose({}), tf_world_ekf_);
}

void LocalizationNode::initializeParams() {
    params_ = LocalizationParams{
        .period = std::chrono::duration<double>(this->declare_parameter<double>("period")),
        .verbose = this->declare_parameter<bool>("verbose"),
        .icp_config = this->declare_parameter<std::string>("icp_config"),

        .bbox_filter =
            {.enable = this->declare_parameter<bool>("bbox_filter.enable"),
             .radius = this->declare_parameter<double>("bbox_filter.radius")},

        .local_scan =
            {.rendering =
                 {.bbox_filtered =
                      {.enable = this->declare_parameter<bool>(
                           "local_scan.rendering.bbox_filtered.enable")}}},

        .global_scan = {
            .config = this->declare_parameter<std::string>("global_scan.config"),

            .rendering = {
                .main =
                    {.enable = this->declare_parameter<bool>("global_scan.rendering.main.enable"),
                     .period = std::chrono::duration<double>(
                         this->declare_parameter<double>("global_scan.rendering.main.period"))},

                .bbox_filtered = {
                    .enable = this->declare_parameter<bool>(
                        "global_scan.rendering.bbox_filtered.enable")}}}};
}

void LocalizationNode::initializeTopicHandlers() {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    signals_ = Signals{
        .tf = this->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::QoS(1).reliability(qos)),

        .global_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/global", rclcpp::QoS(1).reliability(qos)),

        .global_scan_bbox_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/global/bbox_filtered", rclcpp::QoS(1).reliability(qos)),

        .local_scan_bbox_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/local/bbox_filtered", rclcpp::QoS(1).reliability(qos))};

    slots_ = Slots{
        .local_scan = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&LocalizationNode::onLocalScan, this, _1)),

        .pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&LocalizationNode::onReset, this, _1))};

    timers_ = Timers{
        .main = this->create_wall_timer(
            params_.period, std::bind(&LocalizationNode::makeLocalizationTick, this)),
        .global_scan = this->create_wall_timer(
            params_.global_scan.rendering.main.period,
            std::bind(&LocalizationNode::publishScanGlobal, this))};
}

std::optional<tf2::Transform> LocalizationNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        const auto tf_msg = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        return tf;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(
            this->get_logger(), "No transform from '%s' to '%s'!", source.c_str(), target.c_str());
        return std::nullopt;
    }
}

void LocalizationNode::loadScanGlobal() {
    rosbag2_cpp::Reader reader = rosbag2_cpp::Reader();
    reader.open(params_.global_scan.config);

    if (!reader.has_next()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Corrupted bag '%s', stop parsing!",
            params_.global_scan.config.c_str());
        return;
    }

    const auto point_cloud = reader.read_next<sensor_msgs::msg::PointCloud2>();

    RCLCPP_INFO(
        this->get_logger(), "Bag '%s' was succesfully parsed", params_.global_scan.config.c_str());

    DataPoints data_points = toDataPoints(point_cloud);
    data_points.features.row(3).setZero();

    scans_.global = Scans::Global{
        .data_points = toDataPoints(point_cloud),
        .point_cloud = toPointCloud2(point_cloud.header, data_points)};
}

void LocalizationNode::onReset(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto tf_ekf_base = getLatestTranform("base", "odom_ekf");

    if (!tf_ekf_base.has_value()) {
        return;
    }

    tf2::Transform tf_world_base;
    tf2::fromMsg(msg->pose, tf_world_base);

    tf_world_ekf_ = tf_world_base * tf_ekf_base->inverse();
    RCLCPP_INFO(this->get_logger(), "Update pose estimation");
}

void LocalizationNode::onLocalScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    scans_.local = *msg;
}

namespace {

Eigen::Matrix4f transformationMatrix(const geom::Pose& pose) {
    const double dtheta = pose.dir.angle().radians();
    const double cos_dtheta = std::cos(dtheta);
    const double sin_dtheta = std::sin(dtheta);

    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();

    // Rotation
    tf_matrix(0, 0) = cos_dtheta;
    tf_matrix(0, 1) = -1.0 * sin_dtheta;
    tf_matrix(1, 0) = sin_dtheta;
    tf_matrix(1, 1) = cos_dtheta;

    // Translation
    tf_matrix(0, 3) = pose.pos.x;
    tf_matrix(1, 3) = pose.pos.y;

    return tf_matrix;
}

geom::Pose toPose(const Eigen::Matrix4f& tf_matrix) {
    const double tx = tf_matrix(0, 3);
    const double ty = tf_matrix(1, 3);
    const double dtheta = std::atan2(tf_matrix(1, 0), tf_matrix(0, 0));
    return {geom::Vec2(tx, ty), geom::AngleVec2::fromRadians(dtheta)};
}

DataPoints transformDataPoints(const DataPoints& data_points, const tf2::Transform& tf) {
    DataPoints data_points_tf = data_points;

    geometry_msgs::msg::Pose tf_pose;
    tf2::toMsg(tf, tf_pose);

    const Eigen::Matrix4f tf_matrix = transformationMatrix(geom::toPose(tf_pose));
    data_points_tf.features = tf_matrix * data_points_tf.features;

    return data_points_tf;
}

}  // namespace

void LocalizationNode::makeLocalizationTick() {
    const auto tf_ekf_base = getLatestTranform("base", "odom_ekf");
    const auto tf_base_lidar_link = getLatestTranform("lidar_link", "base");

    if (!tf_ekf_base.has_value() || !tf_base_lidar_link.has_value()) {
        return;
    }

    const auto tf_world_lidar_link =
        tf_world_ekf_ * tf_ekf_base.value() * tf_base_lidar_link.value();
    DataPoints local_scan_tf = transformDataPoints(toDataPoints(scans_.local), tf_world_lidar_link);

    DataPoints global_scan_tf = scans_.global.data_points;

    if (params_.bbox_filter.enable) {
        const geom::Vec2 ego = geom::Transform(tf_world_ekf_ * tf_ekf_base.value()).t();
        RCLCPP_INFO(
            this->get_logger(),
            "Сenter (EGO): x = %.3f, y = %.3f, radius = %.2f",
            ego.x,
            ego.y,
            params_.bbox_filter.radius);

        PointMatcherSupport::Parametrizable::Parameters bbox_filter_params = {
            {"xMin", std::to_string(ego.x - params_.bbox_filter.radius)},
            {"xMax", std::to_string(ego.x + params_.bbox_filter.radius)},
            {"yMin", std::to_string(ego.y - params_.bbox_filter.radius)},
            {"yMax", std::to_string(ego.y + params_.bbox_filter.radius)},

            {"zMin", std::to_string(-1e9)},
            {"zMax", std::to_string(1e9)},

            {"removeInside", std::to_string(false)}};

        std::shared_ptr<Matcher::DataPointsFilter> bbox_filter =
            Matcher::get().DataPointsFilterRegistrar.create(
                "BoundingBoxDataPointsFilter", bbox_filter_params);

        local_scan_tf = bbox_filter->filter(local_scan_tf);
        global_scan_tf = bbox_filter->filter(global_scan_tf);
    }

    if (local_scan_tf.features.cols() == 0) {
        RCLCPP_WARN(this->get_logger(), "local_scan_tf is empty after filtering — skipping ICP.");
        return;
    }
    if (global_scan_tf.features.cols() == 0) {
        RCLCPP_WARN(this->get_logger(), "global_scan_tf is empty after filtering — skipping ICP.");
        return;
    }

    try {
        const auto start_time = std::chrono::steady_clock::now();

        const auto tf_icp_matrix = icp_(local_scan_tf, global_scan_tf);

        const auto end_time = std::chrono::steady_clock::now();
        const auto duration_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        const geom::Pose tf_icp_pose = toPose(tf_icp_matrix);

        if (params_.verbose) {
            RCLCPP_INFO(
                this->get_logger(),
                "Localization correction: (%.3f, %.3f, %.3f), ICP time: %ld ms",
                tf_icp_pose.pos.x,
                tf_icp_pose.pos.y,
                tf_icp_pose.dir.angle().radians(),
                duration_ms);
        }

        geometry_msgs::msg::Pose tf_world_ekf_geom_msg;
        tf2::toMsg(tf_world_ekf_, tf_world_ekf_geom_msg);

        geom::Pose tf_world_ekf_geom = geom::toPose(tf_world_ekf_geom_msg);

        const double dx = std::abs(tf_icp_pose.pos.x);
        const double dy = std::abs(tf_icp_pose.pos.y);
        const double dxy = std::hypot(dx, dy);

        if (dxy > 1.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "ICP correction too large: Δx = %.3f, Δy = %.3f (total %.3f m) — ignoring update.",
                tf_icp_pose.pos.x,
                tf_icp_pose.pos.y,
                dxy);
            return;
        }

        tf_world_ekf_geom.pos += tf_icp_pose.pos;
        tf_world_ekf_geom.dir += tf_icp_pose.dir;

        tf2::fromMsg(geom::msg::toPose(tf_world_ekf_geom), tf_world_ekf_);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ICP exception caught: %s", e.what());
        RCLCPP_INFO(
            this->get_logger(),
            "ICP input sizes — local: %zu, global: %zu",
            local_scan_tf.features.cols(),
            global_scan_tf.features.cols());
    }

    publishTf();

    {
        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = this->now();

        if (params_.global_scan.rendering.bbox_filtered.enable) {
            global_scan_tf.features.row(3).setZero();
            signals_.global_scan_bbox_filtered->publish(toPointCloud2(header, global_scan_tf));
        }

        if (params_.local_scan.rendering.bbox_filtered.enable) {
            local_scan_tf.features.row(3).setZero();
            signals_.local_scan_bbox_filtered->publish(toPointCloud2(header, local_scan_tf));
        }
    }
}

void LocalizationNode::publishTf() {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.frame_id = "world";
    tf_stamped.child_frame_id = "odom_ekf";
    tf_stamped.header.stamp = now();
    tf2::toMsg(tf_world_ekf_, tf_stamped.transform);

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tf_stamped);
    signals_.tf->publish(tf_msg);
}

void LocalizationNode::publishScanGlobal() {
    if (params_.global_scan.rendering.main.enable) {
        scans_.global.point_cloud.header.stamp = this->now();
        signals_.global_scan->publish(scans_.global.point_cloud);
    }
}

}  // namespace truck::localization
