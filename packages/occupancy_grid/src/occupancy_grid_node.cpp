#include "occupancy_grid/occupancy_grid_node.h"

#include "common/math.h"
#include "geom/msg.h"
#include "geom/transform.h"
#include "geom/vector.h"

#include <cv_bridge/cv_bridge.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace {

struct DepthTraits {
    static inline bool valid(uint16_t depth) { return depth != 0; }
    static inline float toMeters(uint16_t depth) { return depth * 0.001f; }  // originally mm
    static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
};

}  // namespace

namespace truck::occupancy_grid {

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    params_ = {
        this->declare_parameter("frame_id", "odom_ekf"),
        this->declare_parameter("resolution", 0.1),
        this->declare_parameter("radius", 20.0),
        this->declare_parameter("enable_lidar_grid", false),
        this->declare_parameter("enable_camera_grid", false),
        this->declare_parameter("enable_camera_cloud", false),
        Limits<double>{
            this->declare_parameter("camera_view_hmin", -0.05),
            this->declare_parameter("camera_view_hmax", 0.01)},
        this->declare_parameter("camera_view_distance", 2.0)};

    RCLCPP_INFO(this->get_logger(), "frame_id: %s", params_.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "resolution: %.2fm", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "radius: %.2fm", params_.radius);
    RCLCPP_INFO(this->get_logger(), "enable_camera_cloud: %d", params_.enable_camera_cloud);
    RCLCPP_INFO(this->get_logger(), "enable_lidar_grid: %d", params_.enable_lidar_grid);
    RCLCPP_INFO(this->get_logger(), "enable_camera_grid: %d", params_.enable_camera_grid);
    RCLCPP_INFO(this->get_logger(), "camera_view_distance: %.2f", params_.camera_view_distance);
    RCLCPP_INFO(
        this->get_logger(),
        "camera_view_height: [%.3f, %.3f]m",
        params_.camera_view_height.min,
        params_.camera_view_height.max);

    slot_.camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info",
        10,
        std::bind(&OccupancyGridNode::handleCameraInfo, this, std::placeholders::_1));

    slot_.camera = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&OccupancyGridNode::handleCameraDepth, this, std::placeholders::_1));

    slot_.lidar = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));

    if (params_.enable_camera_cloud) {
        signal_.camera_cloud =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointcloud", 10);
    }

    signal_.grid = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void OccupancyGridNode::handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info) {
    state_.camera_info = info;
}

std::optional<tf2::Transform> OccupancyGridNode::getLatestTranform(
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

void OccupancyGridNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    if (!params_.enable_lidar_grid) {
        return;
    }

    const auto& from_id = scan->header.frame_id;
    const auto& to_id = params_.frame_id;

    const auto tf_opt = getLatestTranform(from_id, to_id);
    if (!tf_opt) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "No transform from '%s' to '%s'!",
            from_id.c_str(),
            to_id.c_str());
        return;
    }

    geom::Transform tf(*tf_opt);

    auto odom_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    odom_cloud->header.frame_id = to_id;
    odom_cloud->header.stamp = scan->header.stamp;

    odom_cloud->is_dense = false;
    odom_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*odom_cloud);
    modifier.setPointCloud2Fields(
        2,
        "x",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "y",
        1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(scan->ranges.size());

    sensor_msgs::PointCloud2Iterator<float> x(*odom_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(*odom_cloud, "y");
    // it's also possible to add z coordinate for visualization

    Limits limit{scan->range_min, scan->range_max};

    size_t point_n = 0;
    for (size_t k = 0; k < scan->ranges.size(); ++k) {
        const double range = scan->ranges[k];
        const bool valid = std::isnormal(range) && limit.isMet(range);

        if (!valid) {
            continue;
        }

        const geom::Angle angle(scan->angle_min + k * scan->angle_increment);
        const geom::Vec2 base_point = range * geom::Vec2::fromAngle(angle);
        const geom::Vec2 odom_point = tf.apply(base_point);
        *x = odom_point.x;
        *y = odom_point.y;

        ++point_n;
        ++x;
        ++y;
    }

    modifier.resize(point_n);
    odom_cloud->height = 1;
    odom_cloud->width = point_n;

    state_.odom_lidar_points = std::move(odom_cloud);
    publishOccupancyGrid();
}

void OccupancyGridNode::handleCameraDepth(sensor_msgs::msg::Image::ConstSharedPtr image) {
    if (!params_.enable_camera_grid) {
        return;
    }

    const auto& from_id = image->header.frame_id;
    const auto& to_id = params_.frame_id;

    const auto tf_opt = getLatestTranform(from_id, to_id);
    if (!tf_opt) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "No transform from `%s` to `%s`!",
            from_id.c_str(),
            to_id.c_str());
        return;
    }

    const auto& tf = *tf_opt;

    if (!state_.camera_info) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "No camera info!");
        return;
    }

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(state_.camera_info);

    auto depth_map = cv_bridge::toCvShare(image, image->encoding);
    auto odom_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    odom_cloud->header.frame_id = to_id;
    odom_cloud->header.stamp = image->header.stamp;

    odom_cloud->is_dense = false;
    odom_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*odom_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(image->height * image->width);

    sensor_msgs::PointCloud2Iterator<float> x(*odom_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(*odom_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(*odom_cloud, "z");

    size_t point_n = 0;
    for (int v = 0; v < depth_map->image.rows; ++v) {
        for (int u = 0; u < depth_map->image.cols; ++u) {
            const auto depth = depth_map->image.at<uint16_t>(v, u);
            if (!DepthTraits::valid(depth)) {
                continue;
            }

            const cv::Point2d image_point{static_cast<double>(u), static_cast<double>(v)};
            const auto point =
                DepthTraits::toMeters(depth) * model.projectPixelTo3dRay(image_point);

            const auto vec = tf2::Vector3{point.x, point.y, point.z};
            if (vec.z() > params_.camera_view_distance) {
                continue;
            }

            const auto odom_vec = tf(vec);

            if (!params_.camera_view_height.isMet(odom_vec.z())) {
                continue;
            }

            *x = odom_vec.x();
            *y = odom_vec.y();
            *z = odom_vec.z();

            ++point_n;
            ++x;
            ++y;
            ++z;
        }
    }
    pcd_modifier.resize(point_n);
    odom_cloud->height = 1;
    odom_cloud->width = point_n;
    state_.odom_camera_points = std::move(odom_cloud);
    publishOccupancyGrid();

    if (params_.enable_camera_cloud) {
        signal_.camera_cloud->publish(*state_.odom_camera_points);
    }
}

namespace {

builtin_interfaces::msg::Time max(
    builtin_interfaces::msg::Time lhs, builtin_interfaces::msg::Time rhs) {
    if (lhs.sec > rhs.sec) {
        return lhs;
    } else if (lhs.sec < rhs.sec) {
        return rhs;
    }

    return lhs.nanosec > rhs.nanosec ? lhs : rhs;
}

std_msgs::msg::Header mergeHeader(
    const sensor_msgs::msg::PointCloud2::SharedPtr& lhs,
    const sensor_msgs::msg::PointCloud2::SharedPtr& rhs) {
    VERIFY(lhs || rhs);

    if (!lhs) {
        return rhs->header;
    } else if (!rhs) {
        return lhs->header;
    }

    VERIFY(lhs->header.frame_id == rhs->header.frame_id);

    std_msgs::msg::Header header;

    header.frame_id = lhs->header.frame_id;
    header.stamp = max(lhs->header.stamp, rhs->header.stamp);

    return header;
}

}  // namespace

void OccupancyGridNode::publishOccupancyGrid() {
    const std::string from_id = "base";
    const auto& to_id = params_.frame_id;

    const auto tf_opt = getLatestTranform(from_id, to_id);
    if (!tf_opt) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "No transform from `%s` to `%s`!",
            from_id.c_str(),
            to_id.c_str());

        return;
    }

    const double radius = params_.radius;
    const auto cell_radius = ceil<int>(radius / params_.resolution);
    const auto cell_num = 2 * cell_radius + 1;

    const geom::Transform tf(*tf_opt);
    const geom::Vec2 origin = tf.apply({0, 0}) - geom::Vec2{radius, radius};

    nav_msgs::msg::OccupancyGrid grid;

    grid.header.frame_id = params_.frame_id;
    grid.header = mergeHeader(state_.odom_camera_points, state_.odom_lidar_points);

    grid.info.resolution = params_.resolution;
    grid.info.width = cell_num;
    grid.info.height = cell_num;

    grid.info.origin.position.x = origin.x;
    grid.info.origin.position.y = origin.y;
    grid.info.origin.position.z = 0;

    grid.info.origin.orientation.x = 0;
    grid.info.origin.orientation.y = 0;
    grid.info.origin.orientation.z = 0;
    grid.info.origin.orientation.w = 1;

    grid.data.resize(cell_num * cell_num);
    std::fill(grid.data.begin(), grid.data.end(), 0);

    auto add = [&](const sensor_msgs::msg::PointCloud2& cloud) {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");

        for (; x != x.end(); ++x, ++y) {
            const geom::Vec2 point = {*x, *y};
            const geom::Vec2 grid_point = (point - origin) / params_.resolution;

            const auto i = floor<int>(grid_point.x);
            if (i < 0 || i >= cell_num) {
                continue;
            }

            const auto j = floor<int>(grid_point.y);
            if (j < 0 || j >= cell_num) {
                continue;
            }

            grid.data[i + j * grid.info.width] = 100;
        }
    };

    if (params_.enable_lidar_grid && state_.odom_lidar_points) {
        add(*state_.odom_lidar_points);
    }

    if (params_.enable_camera_grid && state_.odom_camera_points) {
        add(*state_.odom_camera_points);
    }

    signal_.grid->publish(grid);
}

}  // namespace truck::occupancy_grid
