#include "lidar_map/conversion.h"

#include <common/math.h>
#include <pcl_conversions/pcl_conversions.h>

namespace truck::lidar_map {

namespace {

constexpr bool isBigendian() { return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__; }

}  // namespace

geom::Poses toPoses(const std::vector<nav_msgs::msg::Odometry>& odom_msgs) {
    geom::Poses poses;

    for (const auto& odom_msg : odom_msgs) {
        poses.push_back(geom::toPose(odom_msg));
    }

    return poses;
}

/**
 * Converts a ROS2 PointCloud2 message to a custom Cloud format while filtering points based on
 * intensity.
 *
 * @param point_cloud Input ROS2 PointCloud2 message.
 * @param intensity_threshold Minimum intensity value to retain a point.
 * @return Cloud object containing filtered points.
 */
Cloud toCloud(const sensor_msgs::msg::PointCloud2& point_cloud, const int intensity_threshold) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_intensity;

    pcl::fromROSMsg(point_cloud, pcl_cloud_intensity);
    pcl_conversions::toPCL(point_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    std::vector<Eigen::Vector3f> filtered_points;
    const size_t point_count = pcl_cloud.size();
    for (size_t j = 0; j < point_count; ++j) {
        const auto& point = pcl_cloud.points[j];
        float intensity = pcl_cloud_intensity.points[j].intensity;
        if (intensity > intensity_threshold) {
            filtered_points.emplace_back(point.x, point.y, point.z);
        }
    }

    const size_t filtered_point_count = filtered_points.size();
    Cloud cloud(4, filtered_point_count);
    for (size_t j = 0; j < filtered_point_count; ++j) {
        cloud(0, j) = filtered_points[j][0];
        cloud(1, j) = filtered_points[j][1];
        cloud(2, j) = filtered_points[j][2];
        cloud(3, j) = 1.0;
    }

    return cloud;
}

/**
 * Converts a vector of ROS2 PointCloud2 messages into a vector of Clouds while filtering based on
 * intensity.
 *
 * @param scans Vector of ROS2 PointCloud2 messages.
 * @param intensity_threshold Minimum intensity value to retain a point.
 * @return A vector of Cloud objects containing filtered points from each input scan.
 */
Clouds toClouds(
    const std::vector<sensor_msgs::msg::PointCloud2>& scans, const int intensity_threshold) {
    Clouds clouds;

    for (const auto& scan : scans) {
        clouds.push_back(toCloud(scan, intensity_threshold));
    }

    return clouds;
}

namespace msg {

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 result;

    result.header.frame_id = frame_id;
    result.height = 1;
    result.width = cloud.cols();
    result.fields.resize(4);

    result.fields[0].name = "x";
    result.fields[0].offset = 0;
    result.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[0].count = 1;

    result.fields[1].name = "y";
    result.fields[1].offset = 4;
    result.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[1].count = 1;

    result.fields[2].name = "z";
    result.fields[2].offset = 8;
    result.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[2].count = 1;

    result.fields[3].name = "w";
    result.fields[3].offset = 12;
    result.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[3].count = 1;

    result.point_step = 16;
    result.row_step = result.point_step * result.width;
    result.is_bigendian = isBigendian();
    result.is_dense = true;

    result.data.resize(result.row_step * result.height);
    std::memcpy(result.data.data(), cloud.data(), result.data.size());

    return result;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(
    const Cloud& cloud, const Eigen::VectorXf& weights, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 result;

    result.header.frame_id = frame_id;
    result.height = 1;
    result.width = cloud.cols();
    result.fields.resize(5);

    result.fields[0].name = "x";
    result.fields[0].offset = 0;
    result.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[0].count = 1;

    result.fields[1].name = "y";
    result.fields[1].offset = 4;
    result.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[1].count = 1;

    result.fields[2].name = "z";
    result.fields[2].offset = 8;
    result.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[2].count = 1;

    result.fields[3].name = "w";
    result.fields[3].offset = 12;
    result.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[3].count = 1;

    result.fields[4].name = "weights";
    result.fields[4].offset = 16;
    result.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[4].count = 1;

    result.point_step = 20;
    result.row_step = result.point_step * result.width;
    result.is_bigendian = isBigendian();
    result.is_dense = true;

    result.data.resize(result.row_step * result.height);

    for (size_t i = 0; i < cloud.cols(); i++) {
        std::memcpy(&result.data[i * result.point_step], &cloud(0, i), sizeof(float));
        std::memcpy(&result.data[i * result.point_step + 4], &cloud(1, i), sizeof(float));
        std::memcpy(&result.data[i * result.point_step + 8], &cloud(2, i), sizeof(float));
        std::memcpy(&result.data[i * result.point_step + 12], &cloud(3, i), sizeof(float));
        std::memcpy(&result.data[i * result.point_step + 16], &weights(i), sizeof(float));
    }

    return result;
}

}  // namespace msg

}  // namespace truck::lidar_map
