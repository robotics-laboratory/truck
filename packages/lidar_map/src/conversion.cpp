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


float getIntensity(const sensor_msgs::msg::PointCloud2& point_cloud, size_t point_index) {
    int intensity_offset = -1;
    size_t i = 0;

    while (i < point_cloud.fields.size() && intensity_offset == -1) {
        if (point_cloud.fields[i].name == "intensity") {
            intensity_offset = point_cloud.fields[i].offset;
        }
        ++i;
    }

    if (intensity_offset == -1) {
        throw std::runtime_error("Field 'intensity' not found in PointCloud2!");
    }

    if (point_index >= point_cloud.width * point_cloud.height) {
        throw std::out_of_range("Point index is out of range!");
    }

    size_t byte_index = point_index * point_cloud.point_step + intensity_offset;
    return *reinterpret_cast<const float*>(&point_cloud.data[byte_index]);
}


Cloud toCloud(const sensor_msgs::msg::PointCloud2& point_cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud2;

    pcl::fromROSMsg(point_cloud, pcl_cloud2);
    pcl_conversions::toPCL(point_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    
    std::vector<Eigen::Vector4f> filtered_points;
    const size_t point_count = pcl_cloud.size();
    for (size_t j = 0; j < point_count; ++j) {
        const auto& point = pcl_cloud.points[j];
        float intensity = pcl_cloud2.points[j].intensity;
        if (intensity > 10) {
            filtered_points.emplace_back(point.x, point.y, point.z, 1.0f);
        }
    }

    const size_t point_count2 = filtered_points.size();
    Cloud cloud(4, point_count2);
    for (size_t j = 0; j < point_count2; ++j) {
        cloud(0, j) = filtered_points[j][0];
        cloud(1, j) = filtered_points[j][1];
        cloud(2, j) = filtered_points[j][2];
        cloud(3, j) = 1.0;
    }


    return cloud;
}

Clouds toClouds(const std::vector<sensor_msgs::msg::PointCloud2>& scans) {
    Clouds clouds;

    for (const auto& scan : scans) {
        clouds.push_back(toCloud(scan));
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

}  // namespace msg

}  // namespace truck::lidar_map
