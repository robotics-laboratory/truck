#include "lidar_map/conversion.h"

#include <common/math.h>

namespace truck::lidar_map {

namespace {

constexpr bool isBigendian() { return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__; }

}  // namespace

Cloud toCloud(const sensor_msgs::msg::LaserScan& scan) {
    Limits range_limit{scan.range_min, scan.range_max};

    auto is_valid = [&](double range) { return std::isfinite(range) && range_limit.isMet(range); };

    const size_t point_count = std::count_if(
        scan.ranges.begin(), scan.ranges.end(), [&](float range) { return is_valid(range); });

    Cloud cloud(3, point_count);

    for (size_t i = 0, j = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];

        if (!is_valid(range)) {
            continue;
        }

        const double angle = scan.angle_min + i * scan.angle_increment;

        cloud(0, j) = range * std::cos(angle);
        cloud(1, j) = range * std::sin(angle);
        cloud(2, j) = 1.0;
        j++;
    }

    return cloud;
}

Clouds toClouds(const std::vector<sensor_msgs::msg::LaserScan>& scans) {
    Clouds clouds;

    for (const auto& scan : scans) {
        clouds.push_back(toCloud(scan));
    }

    return clouds;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2 result;

    result.header.frame_id = frame_id;
    result.height = 1;
    result.width = cloud.cols();
    result.fields.resize(3);

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

    result.point_step = 12;
    result.row_step = result.point_step * result.width;
    result.is_bigendian = isBigendian();
    result.is_dense = true;

    result.data.resize(result.row_step * result.height);
    std::memcpy(result.data.data(), cloud.data(), result.data.size());

    return result;
}

}  // namespace truck::lidar_map
