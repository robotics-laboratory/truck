#include <icp_odometry/conversion.h>

namespace truck::icp_odometry {

namespace {

constexpr bool isBigendian() {
    return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
}

} // namespace

DataPoints toDataPoints(const sensor_msgs::msg::LaserScan& scan) {
    DataPoints::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    DataPoints::Labels descriptor_labels;

    const size_t point_count = std::count_if(
        scan.ranges.begin(), scan.ranges.end(),
        [](float range) {
            return std::isfinite(range);
        }
    );

    DataPoints result(feature_labels, descriptor_labels, point_count);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];
        if (!std::isfinite(range)) {
            continue;
        }

        const double angle = scan.angle_min + i * scan.angle_increment;

        result.features(0, i) = range * std::cos(angle);
        result.features(1, i) = range * std::sin(angle);
        result.features(2, i) = 1.0f;
    }

    return result;
}


sensor_msgs::msg::PointCloud2 toPointCloud2(const std_msgs::msg::Header& header, const DataPoints& data_points) {
    sensor_msgs::msg::PointCloud2 result;

    result.header = header;
    result.height = 1;
    result.width = data_points.features.cols();
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
    std::memcpy(result.data.data(), data_points.features.data(), result.data.size());

    return result;
}

} // namespace truck::icp_odometry