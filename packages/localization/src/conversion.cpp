#include "localization/conversion.h"

#include <pcl_conversions/pcl_conversions.h>

#include <common/math.h>

namespace truck::localization {

DataPoints toDataPoints(const sensor_msgs::msg::PointCloud2& msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    DataPoints::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("z", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    const DataPoints::Labels descriptor_labels;

    DataPoints result(feature_labels, descriptor_labels, pcl_cloud.size());

    for (size_t i = 0; i < pcl_cloud.size(); ++i) {
        result.features(0, i) = pcl_cloud[i].x;
        result.features(1, i) = pcl_cloud[i].y;
        result.features(2, i) = pcl_cloud[i].z;
        result.features(3, i) = 1.0f;
    }

    return result;
}

namespace {

constexpr bool isBigendian() { return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__; }

}  // namespace

sensor_msgs::msg::PointCloud2 toPointCloud2(
    const std_msgs::msg::Header& header, const DataPoints& data_points) {
    sensor_msgs::msg::PointCloud2 result;

    result.header = header;
    result.height = 1;
    result.width = data_points.features.cols();
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
    std::memcpy(result.data.data(), data_points.features.data(), result.data.size());

    return result;
}

}  // namespace truck::localization
