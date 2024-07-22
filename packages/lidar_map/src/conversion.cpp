#include "lidar_map/conversion.h"

#include <common/math.h>
#include "geom/msg.h"

namespace truck::lidar_map {

namespace {

constexpr bool isBigendian() { return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__; }

}  // namespace

Cloud toCloud(const sensor_msgs::msg::LaserScan& scan, double z_lev) {
    Limits range_limit{scan.range_min, scan.range_max};

    Cloud::Labels feature_labels;
    feature_labels.push_back(Cloud::Label("x", 1));
    feature_labels.push_back(Cloud::Label("y", 1));
    feature_labels.push_back(Cloud::Label("w", 1));

    const Cloud::Labels descriptor_labels;

    auto is_valid = [&](double range) { return std::isnormal(range) && range_limit.isMet(range); };

    const size_t point_count = std::count_if(
        scan.ranges.begin(), scan.ranges.end(), [&](float range) { return is_valid(range); });

    Cloud result(feature_labels, descriptor_labels, point_count);

    for (size_t i = 0, j = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];

        if (!is_valid(range)) {
            continue;
        }

        const double angle = scan.angle_min + i * scan.angle_increment;

        result.features(0, j) = range * std::cos(angle);
        result.features(1, j) = range * std::sin(angle);
        result.features(2, j) = z_lev;
        j++;
    }

    return result;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 result;

    result.header.frame_id = frame_id;
    result.height = 1;
    result.width = cloud.features.cols();
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
    std::memcpy(result.data.data(), cloud.features.data(), result.data.size());

    return result;
}

visualization_msgs::msg::Marker toMarker(
    const geom::ComplexPolygon& complex_polygon, std::string frame_id, double z_lev,
    std::vector<double> rgba_color) {
    const auto get_color = [&]() {
        std_msgs::msg::ColorRGBA color;
        color.r = rgba_color[0];
        color.g = rgba_color[1];
        color.b = rgba_color[2];
        color.a = rgba_color[3];
        return color;
    };

    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = frame_id;
    msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color = get_color();
    msg.pose.position.z = z_lev;

    for (const geom::Triangle& triangle : complex_polygon.triangles()) {
        msg.points.push_back(geom::msg::toPoint(triangle.p1));
        msg.points.push_back(geom::msg::toPoint(triangle.p2));
        msg.points.push_back(geom::msg::toPoint(triangle.p3));
    }

    return msg;
}

}  // namespace truck::lidar_map
