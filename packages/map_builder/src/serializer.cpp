#include "map_builder/serializer.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/math.h"

namespace truck::map_builder::serializer {

namespace {

DataPoints toDataPoints(const sensor_msgs::msg::LaserScan& scan) {
    Limits range_limit{scan.range_min, scan.range_max};

    DataPoints::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    DataPoints::Labels descriptor_labels;

    const size_t point_count = std::count_if(
        scan.ranges.begin(), scan.ranges.end(), [&](float range) {
            return std::isnormal(range) && range_limit.isMet(range);
        });

    DataPoints result(feature_labels, descriptor_labels, point_count);

    for (size_t i = 0, j = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];
        const bool valid = std::isnormal(range) && range_limit.isMet(range);

        if (!valid) {
            continue;
        }

        const double angle = scan.angle_min + i * scan.angle_increment;

        result.features(0, j) = range * std::cos(angle);
        result.features(1, j) = range * std::sin(angle);
        result.features(2, j) = 1.0f;
		j++;
    }

    return result;
}

constexpr bool isBigendian() { return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__; }

sensor_msgs::msg::PointCloud2 toPointCloud2(
    const std_msgs::msg::Header& header, const DataPoints& data_points) {
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

template<typename T>
std::optional<T> readNextMessage(const std::string& topic_name, std::unique_ptr<rosbag2_cpp::readers::SequentialReader>& reader) {
    while (reader->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

        if (msg->topic_name != topic_name) {
            continue;
        }
        
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        typename T::SharedPtr ros_msg = std::make_shared<T>();

        rclcpp::Serialization<T> serialization;
        serialization.deserialize_message(&serialized_msg, ros_msg.get());

        return *ros_msg;
    }

    return std::nullopt;
}

}  // namespace

std::unique_ptr<rosbag2_cpp::readers::SequentialReader> Serializer::getSequentialReader() {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = params_.path.bag;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader->open(storage_options, converter_options);
    return reader;
}

std::optional<OdomWithPointCloud> Serializer::readNextMessages() {
    auto laser_scan = readNextMessage<sensor_msgs::msg::LaserScan>(params_.topic.point_cloud, reader_);
    auto odom = readNextMessage<nav_msgs::msg::Odometry>(params_.topic.odom, reader_);

    if (!laser_scan.has_value() || !odom.has_value()) {
        return std::nullopt;
    }

    return OdomWithPointCloud{.odom = *odom, .data_points = toDataPoints(*laser_scan)};
}

std::vector<OdomWithPointCloud> Serializer::deserializeBag() {
    std::vector<OdomWithPointCloud> data;

    reader_ = getSequentialReader();

    while (true) {
        std::optional<OdomWithPointCloud> odom_with_point_cloud = readNextMessages();

        if (!odom_with_point_cloud.has_value()) {
            break;
        }

        data.push_back(*odom_with_point_cloud);
    }

    return data;
}

void Serializer::serializeToMCAP(const DataPoints& map) {
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(params_.path.mcap);

    std_msgs::msg::Header header;
    sensor_msgs::msg::PointCloud2 point_cloud = toPointCloud2(header, map);

    rclcpp::Clock clock;
    rclcpp::Time timestamp = clock.now();
    writer->write(point_cloud, "/lidar_map", timestamp);
}

}  // namespace truck::map_builder::serializer