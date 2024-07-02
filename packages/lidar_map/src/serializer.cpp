#include "lidar_map/serializer.h"
#include "lidar_map/conversion.h"

#include "geom/msg.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace truck::lidar_map {

const std::string RIDE_BAGS_FOLDER_PATH = "/truck/packages/lidar_map/test/bags/rides/";
const std::string CLOUD_BAGS_FOLDER_PATH = "/truck/packages/lidar_map/test/bags/clouds/";

namespace {

template<typename T>
std::optional<T> readNextMessage(
    const std::string& topic_name,
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader>& reader) {
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

Serializer::Serializer(const SerializerParams& params) : params_(params) {
    reader_ = getSequentialReader();
}

std::unique_ptr<rosbag2_cpp::readers::SequentialReader> Serializer::getSequentialReader() {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = RIDE_BAGS_FOLDER_PATH + params_.bag_name.ride + ".mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader->open(storage_options, converter_options);
    return reader;
}

std::optional<std::pair<geom::Pose, Cloud>> Serializer::readNextMessages() {
    auto laser_scan =
        readNextMessage<sensor_msgs::msg::LaserScan>(params_.topic.point_cloud, reader_);
    auto odom = readNextMessage<nav_msgs::msg::Odometry>(params_.topic.odom, reader_);

    if (!laser_scan.has_value() || !odom.has_value()) {
        return std::nullopt;
    }

    return std::make_pair(geom::toPose(*odom), toCloud(*laser_scan));
}

std::pair<geom::Poses, Clouds> Serializer::deserializeMCAP() {
    geom::Poses poses;
    Clouds clouds;

    while (true) {
        auto messages = readNextMessages();

        if (!messages.has_value()) {
            break;
        }

        const auto [pose, cloud] = *messages;
        poses.push_back(pose);
        clouds.push_back(cloud);
    }

    return std::make_pair(poses, clouds);
}

void Serializer::serializeToMCAP(
    const Cloud& cloud, std::string cloud_topic, std::optional<geom::ComplexPolygon> map,
    std::string map_topic) {
    const auto get_timestamp = []() {
        rclcpp::Clock clock;
        return clock.now();
    };

    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(CLOUD_BAGS_FOLDER_PATH + params_.bag_name.cloud);

    writer->write(toPointCloud2(cloud), cloud_topic, get_timestamp());

    if (map.has_value()) {
        writer->write(toMarker(*map), map_topic, get_timestamp());
    }
}

}  // namespace truck::lidar_map
