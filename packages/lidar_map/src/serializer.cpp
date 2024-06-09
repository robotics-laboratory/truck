#include "lidar_map/serializer.h"
#include "lidar_map/conversion.h"

#include "geom/msg.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace truck::lidar_map {

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
    storage_options.uri = params_.path.bag;
    storage_options.storage_id = "sqlite3";

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

std::pair<geom::Poses, Clouds> Serializer::deserializeBag() {
    geom::Poses poses;
    Clouds clouds;

    while (true) {
        auto messages = readNextMessages();

        if (!messages.has_value()) {
            break;
        }

        auto [pose, cloud] = *messages;
        poses.push_back(pose);
        clouds.push_back(cloud);
    }

    return std::make_pair(poses, clouds);
}

void Serializer::serializeToMCAP(const Clouds& clouds) {
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(params_.path.mcap);

    for (size_t i = 0; i < clouds.size(); i++) {
        sensor_msgs::msg::PointCloud2 point_cloud = toPointCloud2(clouds[i]);

        std::string topic_name = params_.topic.lidar_map + "_" + std::to_string(i);

        rclcpp::Clock clock;
        rclcpp::Time timestamp = clock.now();
        writer->write(point_cloud, topic_name, timestamp);
    }
}

}  // namespace truck::lidar_map
