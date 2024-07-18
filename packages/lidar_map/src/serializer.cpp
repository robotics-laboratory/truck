#include "lidar_map/serializer.h"
#include "lidar_map/conversion.h"

#include "geom/msg.h"
#include "visualization/msg.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rosbag2_transport/reader_writer_factory.hpp>

namespace truck::lidar_map {

namespace {

template<typename T>
std::optional<T> readNextMessage(
    std::unique_ptr<rosbag2_cpp::Reader>& reader, const std::string& topic_name) {
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

Clouds loadLidarScan(const std::string& mcap_path, const std::string& scan_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>(); 
    reader->open(mcap_path);

    Clouds clouds;

    while (true) {
        auto laser_scan_msg = readNextMessage<sensor_msgs::msg::LaserScan>(reader, scan_topic);

        if (!laser_scan_msg.has_value()) {
            break;
        }

        clouds.push_back(toCloud(*laser_scan_msg));
    }

    return clouds;
}

geom::Poses loadOdometry(const std::string& mcap_path, const std::string& odom_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>(); 
    reader->open(mcap_path);

    geom::Poses poses;

    while (true) {
        auto odom_msg = readNextMessage<nav_msgs::msg::Odometry>(reader, odom_topic);

        if (!odom_msg.has_value()) {
            break;
        }

        poses.push_back(geom::toPose(*odom_msg));
    }

    return poses;
}

void serializeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, std::string cloud_topic, std::optional<geom::ComplexPolygon> map,
    std::string map_topic) {
    rclcpp::Time time;

    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(mcap_path);

    writer->write(toPointCloud2(cloud, "world"), cloud_topic, time);

    if (map.has_value()) {
        writer->write(visualization::msg::toMarker(*map, "world", {0.3, 0.3, 0.3, 1.0}), map_topic, time);
    }
}

}  // namespace truck::lidar_map
