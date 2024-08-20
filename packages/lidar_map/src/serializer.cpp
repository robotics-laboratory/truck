#include "lidar_map/serializer.h"

#include "common/exception.h"
#include "geom/msg.h"
#include "lidar_map/conversion.h"
#include "visualization/msg.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

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

std::vector<nav_msgs::msg::Odometry> loadOdomTopic(
    const std::string& mcap_path, const std::string& odom_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    std::vector<nav_msgs::msg::Odometry> data;

    while (true) {
        auto msg = readNextMessage<nav_msgs::msg::Odometry>(reader, odom_topic);

        if (!msg.has_value()) {
            break;
        }

        data.push_back(*msg);
    }

    return data;
}

std::vector<sensor_msgs::msg::LaserScan> loadLaserScanTopic(
    const std::string& mcap_path, const std::string& laser_scan_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    std::vector<sensor_msgs::msg::LaserScan> data;

    while (true) {
        auto msg = readNextMessage<sensor_msgs::msg::LaserScan>(reader, laser_scan_topic);

        if (!msg.has_value()) {
            break;
        }

        data.push_back(*msg);
    }

    return data;
}

namespace {

bool operator<(const std_msgs::msg::Header& a, const std_msgs::msg::Header& b) {
    if (a.stamp.sec < b.stamp.sec) {
        return true;
    }

    if (a.stamp.sec > b.stamp.sec) {
        return false;
    }

    return a.stamp.nanosec < b.stamp.nanosec;
}

}  // namespace

void syncOdomWithCloud(
    std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    std::vector<sensor_msgs::msg::LaserScan>& laser_scan_msgs) {
    VERIFY(!odom_msgs.empty());
    VERIFY(!laser_scan_msgs.empty());

    const size_t odom_count = odom_msgs.size();
    const size_t laser_scan_count = laser_scan_msgs.size();

    size_t odom_id = 0;
    size_t laser_scan_id = 0;

    std::vector<nav_msgs::msg::Odometry> odom_msgs_synced;
    std::vector<sensor_msgs::msg::LaserScan> laser_scan_msgs_synced;

    while (laser_scan_id < laser_scan_count) {
        while (odom_id < odom_count
               && odom_msgs[odom_id].header < laser_scan_msgs[laser_scan_id].header) {
            odom_id++;
        }

        if (odom_id >= odom_count) {
            break;
        }

        odom_msgs_synced.push_back(odom_msgs[odom_id]);
        laser_scan_msgs_synced.push_back(laser_scan_msgs[laser_scan_id]);

        laser_scan_id++;
    }

    odom_msgs = std::move(odom_msgs_synced);
    laser_scan_msgs = std::move(laser_scan_msgs_synced);
}

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name) {
    const rclcpp::Time time;
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(mcap_path);
    writer->write(msg::toPointCloud2(cloud), cloud_topic_name, time);
}

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name,
    const geom::ComplexPolygon& map, const std::string& map_topic_name) {
    const rclcpp::Time time;
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(mcap_path);
    writer->write(msg::toPointCloud2(cloud), cloud_topic_name, time);
    writer->write(visualization::msg::toMarker(map), map_topic_name, time);
}

}  // namespace truck::lidar_map
