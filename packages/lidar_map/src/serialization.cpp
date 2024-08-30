#include "lidar_map/serialization.h"

#include "common/exception.h"
#include "geom/msg.h"
#include "lidar_map/conversion.h"
#include "visualization/msg.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

std::pair<std::vector<nav_msgs::msg::Odometry>, std::vector<sensor_msgs::msg::LaserScan>>
syncOdomWithCloud(
    const std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    const std::vector<sensor_msgs::msg::LaserScan>& laser_scan_msgs) {
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

    return {odom_msgs_synced, laser_scan_msgs_synced};
}

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name) {
    const rclcpp::Time time;
    rosbag2_cpp::Writer writer = rosbag2_cpp::Writer();
    writer.open(mcap_path);
    writer.write(msg::toPointCloud2(cloud), cloud_topic_name, time);
}

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name,
    const geom::ComplexPolygon& map, const std::string& map_topic_name) {
    const rclcpp::Time time;
    rosbag2_cpp::Writer writer = rosbag2_cpp::Writer();
    writer.open(mcap_path);
    writer.write(msg::toPointCloud2(cloud), cloud_topic_name, time);
    writer.write(visualization::msg::toMarker(map), map_topic_name, time);
}

Cloud loadPCD(const std::string& pcd_path) {
    Cloud cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, pcl_cloud) == -1) {
        return cloud;
    }

    const size_t points_count = pcl_cloud.points.size();
    cloud = Cloud(3, points_count);

    for (size_t i = 0; i < points_count; i++) {
        cloud(0, i) = pcl_cloud.points[i].x;
        cloud(1, i) = pcl_cloud.points[i].y;
        cloud(2, i) = pcl_cloud.points[i].z;
    }

    return cloud;
}

void writeToPCD(const std::string& pcd_path, const Cloud& cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.width = cloud.cols();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    pcl_cloud.resize(pcl_cloud.width * pcl_cloud.height);

    for (size_t i = 0; i < cloud.cols(); i++) {
        pcl_cloud.points[i].x = cloud(0, i);
        pcl_cloud.points[i].y = cloud(1, i);
        pcl_cloud.points[i].z = 1.0;
    }

    pcl::io::savePCDFileASCII(pcd_path, pcl_cloud);
}

}  // namespace truck::lidar_map
