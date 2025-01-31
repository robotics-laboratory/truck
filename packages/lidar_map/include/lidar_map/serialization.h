#pragma once

#include "lidar_map/builder.h"
#include "lidar_map/common.h"
#include "geom/complex_polygon.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace truck::lidar_map {

std::pair<std::vector<nav_msgs::msg::Odometry>, std::vector<sensor_msgs::msg::PointCloud2>>
syncOdomWithPointCloud(
    const std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    const std::vector<sensor_msgs::msg::PointCloud2>& point_cloud_msgs);

namespace reader {

Cloud readPCD(const std::string& pcd_path);

std::vector<nav_msgs::msg::Odometry> readOdomTopic(
    const std::string& mcap_path, const std::string& odom_topic);

std::vector<sensor_msgs::msg::PointCloud2> readPointCloudTopic(
    const std::string& mcap_path, const std::string& point_cloud_topic);

}  // namespace reader

namespace writer {

void writePoseGraphInfoToJSON(
    const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration);

void writeToPCD(const std::string& pcd_path, const Cloud& cloud);

struct MCAPWriterParams {
    std::string mcap_path;
    std::string poses_topic_name;
    std::string cloud_topic_name;
    std::string frame_name = "";
    double topic_frequency = 1.0;
};

class MCAPWriter {
  public:
    MCAPWriter(const MCAPWriterParams& params);

    void update();

    void writeCloud(const Cloud& cloud);

    void writePoses(const geom::Poses& poses);

    static void writeCloud(
        const std::string& mcap_path, const Cloud& cloud, const std::string& topic_name,
        std::string frame_name = "");

  private:
    size_t msg_id_ = 0;
    rosbag2_cpp::Writer writer_;

    MCAPWriterParams params_;
};

}  // namespace writer

}  // namespace truck::lidar_map
