#pragma once

#include "lidar_map/builder.h"
#include "lidar_map/common.h"
#include "geom/complex_polygon.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace truck::lidar_map {

std::vector<nav_msgs::msg::Odometry> loadOdomTopic(
    const std::string& mcap_path, const std::string& odom_topic);

std::vector<sensor_msgs::msg::PointCloud2> loadLaserScanTopic(
    const std::string& mcap_path, const std::string& laser_scan_topic);

std::pair<std::vector<nav_msgs::msg::Odometry>, std::vector<sensor_msgs::msg::PointCloud2>>
syncOdomWithCloud(
    const std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    const std::vector<sensor_msgs::msg::PointCloud2>& laser_scan_msgs);

void writePoseGraphInfoToJSON(
    const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration);

class BagWriter {
  public:
    BagWriter(const std::string& mcap_path, const std::string& frame_name, double freqency);

    static void writeCloud(
        const std::string& mcap_path, const Cloud& cloud, const std::string& frame_name,
        const std::string& topic_name);

    void addNormals(const std::string& mcap_path, const CloudWithAttributes& cloud_with_attributes);
    
    void addOptimizationStep(
        const geom::Poses& poses, const std::string& poses_topic_name, const Cloud& merged_clouds,
        const std::string& merged_clouds_topic_name);

  private:
    void addPoses(const geom::Poses& poses, const std::string& topic_name);
    void addMergedClouds(const Cloud& merged_clouds, const std::string& topic_name);

    size_t id_ = 0;
    const std::string frame_name_;
    const double freqency_;

    rosbag2_cpp::Writer writer_;
};

Cloud loadPCD(const std::string& pcd_path);

void writeToPCD(const std::string& pcd_path, const Cloud& cloud);

}  // namespace truck::lidar_map
