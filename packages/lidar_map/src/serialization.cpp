#include "lidar_map/serialization.h"

#include "common/exception.h"
#include "geom/msg.h"
#include "lidar_map/conversion.h"

#include <cmath>
#include <rosbag2_cpp/reader.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <optional>

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

std::vector<sensor_msgs::msg::PointCloud2> loadLaserScanTopic(
    const std::string& mcap_path, const std::string& laser_scan_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    std::vector<sensor_msgs::msg::PointCloud2> data;

    while (true) {
        auto msg = readNextMessage<sensor_msgs::msg::PointCloud2>(reader, laser_scan_topic);

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

std::pair<std::vector<nav_msgs::msg::Odometry>, std::vector<sensor_msgs::msg::PointCloud2>>
syncOdomWithCloud(
    const std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    const std::vector<sensor_msgs::msg::PointCloud2>& laser_scan_msgs) {
    VERIFY(!odom_msgs.empty());
    VERIFY(!laser_scan_msgs.empty());

    const size_t odom_count = odom_msgs.size();
    const size_t laser_scan_count = laser_scan_msgs.size();

    size_t odom_id = 0;
    size_t laser_scan_id = 0;

    std::vector<nav_msgs::msg::Odometry> odom_msgs_synced;
    std::vector<sensor_msgs::msg::PointCloud2> laser_scan_msgs_synced;

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

/**
 * Writing information about icp edges to a json file
 */
void writePoseGraphInfoToJSON(
    const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration) {
    nlohmann::json json_data;

    std::ifstream input_file(json_path);
    if (input_file.is_open()) {
        input_file >> json_data;
        input_file.close();
    }

    nlohmann::json current_iteration_data;

    for (const auto& vertex : pose_graph_info.poses) {
        nlohmann::json vertex_json;
        vertex_json["id"] = vertex.id;
        vertex_json["x"] = vertex.pose.pos.x;
        vertex_json["y"] = vertex.pose.pos.y;
        vertex_json["theta"] = vertex.pose.dir.angle().radians();

        current_iteration_data["vertices"].push_back(vertex_json);
    }

    for (const auto& edge : pose_graph_info.edges) {
        nlohmann::json edge_json;
        edge_json["from_edge"] = edge.from_edge;
        edge_json["to_edge"] = edge.to_edge;
        edge_json["error_val"] = edge.error_val;
        edge_json["type"] = edge.type;

        current_iteration_data["edges"].push_back(edge_json);
    }
    std::string formatted_iteration;

    if (iteration < 10) {
        json_data["0" + std::to_string(iteration)] = current_iteration_data;
    } else {
        json_data[std::to_string(iteration)] = current_iteration_data;
    }

    std::ofstream output_file(json_path);
    if (output_file.is_open()) {
        output_file << json_data.dump(4);
        output_file.close();
    } else {
        std::cerr << "Error when opening a file for writing: " << json_path << std::endl;
    }
}

BagWriter::BagWriter(const std::string& mcap_path, const std::string& frame_name, double freqency) :
    frame_name_(frame_name), freqency_(freqency) {
    if (!mcap_path.empty()) {
        writer_.open(mcap_path);
    }
}

namespace {

rclcpp::Time getTime(double seconds = 0.0) {
    auto nanoseconds = (seconds - static_cast<int32_t>(seconds)) * 1e9;
    return {static_cast<int32_t>(seconds), static_cast<uint32_t>(nanoseconds)};
}

}  // namespace

void BagWriter::writeCloud(
    const std::string& mcap_path, const Cloud& cloud, const std::string& frame_name,
    const std::string& topic_name) {
    rosbag2_cpp::Writer writer;
    writer.open(mcap_path);
    writer.write(msg::toPointCloud2(cloud, frame_name), topic_name, getTime());
}

void BagWriter::addOptimizationStep(
    const geom::Poses& poses, const std::string& poses_topic_name, const Cloud& merged_clouds,
    const std::string& merged_clouds_topic_name) {
    addPoses(poses, poses_topic_name);
    addMergedClouds(merged_clouds, merged_clouds_topic_name);
    id_++;
}
void BagWriter::addNormals(const std::vector<DataPoints>& normals, const std::string& topic_name) {
    auto get_color = [](double a = 1.0, double r = 1.0, double g = 0.0, double b = 0.0) {
        std_msgs::msg::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    };

    visualization_msgs::msg::MarkerArray msg_array;

    for (size_t i = 0; i < 1; i++) {
        const DataPoints& dp = normals[i];
        for (size_t j = 0; j < dp.features.cols(); j++) {
            double A = dp.features(0, j);
            double B = dp.features(1, j);
            double C = dp.features(2, j);
            double D = dp.features(4, j);
            double E = dp.features(5, j);
            double F = dp.features(6, j);
            
            visualization_msgs::msg::Marker msg_;
            msg_.header.frame_id = frame_name_;
            msg_.id = i * dp.features.cols() + j;
            msg_.type = visualization_msgs::msg::Marker::ARROW;
            msg_.action = visualization_msgs::msg::Marker::ADD;
            msg_.color = get_color();

            msg_.pose.position.x = A;
            msg_.pose.position.y = B;
            msg_.pose.position.z = C;

            double dir_x = D - A;
            double dir_y = E - B;
            double dir_z = F - C;

            msg_.scale.x = 2;
            msg_.scale.y = 0.1;
            msg_.scale.z = 0.1;

            double yaw = std::atan2(dir_y, dir_x);

            msg_.pose.orientation = geom::msg::toQuaternion(truck::geom::Angle(yaw));

            msg_array.markers.push_back(msg_);
            std::cout << A << ' ' << B << ' ' << C << " yaw=" << yaw << '\n';
        }
    }
    std::cout << "Количество сообщений в msg_array.markers: " << msg_array.markers.size() << '\n';

    writer_.write(msg_array, topic_name, getTime(id_ * freqency_));
    id_++;
}

void BagWriter::addPoses(const geom::Poses& poses, const std::string& topic_name) {
    auto get_color = [](double a = 1.0, double r = 0.0, double g = 0.0, double b = 1.0) {
        std_msgs::msg::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    };

    auto get_scale = [](double x = 1.2, double y = 0.2, double z = 0.2) {
        geometry_msgs::msg::Vector3 scale;
        scale.x = x;
        scale.y = y;
        scale.z = z;
        return scale;
    };

    visualization_msgs::msg::MarkerArray msg_array;

    for (size_t i = 0; i < poses.size(); i++) {
        const geom::Pose& pose = poses[i];

        visualization_msgs::msg::Marker msg;
        msg.header.frame_id = frame_name_;
        msg.id = i;
        msg.type = visualization_msgs::msg::Marker::ARROW;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.color = get_color();
        msg.pose.position.x = pose.pos.x;
        msg.pose.position.y = pose.pos.y;
        msg.pose.orientation = geom::msg::toQuaternion(pose.dir);
        msg.scale = get_scale();

        msg_array.markers.push_back(msg);
    }

    writer_.write(msg_array, topic_name, getTime(id_ * freqency_));
}

void BagWriter::addMergedClouds(const Cloud& merged_clouds, const std::string& topic_name) {
    writer_.write(
        msg::toPointCloud2(merged_clouds, frame_name_), topic_name, getTime(id_ * freqency_));
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
