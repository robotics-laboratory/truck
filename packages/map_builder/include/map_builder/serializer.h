#pragma once

#include "map_builder/map_builder.h"

#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace truck::map_builder::serializer {

struct SerializerParams {
    struct Path {
        std::string bag = "/truck/packages/icp_odometry/bags/file_1_atrium.db3";
        std::string mcap = "/truck/packages/map_builder/test/data";
    } path;

    struct Topic {
        std::string odom = "/ekf/odometry/filtered";
        std::string point_cloud = "/lidar/scan";
    } topic;
};

class Serializer {
    public:
        std::vector<OdomWithPointCloud> deserializeBag();
        void serializeToMCAP(const DataPoints& map);

    private:
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> getSequentialReader();

        std::optional<OdomWithPointCloud> readNextMessages();

        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_ = nullptr;

        SerializerParams params_;
};

}  // namespace truck::map_builder::serializer