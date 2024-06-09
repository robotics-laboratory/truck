#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"

#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace truck::lidar_map {

struct SerializerParams {
    struct Path {
        std::string bag = "/truck/packages/lidar_map/test/bag/file_1_atrium.db3";
        std::string mcap = "/truck/packages/lidar_map/test/mcap";
    } path;

    struct Topic {
        std::string odom = "/ekf/odometry/filtered";
        std::string point_cloud = "/lidar/scan";
        std::string lidar_map = "/lidar/map";
    } topic;
};

class Serializer {
  public:
    Serializer(const SerializerParams& params);

    std::pair<geom::Poses, Clouds> deserializeBag();
    void serializeToMCAP(const Clouds& clouds);

  private:
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> getSequentialReader();

    std::optional<std::pair<geom::Pose, Cloud>> readNextMessages();

    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_ = nullptr;

    SerializerParams params_;
};

}  // namespace truck::lidar_map
