#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"
#include "geom/complex_polygon.h"

#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace truck::lidar_map {

struct SerializerParams {
    struct Topic {
        std::string odom;
        std::string point_cloud;
    } topic;

    struct BagName {
        std::string ride;
        std::string cloud;
    } bag_name;
};

class Serializer {
  public:
    Serializer(const SerializerParams& params);

    std::pair<geom::Poses, Clouds> deserializeMCAP();
    void serializeToMCAP(
        const Cloud& cloud, std::string cloud_topic = "/cloud",
        std::optional<geom::ComplexPolygon> map = std::nullopt, std::string map_topic = "/map");

  private:
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> getSequentialReader();

    std::optional<std::pair<geom::Pose, Cloud>> readNextMessages();

    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_ = nullptr;

    SerializerParams params_;
};

}  // namespace truck::lidar_map
