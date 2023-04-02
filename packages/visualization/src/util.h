#pragma once

#include "geom/pose.h"

#include <visualization_msgs/marker.hpp>

namespace truck::visualization {

visualization_msgs::msg::Marker triangulate(
    const std::vector<geom::Pose>& poses, double width, double z);

} // namespace truck::visualization