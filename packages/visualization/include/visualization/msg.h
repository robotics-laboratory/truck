#pragma once

#include "geom/complex_polygon.h"

#include <visualization_msgs/msg/marker.hpp>

namespace truck::visualization {

namespace msg {

visualization_msgs::msg::Marker toMarker(
    const geom::ComplexPolygon& complex_polygon, std::string frame_id = "world",
    std::vector<float> rgba_color = {0.3, 0.3, 0.3, 1.0}, float z_lev = 1.0);

}  // namespace msg
}  // namespace truck::visualization
