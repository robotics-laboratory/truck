#pragma once

#include "geom/complex_polygon.h"

#include <visualization_msgs/msg/marker.hpp>

namespace truck::visualization {

namespace msg {

visualization_msgs::msg::Marker toMarker(
    const geom::ComplexPolygon& complex_polygon, const std::string& frame_id,
    const std::vector<float>& rgba_color, float z_lev = 1.0);

}  // namespace msg
}  // namespace truck::visualization
