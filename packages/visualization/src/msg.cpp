#include "visualization/msg.h"
#include "visualization/color.h"

#include "geom/msg.h"

namespace truck::visualization {

namespace msg {

visualization_msgs::msg::Marker toMarker(
    const geom::ComplexPolygon& complex_polygon, const std::string& frame_id,
    const std::vector<float>& rgba_color, float z_lev) {
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = frame_id;
    msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color = color::make(rgba_color);
    msg.pose.position.z = z_lev;

    for (const geom::Triangle& triangle : complex_polygon.triangles()) {
        msg.points.push_back(geom::msg::toPoint(triangle.p1));
        msg.points.push_back(geom::msg::toPoint(triangle.p2));
        msg.points.push_back(geom::msg::toPoint(triangle.p3));
    }

    return msg;
}

}  // namespace msg
}  // namespace truck::visualization