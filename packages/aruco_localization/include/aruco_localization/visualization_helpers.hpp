#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include "transform.hpp"

namespace rosaruco {

visualization_msgs::msg::Marker getMarker(const Transform& t, double size);

visualization_msgs::msg::Marker getLabel(int id, const tf2::Vector3& p, double size);

void addLabeledMarker(
    std::vector<visualization_msgs::msg::Marker>& markers, const Transform& t, int id, double size,
    bool is_visible);

}  // namespace rosaruco
