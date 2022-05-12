#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include "transform.hpp"

namespace rosaruco {

visualization_msgs::msg::Marker get_marker(const Transform &t, double size);

visualization_msgs::msg::Marker get_label(int id, const tf2::Vector3 &p, double size);

void add_labeled_marker(std::vector<visualization_msgs::msg::Marker> &markers, const Transform &t, int id, double size);

} // namespace rosaruco
