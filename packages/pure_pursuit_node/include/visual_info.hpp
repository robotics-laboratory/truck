#pragma once

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geom/vector.hpp"

struct VisualInfo {
    visualization_msgs::msg::MarkerArray arc;

    void addPoint(geom::Vec2d pos, double size, double r, double g, double b) {
        using visualization_msgs::msg::Marker;
        Marker mark;
        mark.type = Marker::SPHERE;
        mark.pose.position.x = pos.x;
        mark.pose.position.y = pos.y;
        mark.color.a = 1;
        mark.color.r = r;
        mark.color.g = g;
        mark.color.b = b;
        mark.scale.x = size;
        mark.scale.y = size;
        mark.scale.z = size;
        mark.id = arc.markers.size();
        arc.markers.emplace_back(mark);
    }
};
