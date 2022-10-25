#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

Vec2 toPosition(const geometry_msgs::msg::Point& p);

Angle toYawAngle(const geometry_msgs::msg::Quaternion& q);

Pose toPose(const geometry_msgs::msg::Pose& p);

namespace msg {

geometry_msgs::msg::Point toPoint(const Vec2& v);

} // namespace msg

} // namepsace truck::geom