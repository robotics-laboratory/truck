#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

Vec2 toPosition(const geometry_msgs::msg::Point& p);

Angle toYawAngle(const geometry_msgs::msg::Quaternion& q);

Pose toPose(const geometry_msgs::msg::Pose& p);

Pose toPose(const nav_msgs::msg::Odometry& odom);

namespace msg {

geometry_msgs::msg::Quaternion toQuaternion(const Angle& a);

geometry_msgs::msg::Point toQauternion(const Vec2& v);

geometry_msgs::msg::Point toPoint(const Vec2& v);

geometry_msgs::msg::Pose toPose(const Pose& p);

geometry_msgs::msg::PoseStamped toPoseStamped(const std_msgs::msg::Header& header, const Pose& p);

} // namespace msg
} // namepsace truck::geom