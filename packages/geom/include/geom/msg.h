#pragma once

#include "geom/pose.h"
#include "geom/transform.h"
#include "geom/vector.h"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace truck::geom {

Vec2 toVec2(const geometry_msgs::msg::Point& p);

Vec2 toVec2(const geometry_msgs::msg::PointStamped& p);

Vec2 toVec2(const geometry_msgs::msg::PoseStamped& p);

Vec2 toVec2(const geometry_msgs::msg::Pose& p);

Vec2 toVec2(const nav_msgs::msg::Odometry& odom);

Angle toAngle(const geometry_msgs::msg::Quaternion& q);

Angle toYawAngle(const geometry_msgs::msg::Quaternion& q);

Vec2 toYawDir(const geometry_msgs::msg::Quaternion& q);

Pose toPose(const geometry_msgs::msg::Pose& p);

Pose toPose(const nav_msgs::msg::Odometry& odom);

Vec2 toVec2(const geometry_msgs::msg::Vector3& v);

Transform toTransform(const geometry_msgs::msg::Transform& t);

Transform toTransform(const geometry_msgs::msg::TransformStamped& t);

namespace msg {

geometry_msgs::msg::Quaternion toQuaternion(const Angle& a);

geometry_msgs::msg::Quaternion toQuaternion(const geom::Vec2& dir);

geometry_msgs::msg::Point toPoint(const Vec2& v);

geometry_msgs::msg::Point toPoint(const Pose& p);

geometry_msgs::msg::Pose toPose(const Pose& p);

geometry_msgs::msg::PoseStamped toPoseStamped(const std_msgs::msg::Header& header, const Pose& p);

} // namespace msg
} // namepsace truck::geom