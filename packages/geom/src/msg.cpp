#include "geom/msg.h"

namespace truck::geom {

Vec2 toVec2(const geometry_msgs::msg::Point& p) { return {p.x, p.y}; }

Vec2 toVec2(const geometry_msgs::msg::PointStamped& p) { return toVec2(p.point); }

Vec2 toVec2(const geometry_msgs::msg::Pose& p) { return toVec2(p.position); }

Vec2 toVec2(const geometry_msgs::msg::PoseStamped& p) { return toVec2(p.pose); }

Vec2 toVec2(const nav_msgs::msg::Odometry& odom) { return toVec2(odom.pose.pose); }

Angle toAngle(const geometry_msgs::msg::Quaternion& q) {
    return Angle::fromRadians(std::copysign(2 * std::acos(q.w), q.z));
}

Angle toYawAngle(const geometry_msgs::msg::Quaternion& q) { return toAngle(q); }

AngleVec2 toYawDir(const geometry_msgs::msg::Quaternion& q) { return toYawAngle(q); }

Pose toPose(const geometry_msgs::msg::Pose& p) { return Pose{toVec2(p), toYawDir(p.orientation)}; }

Pose toPose(const nav_msgs::msg::Odometry& odom) { return toPose(odom.pose.pose); }

Vec2 toVec2(const geometry_msgs::msg::Vector3& v) { return Vec2(v.x, v.y); }

Transform toTransform(const geometry_msgs::msg::Transform& t) {
    return Transform(toVec2(t.translation), toAngle(t.rotation));
}

Transform toTransform(const geometry_msgs::msg::TransformStamped& t) {
    return toTransform(t.transform);
}

Localization toLocalization(const nav_msgs::msg::Odometry& odom) {
    return Localization{.pose = toPose(odom), .velocity = toVec2(odom.twist.twist.linear).len()};
}

namespace msg {

geometry_msgs::msg::Point toPoint(const geom::Vec2& v) {
    geometry_msgs::msg::Point msg;

    msg.x = v.x;
    msg.y = v.y;
    msg.z = 0;

    return msg;
}

geometry_msgs::msg::Point toPoint(const geom::Pose& p) { return toPoint(p.pos); }

geometry_msgs::msg::Quaternion toQuaternion(const geom::Angle& a) {
    geometry_msgs::msg::Quaternion msg;

    msg.w = std::cos(a.radians() / 2);
    msg.z = std::sin(a.radians() / 2);

    return msg;
}

geometry_msgs::msg::Quaternion toQuaternion(const geom::AngleVec2& v) {
    return toQuaternion(Angle(v));
}

geometry_msgs::msg::Pose toPose(const geom::Pose& p) {
    geometry_msgs::msg::Pose msg;

    msg.position = toPoint(p.pos);
    msg.orientation = toQuaternion(p.dir);

    return msg;
}

geometry_msgs::msg::PoseStamped toPoseStamped(
    const std_msgs::msg::Header& header, const geom::Pose& p) {
    geometry_msgs::msg::PoseStamped msg;

    msg.header = header;
    msg.pose = toPose(p);

    return msg;
}

}  // namespace msg
}  // namespace truck::geom
