#include "geom/transform.h"

namespace truck::geom {

Vec2 toPosition(const geometry_msgs::msg::Point& p) { return {p.x, p.y}; }

Angle toYawAngle(const geometry_msgs::msg::Quaternion& q) {
    return Angle::fromRadians(std::copysign(2 * std::acos(q.w), q.z));
}

Vec2 toYawDir(const geometry_msgs::msg::Quaternion& q) { return Vec2(toYawAngle(q)); }

Pose toPose(const geometry_msgs::msg::Pose& p) {
    return Pose {toPosition(p.position), toYawDir(p.orientation)};
}

Pose toPose(const nav_msgs::msg::Odometry& odom) {
    return toPose(odom.pose.pose);
}

namespace msg {

geometry_msgs::msg::Point toPoint(const geom::Vec2& v) {
    geometry_msgs::msg::Point msg;

    msg.x = v.x;
    msg.y = v.y;
    msg.z = 0;

    return msg;
}

geometry_msgs::msg::Quaternion toQuaternion(const geom::Angle& a) {
    geometry_msgs::msg::Quaternion msg;

    msg.w = std::cos(a.radians() / 2);
    msg.z = std::sin(a.radians() / 2);

    return msg;
}

geometry_msgs::msg::Quaternion toQuaternion(const geom::Vec2& v) {
    return toQuaternion(v.angle());
}

geometry_msgs::msg::Pose toPose(const geom::Pose& p) {
    geometry_msgs::msg::Pose msg;

    msg.position = toPoint(p.pos);
    msg.orientation = toQuaternion(p.dir);

    return msg;
}

geometry_msgs::msg::PoseStamped toPoseStamped(const std_msgs::msg::Header& header, const geom::Pose& p) {
    geometry_msgs::msg::PoseStamped msg;

    msg.header = header;
    msg.pose = toPose(p);

    return msg;
}

} // namespace msg
} // namespace truck::geom
