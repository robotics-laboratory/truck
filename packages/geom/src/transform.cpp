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

namespace msg {

geometry_msgs::msg::Point toPoint(const Vec2& v) {
    geometry_msgs::msg::Point msg;

    msg.x = v.x;
    msg.y = v.y;

    return msg;
}

}  // namespace msg

}  // namespace truck::geom