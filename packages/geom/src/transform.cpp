#include <utility>

#include "geom/transform.h"

namespace truck::geom {

Transform::Transform(Vec2 t, Angle a) : translation_(t), rotation_(a) {}

Transform::Transform(Vec2 t, AngleVec2 r) : translation_(t), rotation_(std::move(r)) {}

namespace {

geom::Vec2 toVector(const tf2::Vector3& v) { return {v.x(), v.y()}; }

}  // namespace

geom::Angle toAngle(const tf2::Quaternion& q) {
    return geom::Angle::fromRadians(std::copysign(2 * std::acos(q.w()), q.z()));
}

Transform::Transform(const tf2::Transform& tf) :
    translation_(toVector(tf.getOrigin())), rotation_(toAngle(tf.getRotation())) {}

Vec2 Transform::apply(Vec2 v) const { return translation_ + rotation_.apply(v); }

Pose Transform::apply(Pose p) const { return Pose{apply(p.pos), rotation_.apply(p.dir)}; }

Vec2 Transform::operator()(Vec2 v) const { return apply(v); }

Pose Transform::operator()(Pose p) const { return apply(p); }

Transform Transform::inv() const {
    auto r_inv = rotation_.inv();
    return {r_inv.apply(-translation_), r_inv};
}

std::ostream& operator<<(std::ostream& out, const Transform& transform) noexcept {
    return out << "TF[t=" << transform.t() << ", R=" << transform.r().angle() << "]";
}

}  // namespace truck::geom
