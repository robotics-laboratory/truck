#include "geom/transform.h"

namespace truck::geom {

Transform::Transform(const Vec2& t, Angle a) : translation_(t), rotation_(a) {}

Transform::Transform(const Vec2& t, const Vec2& r) : translation_(t), rotation_(r) {}

Vec2 Transform::apply(const Vec2& v) const { return translation_ + v.rotate(rotation_); }

Pose Transform::apply(const Pose& p) const { return Pose{apply(p.pos), p.dir.rotate(rotation_)}; }

Vec2 Transform::operator()(const Vec2& v) const { return apply(v); }

Pose Transform::operator()(const Pose& p) const { return apply(p); }

Transform Transform::inv() const {
    auto r_inv = rotation_.inv();
    return Transform(-translation_.rotate(r_inv), r_inv);
}

}  // namespace truck::geom