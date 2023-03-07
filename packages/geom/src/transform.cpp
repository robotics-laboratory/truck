#include "geom/transform.h"

namespace truck::geom {

Vec2 Transform::apply(const Vec2& v) const {
    return translation_ + v.rotate(rotation_);
}

Pose Transform::apply(const Pose& p) const {
    return Pose{apply(p.pos), p.dir.rotate(rotation_)};
}

Transform Transform::inv() const {
    return Transform(-translation_, -rotation_);
}

} // namespace truck::geom