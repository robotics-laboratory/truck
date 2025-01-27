#include "geom/common.h"
#include "geom/pose.h"

#include "common/exception.h"

#include <iomanip>

namespace truck::geom {

std::ostream& operator<<(std::ostream& out, const Pose& pose) noexcept {
    return out << std::fixed << std::setprecision(2) << "[" << pose.pos.x << ", " << pose.pos.y
               << ", " << Angle(pose.dir) << "]";
}

Pose interpolate(const Pose& a, const Pose& b, double t) noexcept {
    VERIFY(0.0 <= t && t <= 1.0);

    const Vec2 pos = interpolate(a.pos, b.pos, t);
    const AngleVec2 dir = interpolate(a.dir, b.dir, t);
    return {pos, dir};
}

template<>
Pose lerp<Pose>(const Pose& from, const Pose& to, double t) {
    return interpolate(from, to, t);
}

}  // namespace truck::geom
