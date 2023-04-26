#include "geom/pose.h"

#include "common/exception.h"

#include <iomanip>

namespace truck::geom {

std::ostream& operator<<(std::ostream& out, const Pose& pose) noexcept {
    return out << std::fixed << std::setprecision(2)
        << "["
        << pose.pos.x << ", " << pose.pos.y << ", "
        << pose.dir.angle()
        << "]";
}

Pose interpolate(const Pose& a, const Pose& b, double t) noexcept {
    VERIFY(0.0 <= t && t <= 1.0);

    return Pose{
        .pos = interpolate(a.pos, b.pos, t),
        .dir = t * a.dir + (1 - t) * b.dir,
    };
}

}  // namespace truck::geom