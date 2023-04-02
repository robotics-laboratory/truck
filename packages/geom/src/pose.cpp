#include "geom/pose.h"

#include "common/exception.h"

namespace truck::geom {

Pose interpolate(const Pose& a, const Pose& b, double t) noexcept {
    VERIFY(0 <= t && t <= 1);

    return Pose{
        .pos = interpolate(a.pos, b.pos, t),
        .dir = t * a.dir + (1 - t) * b.dir,
    };
}

}  // namespace truck::geom