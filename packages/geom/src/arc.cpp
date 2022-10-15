#include "geom/arc.h"

namespace truck::geom {

Arc Arc::byTwoPointsAndTangent(const Vec2& start, const Vec2& end, const Vec2& tangent) noexcept {
    const Vec2 chord_half = (end - start) / 2;
    const double curvature = cross(tangent, chord_half) / chord_half.lenSq();

    return Arc{start, end, curvature};
}

} // namespace truck::geom