#include "geom/arc.h"
#include "common/math.h"

#include <boost/assert.hpp>

namespace truck::geom {

Arc Arc::byTwoPointsAndTangent(const Vec2& start, const Vec2& end, const Vec2& tangent) noexcept {
    const Vec2 chord_half = (end - start) / 2;
    const double curvature = cross(tangent, chord_half) / chord_half.lenSq();

    constexpr double eps = 1e-3;
    if (std::abs(curvature) < eps) {
        return Arc{start, end, 0};
    }

    return Arc{start, end, curvature};
}

} // namespace truck::geom