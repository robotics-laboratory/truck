#include "geom/distance.h"

namespace geom {

double DistanceSq(const Point2& a, const Point2& b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;

    return dx*dx + dy*dy;
}

} // namespace geom