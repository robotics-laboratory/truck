#include "geom/common.h"
#include "geom/vector.h"

#include "common/exception.h"

#include <iomanip>

namespace truck::geom {

bool equal(const Vec2& a, const Vec2& b, double eps) noexcept {
    return equal(a.x, b.x, eps) && equal(a.y, b.y, eps);
}

geom::Angle angleBetween(const Vec2& a, const Vec2& b) noexcept {
    return geom::Angle::fromRadians(std::atan2(cross(a, b), dot(a, b)));
}

Vec2 interpolate(const Vec2& a, const Vec2& b, double t) noexcept {
    VERIFY(0 <= t && t <= 1);
    return a + t * (b - a);
}

std::ostream& operator<<(std::ostream& out, const Vec2& v) {
    return out << std::fixed << std::setprecision(3) << "[" << v.x << ", " << v.y << "]";
}

}  // namespace truck::geom
