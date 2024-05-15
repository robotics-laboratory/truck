#include "geom/common.h"
#include "geom/vector3.h"

#include "common/exception.h"

#include <iomanip>

namespace truck::geom {

bool equal(const Vec3& a, const Vec3& b, double eps) noexcept {
    return equal(a.x, b.x, eps) && equal(a.y, b.y, eps) && equal(a.z, b.z, eps);
}

Vec3 interpolate(const Vec3& a, const Vec3& b, double t) noexcept {
    VERIFY(0 <= t && t <= 1);
    return a + t * (b - a);
}

std::ostream& operator<<(std::ostream& out, const Vec3& v) {
    return out << std::fixed << std::setprecision(3) << "[" << v.x << ", " << v.y << ", " << v.z
               << "]";
}

}  // namespace truck::geom
