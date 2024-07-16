#include "geom/angle_vector.h"

#include "common/exception.h"
#include "geom/common.h"

namespace truck::geom {

bool equal(const AngleVec2& a, const AngleVec2& b, double eps) noexcept {
    return equal(a.x(), b.x(), eps) && equal(a.y(), b.y(), eps);
}

AngleVec2 interpolate(const AngleVec2& a, const AngleVec2& b, double t) noexcept {
    VERIFY(0 <= t && t <= 1);
    return {a.angle() + t * (b.angle() - a.angle())};
}

std::ostream& operator<<(std::ostream& out, const AngleVec2& a) noexcept {
    return out << "[" << a.vec() << ": " << a.angle() << "]";
}

}  // namespace truck::geom
