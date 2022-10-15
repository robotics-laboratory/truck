#include "geom/circle.h"
#include "geom/common.h"

namespace truck::geom {

bool equal(const Circle& a, const Circle& b, double eps) noexcept {
    return equal(a.radius, b.radius, eps) && equal(a.center, b.center, eps);
}

std::ostream& operator<<(std::ostream& out, const Circle& c) noexcept {
    return out << "Circle{" << c.center << ", " << c.radius << "}";
}

}  // namespace truck::geom
