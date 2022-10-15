#include "geom/line.h"

namespace truck::geom {

bool equal(const Line& a, const Line& b, double eps) noexcept {
    return equal(a.normal() * b.c, b.normal() * a.c, eps);
}

std::ostream& operator<<(std::ostream& out, const Line& l) noexcept {
    return out << "Line(" << l.a << ", " << l.b << ", " << l.c << ")";
}

}  // namespace truck::geom
