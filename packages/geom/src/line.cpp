#include "geom/line.h"

namespace truck::geom {

bool equal(const Line& a, const Line& b, double eps) noexcept {
    return equal(a.normal() * b.c, b.normal() * a.c, eps);
}

std::optional<Vec2> intersect(const Line& l1, const Line& l2) noexcept {
    double det = cross(Vec2(l1.a, l1.b), Vec2(l2.a, l2.b));
    if (det == 0) {
        return std::nullopt;
    }
    return Vec2(
        cross(Vec2(l1.b, l1.c), Vec2(l2.b, l2.c)) / det,
        cross(Vec2(l1.c, l1.a), Vec2(l2.c, l2.a)) / det);
}

std::ostream& operator<<(std::ostream& out, const Line& l) noexcept {
    return out << "Line(" << l.a << ", " << l.b << ", " << l.c << ")";
}

}  // namespace truck::geom
