#include "geom/common.h"
#include "geom/vector.h"

namespace truck::geom {

Vec2 operator*(const Vec2& v, double c) noexcept { return {v.x * c, v.y * c}; }

Vec2 operator*(double c, const Vec2& v) noexcept { return v * c; }

Vec2 operator/(const Vec2& v, double c) noexcept { return Vec2{v.x / c, v.y / c}; }

bool equal(const Vec2& a, const Vec2& b, double eps) noexcept {
    return equal(a.x, b.x, eps) && equal(a.y, b.y, eps);
}

double dot(const Vec2& a, const Vec2& b) noexcept { return a.x * b.x + a.y * b.y; }

double cross(const Vec2& a, const Vec2& b) noexcept { return a.x * b.y - a.y * b.x; }

Angle angleBetween(const Vec2& from, const Vec2& to) {
    return acos(dot(from, to) / std::sqrt(from.lenSq() * to.lenSq()));
}

std::ostream& operator<<(std::ostream& out, const Vec2& v) {
    return out << "(" << v.x << ", " << v.y << ")";
}

}  // namespace truck::geom