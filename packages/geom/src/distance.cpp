#include "geom/distance.h"

#include "common/math.h"

namespace truck::geom {

double distanceSq(const Vec2& a, const Vec2& b) noexcept { return (a - b).lenSq(); }

double distance(const Vec2& a, const Vec2& b) noexcept { return (a - b).len(); }

double distanceSq(const Vec2& p, const Segment& s) noexcept { return squared(distance(p, s)); }

double distance(const Vec2& p, const Segment& s) noexcept {
    const Vec2 a = s.begin;
    const Vec2 b = s.end;
    const Vec2 ab = b - a;
    const Vec2 ap = p - a;

    const double ap_ab = dot(ap, ab);

    if (ap_ab <= 0) {
        return (p - a).len();
    }

    // projection scalar of ap on ab
    const double t = ap_ab / dot(ab, ab);

    if (t >= 1) {
        return (p - b).len();
    }

    const Vec2 c = a + t * ab;
    return (p - c).len();
}

double denormalizedDistance(const Line& l, const Vec2& p) noexcept {
    return (dot(l.normal(), p) + l.c);
}

double denormalizedDistance(const Vec2& p, const Line& l) noexcept {
    return denormalizedDistance(l, p);
}

double distance(const Line& l, const Vec2& p) noexcept {
    return std::abs(denormalizedDistance(l, p) / l.normal().len());
}

double distance(const Vec2& p, const Line& l) noexcept { return distance(l, p); }

}  // namespace truck::geom
