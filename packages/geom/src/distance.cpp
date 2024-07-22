#include "geom/distance.h"

#include "common/math.h"

namespace truck::geom {

double distanceSq(const Vec2& a, const Vec2& b) noexcept { return (a - b).lenSq(); }

double distance(const Vec2& a, const Vec2& b) noexcept { return (a - b).len(); }

double distanceSq(const Vec2& p, const Segment& s) noexcept {
    const Vec2 ab = s.end - s.begin;
    const Vec2 ap = p - s.begin;
    const Vec2 bp = p - s.end;

    const double ab_len_sq = ab.lenSq();
    const double ap_ab = dot(ap, ab);
    const double bp_ab = dot(bp, ab);

    if (ap_ab <= 0) {
        return ap.lenSq();
    }

    if (bp_ab >= 0) {
        return bp.lenSq();
    }

    return ap.lenSq() - squared(ap_ab) / ab_len_sq;
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
