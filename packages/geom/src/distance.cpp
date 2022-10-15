#include "geom/distance.h"

namespace truck::geom {

double distanceSq(const Vec2& a, const Vec2& b) noexcept { return (a - b).lenSq(); }

double distance(const Vec2& a, const Vec2& b) noexcept { return (a - b).len(); }

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
