#include "geom/arc.h"
#include "geom/common.h"

#include "common/math.h"

#include <boost/assert.hpp>

#include <iostream>

namespace truck::geom {

std::variant<std::monostate, Segment, Arc> tryBuildArc(const Pose& from, const Vec2& to) {
    constexpr double eps = 1e-3;

    const Vec2 chord = to - from.pos;
    const double len = chord.len();

    if (len < eps) {
        return Segment{from.pos, from.pos};
    }

    const Vec2 chord_dir = chord / len;

    const double cross_prod = cross(from.dir, chord_dir);
    const double dot_prod = dot(from.dir, chord_dir);

    if (equal(cross_prod, 0.0, eps)) {
        if (equal(dot_prod, 1.0, eps)) {
            return Segment{from.pos, to};
        }

        return std::monostate();
    }

    const double radius = len / (2 * std::abs(cross_prod));
    const Vec2 radius_dir = cross_prod > 0 ? from.dir.left() : from.dir.right();

    return Arc(
        from.pos + radius * radius_dir,
        radius,
        -radius_dir,
        2 * atan(cross_prod, dot_prod)
    );
}

std::ostream& operator<<(std::ostream& os, const Arc& arc) {
    os << "arc("
       << "c=" << arc.center
       << " r=" << arc.radius
       << " b=" << arc.begin
       << " d=" << arc.delta
       << ")";
    return os;
}

Poses Arc::trace(double step) const {
    BOOST_ASSERT(step > 0);

    const size_t n = 1 + ceil<size_t>(len() / step);
    const Angle angle_step = delta / n;

    Vec2 radius_vec = radius * begin;
    Vec2 dir = ccw() > 0 ? begin.left() : begin.right();

    Poses poses;
    poses.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        poses.emplace_back(center + radius_vec, dir);
        radius_vec = radius_vec.rotate(angle_step);
        dir = dir.rotate(angle_step);
    }

    return poses;
}

} // namespace truck::geom