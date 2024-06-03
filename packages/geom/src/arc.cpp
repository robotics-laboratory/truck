#include "geom/arc.h"
#include "geom/common.h"

#include "common/exception.h"
#include "common/math.h"

#include <iostream>

namespace truck::geom {

std::variant<std::monostate, Segment, Arc> tryBuildArc(
    const Pose& from, const Vec2& to, double eps) {
    const Vec2 chord = to - from.pos;
    const double len = chord.len();

    if (len < eps) {
        return std::monostate();
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

    return Arc(from.pos + radius * radius_dir, radius, -radius_dir, 2 * atan(cross_prod, dot_prod));
}

std::ostream& operator<<(std::ostream& os, const Arc& arc) {
    os << "arc("
       << "c=" << arc.center << " r=" << arc.radius << " b=" << arc.begin << " d=" << arc.delta
       << ")";
    return os;
}

Poses Arc::trace(double step) const {
    VERIFY(step > 0);

    const size_t n = 1 + ceil<size_t>(len() / step);
    const AngleVec2 angle_step(delta / n);

    Vec2 radius_vec = radius * begin;

    auto dir = AngleVec2::fromVector(ccw() > 0 ? begin.left() : begin.right());

    Poses poses;
    poses.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        poses.emplace_back(center + radius_vec, dir);
        radius_vec = angle_step.apply(radius_vec);
        dir = angle_step.apply(dir);
    }

    return poses;
}

}  // namespace truck::geom
