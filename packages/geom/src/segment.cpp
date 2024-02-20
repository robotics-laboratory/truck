#include "geom/segment.h"
#include "geom/bezier.h"

#include "common/math.h"

namespace truck::geom {

Poses Segment::trace(double step) const noexcept { return bezier1(begin, end, step); }

Vec2 Segment::pos(double t) const noexcept {
    t = clamp(t, 0.0, 1.0);
    return begin * (1 - t) + end * t;
}

geom::Vec2 projection(const geom::Vec2& point, const geom::Segment& segment) noexcept {
    const geom::Vec2 dir(segment);

    const double t = dot(point - segment.begin, dir) / dir.lenSq();
    return segment.pos(t);
}

}  // namespace truck::geom
