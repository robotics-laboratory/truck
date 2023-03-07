#pragma once

#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

struct Segment {
    Segment(const Vec2& begin, const Vec2& end) : begin(begin), end(end) {}

    operator Vec2() const noexcept { return end - begin; }

    double lenSq() const noexcept { return geom::lenSq(end - begin); }

    double len() const { return geom::len(end - begin); }

    Vec2 dir() const { return (end - begin).unit(); }

    Vec2 pos(double t) const noexcept { return begin * (1 - t) + end * t; }

    Poses trace(double step) const noexcept;

    Vec2 begin;
    Vec2 end;
};

} // namespace truck::geom