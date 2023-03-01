#pragma once

#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

struct Segment {
    Vec2 begin;
    Vec2 end;

    operator Vec2() const { return end - begin; }

    double lenSq() const { return geom::lenSq(end - begin); }

    double len() const { return geom::len(end - begin); }

    Vec2 dir() const { return (end - begin).unit(); }

    Vec2 pos(double t) const { return begin * (1 - t) + end * t; }

    Poses trace(double step) const;
};

} // namespace truck::geom