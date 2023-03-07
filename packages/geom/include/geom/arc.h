#pragma once

#include "geom/common.h"
#include "geom/segment.h"
#include "geom/vector.h"
#include "geom/polyline.h"

#include <boost/assert.hpp>

#include <variant>

namespace truck::geom {

struct Arc {
    Arc(const Vec2& center, double radius, const Vec2 begin, Angle delta)
        : center(center)
        , radius(radius)
        , begin(begin)
        , delta(delta)
    {
        BOOST_ASSERT(radius > 0);
    }

    int ccw() const { return delta >= Angle::zero() ? 1 : -1; };

    double curv() const { return ccw() / radius; }

    double len() const { return radius * abs(delta).radians(); }

    Poses trace(double step) const;

    Vec2 center;
    double radius;
    Vec2 begin;
    Angle delta;
};

std::ostream& operator<<(std::ostream& os, const Arc& arc);

std::variant<std::monostate, Segment, Arc> tryBuildArc(const Pose& from, const Vec2& to);

}  // namespace truck::geom
