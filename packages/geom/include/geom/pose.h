#pragma once

#include "geom/vector.h"

#include <ostream>
#include <vector>

namespace truck::geom {

struct Pose {
    Pose() : pos(Vec2()), dir(Vec2()) {}

    Pose(const Vec2& pos, const Vec2& dir) : pos(pos), dir(dir) {}

    operator Vec2() const { return pos; }

    Vec2 pos;
    Vec2 dir;
};

std::ostream& operator<<(std::ostream& out, const Pose& pose) noexcept;

Pose interpolate(const Pose& a, const Pose& b, double t) noexcept;

using Poses = std::vector<Pose>;

}  // namespace truck::geom