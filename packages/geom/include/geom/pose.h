#pragma once

#include "geom/angle_vector.h"
#include "geom/vector.h"
#include "geom/common.h"

#include <ostream>
#include <vector>

namespace truck::geom {

/*
 * Position of rigid body in 2D space.
 *
 * The pose is a combination of position (translation) and orientation (rotation).
 */

struct Pose {
    constexpr operator Vec2() const { return pos; }

    Vec2 pos{};
    AngleVec2 dir{};
};

std::ostream& operator<<(std::ostream& out, const Pose& pose) noexcept;

Pose interpolate(const Pose& a, const Pose& b, double t) noexcept;

template<>
Pose lerp<Pose>(const Pose& a, const Pose& b, double t);

using Poses = std::vector<Pose>;

}  // namespace truck::geom
