#pragma once

#include "geom/angle.h"
#include "geom/vector.h"
#include "geom/pose.h"

#include <tf2/LinearMath/Transform.h>

#include <ostream>

namespace truck::geom {

/*
 * Transform of rigid body in 2D space.
 *
 * The transform is a combination of translation and rotation.
 * Pose of rigid body can be represented as a transform applied to the origin.
 */

class Transform {
  public:
    Transform() = default;

    Transform(Vec2 t, Angle a);
    Transform(Vec2 t, AngleVec2 r);

    Transform(const tf2::Transform& tf);

    Vec2 apply(Vec2 v) const;
    Pose apply(Pose p) const;

    Vec2 operator()(Vec2 v) const;
    Pose operator()(Pose p) const;

    const Vec2& t() const { return translation_; }
    const AngleVec2& r() const { return rotation_; }

    Transform inv() const;

  private:
    Vec2 translation_ = {};
    AngleVec2 rotation_ = {};
};

std::ostream& operator<<(std::ostream& out, const Transform& transform) noexcept;

}  // namespace truck::geom
