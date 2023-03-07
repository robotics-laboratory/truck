#pragma once

#include "geom/angle.h"
#include "geom/vector.h"
#include "geom/pose.h"

namespace truck::geom {

class Transform {
public:
    Transform() = default;

    Transform(const Vec2& t, const Angle& r) : translation_(t), rotation_(r) {}

    Vec2 apply(const Vec2& v) const;
    Pose apply(const Pose& p) const;

    Transform inv() const;

private:
    Vec2 translation_;
    Angle rotation_;
};

}  // namespace truck::geom