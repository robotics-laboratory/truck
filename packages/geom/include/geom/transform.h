#pragma once

#include "geom/angle.h"
#include "geom/vector.h"
#include "geom/pose.h"

namespace truck::geom {

class Transform {
public:
    Transform() = default;
    Transform(const Vec2& t, Angle a);
    Transform(const Vec2& t, const Vec2& r);

    Vec2 apply(const Vec2& v) const;
    Pose apply(const Pose& p) const;

    Vec2 operator()(const Vec2& v) const;
    Pose operator()(const Pose& p) const;

    Transform inv() const;

private:
    Vec2 translation_ = {0 ,0};
    Vec2 rotation_ {1, 0};
};

}  // namespace truck::geom