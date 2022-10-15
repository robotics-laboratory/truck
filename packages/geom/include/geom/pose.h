#pragma once

#include "geom/vector.h"

namespace truck::geom {

struct Pose {
    Vec2 pos;
    Vec2 dir;
};

} // namespace truck::geom