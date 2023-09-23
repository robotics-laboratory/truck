#pragma once

#include "geom/vector.h"

namespace truck::geom {

struct BBox {
    Vec2 left_lower;
    Vec2 right_upper;
};

}  // namespace truck::geom