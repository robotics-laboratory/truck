#pragma once

#include "geom/vector.h"

#include <algorithm>

namespace truck::geom {

struct BoundingBox {
    BoundingBox(const Vec2& v) {
        min = v;
        max = v;
    }

    BoundingBox(const Vec2& a, const Vec2& b) {
        min.x = std::min(a.x, b.x);
        min.y = std::min(a.y, b.y);
        max.x = std::max(a.x, b.x);
        max.y = std::max(a.y, b.y);
    }

    BoundingBox& extend(const Vec2& v) noexcept;
    BoundingBox& extend(double margin) noexcept;

    Vec2 min, max;
};

inline BoundingBox extend(BoundingBox& box, const Vec2& v) noexcept {
    return box.extend(v);
}

inline BoundingBox extend(BoundingBox& box, double margin) noexcept {
    return box.extend(margin);
}

}  // namespace truck::geom
