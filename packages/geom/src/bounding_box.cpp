#include "geom/bounding_box.h"

namespace truck::geom {

BoundingBox& BoundingBox::extend(const Vec2& v) noexcept {
    min.x = std::min(min.x, v.x);
    min.y = std::min(min.y, v.y);
    max.x = std::max(max.x, v.x);
    max.y = std::max(max.y, v.y);
    return *this;
}

BoundingBox& BoundingBox::extend(double margin) noexcept {
    min.x -= margin;
    min.y -= margin;
    max.x += margin;
    max.y += margin;
    return *this;
}

}  // namespace truck::geom
