#include "geom/bounding_box.h"

#include "common/exception.h"

namespace truck::geom {

BoundingBox& BoundingBox::extend(const geom::Vec2& v) noexcept {
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

inline BoundingBox extend(BoundingBox& box, const geom::Vec2& v) noexcept {
    return box.extend(v);
}

inline BoundingBox extend(BoundingBox& box, double margin) noexcept {
    return box.extend(margin);
}

BoundingBox makeBoundingBox(const Polygon& polygon) noexcept {
    VERIFY(!polygon.empty());

    geom::BoundingBox box(polygon[0]);
    for (auto i = 1; i < polygon.size(); ++i) {
        box.extend(polygon[i]);
    }

    return box;
}

}  // namespace truck::geom
