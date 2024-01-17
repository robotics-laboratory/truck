#include "geom/ray.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <cmath>

namespace truck::geom {

bool checkIntersection(const Ray& ray, const Segment& segment,
    Vec2& intersection, double precision) noexcept {

    auto ray_dir = ray.dir.vec();
    auto segment_dir = static_cast<Vec2>(segment);

    auto det = cross(segment_dir, ray_dir);
    if (std::abs(det) < precision) {
        return false;
    }

    auto rayOriginToSegmentBegin = segment.begin - ray.origin;

    auto t = cross(segment_dir, rayOriginToSegmentBegin) / det;
    auto u = cross(ray_dir, rayOriginToSegmentBegin) / det;

    if (t >= -precision && u >= -precision && u <= 1 + precision) {
        intersection = ray.origin + t * ray_dir;
        return true;
    }

    return false;
}

}  // namespace truck::geom
