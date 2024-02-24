#include "geom/intersection.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <cmath>

namespace truck::geom {

std::optional<Vec2> intersect(const Ray& ray, const Segment& segment, double precision) noexcept {

    auto ray_dir = ray.dir.vec();
    auto segment_dir = static_cast<Vec2>(segment);

    auto det = cross(segment_dir, ray_dir);
    if (std::abs(det) < precision) {
        return {};
    }

    auto rayOriginToSegmentBegin = segment.begin - ray.origin;

    auto t = cross(segment_dir, rayOriginToSegmentBegin) / det;
    auto u = cross(ray_dir, rayOriginToSegmentBegin) / det;

    if (t >= -precision && u >= -precision && u <= 1 + precision) {
        return ray.origin + t * ray_dir;
    }

    return {};
}

}  // namespace truck::geom
