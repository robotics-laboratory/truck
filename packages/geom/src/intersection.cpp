#include "geom/intersection.h"

#include "geom/segment.h"
#include "geom/vector.h"

#include <cmath>

namespace truck::geom {

namespace {

enum class Mode : int8_t { kColinear = 0, kClockwise = 1, kCounterclockwise = -1 };

}  // namespace

/** Implementation of segments intersection
 *
 * See https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect
 */
bool intersect(const Segment& seg1, const Segment& seg2, const double eps) noexcept {
    // Find orientation of ordered triplet (p, q, r)
    auto orientation = [&](const Vec2& p, const Vec2& q, const Vec2& r) {
        double const val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

        if (std::abs(val) < eps) {
            return Mode::kColinear;
        }

        if (val > 0) {
            return Mode::kClockwise;
        }

        return Mode::kCounterclockwise;
    };

    // Given three collinear points p, q, r, the function checks if
    // point q lies on line segment 'pr'
    auto on_segment = [](const Vec2& p, const Vec2& q, const Vec2& r) {
        return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y)
               && q.y >= std::min(p.y, r.y);
    };

    const Vec2& p1 = seg1.begin;
    const Vec2& q1 = seg1.end;
    const Vec2& p2 = seg2.begin;
    const Vec2& q2 = seg2.end;

    auto o1 = orientation(p1, q1, p2);
    auto o2 = orientation(p1, q1, q2);
    auto o3 = orientation(p2, q2, p1);
    auto o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) {
        return true;
    }

    if (o1 == Mode::kColinear && on_segment(p1, p2, q1)) {
        return true;
    }

    if (o2 == Mode::kColinear && on_segment(p1, q2, q1)) {
        return true;
    }

    if (o3 == Mode::kColinear && on_segment(p2, p1, q2)) {
        return true;
    }

    if (o4 == Mode::kColinear && on_segment(p2, q1, q2)) {
        return true;
    }

    return false;
}

bool intersect(const Polygon& polygon, const Segment& seg, const double eps) noexcept {
    for (const Segment& poly_seg : polygon.segments()) {
        if (intersect(poly_seg, seg, eps)) {
            return true;
        }
    }

    return false;
}

std::optional<Vec2> intersect(const Line& l1, const Line& l2, const double eps) noexcept {
    double const det = cross(Vec2(l1.a, l1.b), Vec2(l2.a, l2.b));
    if (std::abs(det) < eps) {
        return std::nullopt;
    }
    return Vec2(
        cross(Vec2(l1.b, l1.c), Vec2(l2.b, l2.c)) / det,
        cross(Vec2(l1.c, l1.a), Vec2(l2.c, l2.a)) / det);
}

std::optional<Vec2> intersect(const Ray& ray, const Segment& segment, double precision) noexcept {
    auto ray_dir = ray.dir.vec();
    auto segment_dir = static_cast<Vec2>(segment);

    auto det = cross(segment_dir, ray_dir);
    if (std::abs(det) < precision) {
        return {};
    }

    auto ray_origin_to_segment_begin = segment.begin - ray.origin;

    auto t = cross(segment_dir, ray_origin_to_segment_begin) / det;
    auto u = cross(ray_dir, ray_origin_to_segment_begin) / det;

    if (t >= -precision && u >= -precision && u <= 1 + precision) {
        return ray.origin + t * ray_dir;
    }

    return {};
}

}  // namespace truck::geom
