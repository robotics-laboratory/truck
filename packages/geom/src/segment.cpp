#include "geom/segment.h"
#include "geom/bezier.h"

#include "common/math.h"

namespace truck::geom {

Poses Segment::trace(double step) const noexcept { return bezier1(begin, end, step); }

Vec2 Segment::pos(double t) const noexcept {
    t = clamp(t, 0.0, 1.0);
    return begin * (1 - t) + end * t;
}

geom::Vec2 projection(const geom::Vec2& point, const geom::Segment& segment) noexcept {
    const geom::Vec2 dir(segment);

    const double t = dot(point - segment.begin, dir) / dir.lenSq();
    return segment.pos(t);
}


/** Implementation of segments intersection
 *
 * See https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect
 */
bool intersect(const geom::Segment& seg1, const geom::Segment& seg2) noexcept {
    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are collinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    auto orientation = [](const geom::Vec2& p, const geom::Vec2& q, const geom::Vec2& r) {
        int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y); 

        // collinear 
        if (val == 0) {
            return 0;
        }  

        // clock or counterclock wise 
        return (val > 0)? 1: 2;
    };

    // Given three collinear points p, q, r, the function checks if
    // point q lies on line segment 'pr'
    auto onSegment = [](const geom::Vec2& p, const geom::Vec2& q, const geom::Vec2& r) {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;

        return false;
    };

    const geom::Vec2& p1 = seg1.begin;
    const geom::Vec2& q1 = seg1.end;
    const geom::Vec2& p2 = seg2.begin;
    const geom::Vec2& q2 = seg2.end;

    // Find the four orientations needed for general and special cases 
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

}  // namespace truck::geom
