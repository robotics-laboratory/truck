#include "geom/rtree.h"

#include "common/math.h"

namespace truck::geom {

Vec2 toVec2(const RTreePoint& rtree_point) {
    return Vec2(rtree_point.get<0>(), rtree_point.get<1>());
}

std::vector<Vec2> toVec2Array(const RTreeIndexedPoints& rtree_indexed_points) {
    std::vector<Vec2> points;

    for (const RTreeIndexedPoint& rtree_indexed_point : rtree_indexed_points) {
        points.emplace_back(toVec2(rtree_indexed_point.first));
    }

    return points;
}

RTreePoint toRTreePoint(const Vec2& point) { return RTreePoint(point.x, point.y); }

RTreeIndexedPoint toRTreeIndexedPoint(const Vec2& point, size_t index) {
    return RTreeIndexedPoint(toRTreePoint(point), index);
}

RTree toRTree(const std::vector<Vec2>& points) {
    RTree rtree;

    for (size_t i = 0; i < points.size(); i++) {
        rtree.insert(toRTreeIndexedPoint(points[i], i));
    }

    return rtree;
}

RTreeIndexedPoints RTreeSearchKNN(const RTree& rtree, const Vec2& point, size_t k) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), k), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints RTreeSearchInsideBox(const RTree& rtree, const Vec2& center, double radius) {
    RTreeIndexedPoints rtree_indexed_points;

    RTreeBox rtree_box(
        RTreePoint(center.x - radius, center.y - radius),
        RTreePoint(center.x + radius, center.y + radius));

    rtree.query(bg::index::intersects(rtree_box), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints RTreeSearchInsideCircle(const RTree& rtree, const Vec2& center, double radius) {
    RTreeIndexedPoints rtree_indexed_points;

    RTreeBox rtree_box(
        RTreePoint(center.x - radius, center.y - radius),
        RTreePoint(center.x + radius, center.y + radius));

    rtree.query(
        bg::index::intersects(rtree_box) &&
            bg::index::satisfies([&](RTreeIndexedPoint const& rtree_indexed_point) {
                Vec2 neighbor = toVec2(rtree_indexed_point.first);
                return (center - neighbor).lenSq() < squared(radius);
            }),
        std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

}  // namespace truck::geom