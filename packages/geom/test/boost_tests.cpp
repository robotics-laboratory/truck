#include <algorithm>

#include <gtest/gtest.h>
#include <boost/geometry.hpp>

#include "geom/boost.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;
using Points = std::vector<Vec2>;

namespace bg = boost::geometry;

template<typename T>
T geometry_from_points(const Points& pts) {
    T geometry;

    for (Vec2 p : pts) {
        bg::append(geometry, p);
    }

    return geometry;
}

bool cmp_vec2(const Vec2& lhs, const Vec2& rhs) {
    return std::tie(lhs.x, lhs.y) == std::tie(rhs.x, rhs.y);
}

bool equal_points(const Points& p1, const Points& p2) {
    return std::equal(p1.begin(), p1.end(), p2.begin(), p2.end(), cmp_vec2);
}

TEST(BoostVector, coords) {
    Vec2 v2(1, 2);
    Vec3 v3(3, 4, 5);

    bg::set<0>(v2, 42.69);
    bg::set<2>(v3, 420);

    ASSERT_GEOM_EQUAL(v2.x, 42.69);
    ASSERT_GEOM_EQUAL(v3.z, 420.0);

    EXPECT_TRUE(std::make_tuple(v2.x, v2.y) == std::make_tuple(bg::get<0>(v2), bg::get<1>(v2)));
    EXPECT_TRUE(
        std::make_tuple(v3.x, v3.y, v3.z)
        == std::make_tuple(bg::get<0>(v3), bg::get<1>(v3), bg::get<2>(v3)));
}

TEST(BoostBoundingBox, corners) {
    BoundingBox box({0, 0}, {0, 0});

    bg::set<bg::min_corner, 0>(box, 1.0);
    bg::set<bg::min_corner, 1>(box, 2.0);
    bg::set<bg::max_corner, 0>(box, 3.0);
    bg::set<bg::max_corner, 1>(box, 4.0);

    const double x0 = bg::get<bg::min_corner, 0>(box);
    const double y0 = bg::get<bg::min_corner, 1>(box);
    const double x1 = bg::get<bg::max_corner, 0>(box);
    const double y1 = bg::get<bg::max_corner, 1>(box);

    ASSERT_GEOM_EQUAL(x0, 1.0);
    ASSERT_GEOM_EQUAL(y0, 2.0);
    ASSERT_GEOM_EQUAL(x1, 3.0);
    ASSERT_GEOM_EQUAL(y1, 4.0);
}

TEST(BoostSegment, points) {
    Segment seg({}, {});

    bg::set<0, 0>(seg, 1.0);
    bg::set<0, 1>(seg, 2.0);
    bg::set<1, 0>(seg, 3.0);
    bg::set<1, 1>(seg, 4.0);

    const double x0 = bg::get<0, 0>(seg);
    const double y0 = bg::get<0, 1>(seg);
    const double x1 = bg::get<1, 0>(seg);
    const double y1 = bg::get<1, 1>(seg);

    ASSERT_GEOM_EQUAL(x0, 1.0);
    ASSERT_GEOM_EQUAL(y0, 2.0);
    ASSERT_GEOM_EQUAL(x1, 3.0);
    ASSERT_GEOM_EQUAL(y1, 4.0);
}

TEST(BoostPolygon, points) {
    Points pts = {{0, 0}, {0, 5}, {5, 5}, {5, 0}, {0, 0}};
    Polygon ring = geometry_from_points<Polygon>(pts);

    ASSERT_EQ(bg::area(ring), 25.0);
    ASSERT_TRUE(equal_points(pts, ring));
}

TEST(BoostComplexPolygon, area) {
    ComplexPolygon poly;

    bg::append(poly.outer, Vec2(0.0, 0.0));
    bg::append(poly.outer, Vec2(0.0, 5.0));
    bg::append(poly.outer, Vec2(5.0, 5.0));
    bg::append(poly.outer, Vec2(5.0, 0.0));
    bg::append(poly.outer, Vec2(0.0, 0.0));

    poly.inners.resize(1);
    bg::append(poly.inners[0], Vec2(1.0, 1.0));
    bg::append(poly.inners[0], Vec2(4.0, 1.0));
    bg::append(poly.inners[0], Vec2(4.0, 4.0));
    bg::append(poly.inners[0], Vec2(1.0, 4.0));
    bg::append(poly.inners[0], Vec2(1.0, 1.0));

    ASSERT_GEOM_EQUAL(bg::area(poly), 16.0);
}

TEST(BoostPolyline, points) {
    Points pts = {{0, 0}, {1, 0}, {1, 2}};
    Polyline ls = geometry_from_points<Polyline>(pts);

    ASSERT_TRUE(equal_points(pts, ls));
    ASSERT_EQ(bg::length(ls), 3.0);
}
