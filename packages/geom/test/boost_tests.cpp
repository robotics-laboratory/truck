#include <gtest/gtest.h>
#include <boost/geometry.hpp>

#include "geom/boost.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;
namespace bg = boost::geometry;

TEST(BoostVector, get) {
    const Vec2 v2(1, 2);
    const Vec3 v3(3, 4, 5);

    EXPECT_TRUE(std::make_tuple(v2.x, v2.y) == std::make_tuple(bg::get<0>(v2), bg::get<1>(v2)));
    EXPECT_TRUE(
        std::make_tuple(v3.x, v3.y, v3.z)
        == std::make_tuple(bg::get<0>(v3), bg::get<1>(v3), bg::get<2>(v3)));
}

TEST(BoostVector, set) {
    Vec2 v2;

    bg::set<0>(v2, 42.69);

    ASSERT_GEOM_EQUAL(v2.x, 42.69);
}

TEST(BoostBoundingBox, get) {
    const BoundingBox box({1, 2}, {3, 4});
    double x0 = bg::get<bg::min_corner, 0>(box);
    double y0 = bg::get<bg::min_corner, 1>(box);
    double x1 = bg::get<bg::max_corner, 0>(box);
    double y1 = bg::get<bg::max_corner, 1>(box);

    ASSERT_GEOM_EQUAL(x0, 1.0);
    ASSERT_GEOM_EQUAL(y0, 2.0);
    ASSERT_GEOM_EQUAL(x1, 3.0);
    ASSERT_GEOM_EQUAL(y1, 4.0);
}

TEST(BoostBoundingBox, set) {
    BoundingBox box({0, 0}, {0, 0});

    bg::set<bg::min_corner, 0>(box, 1.0);
    bg::set<bg::max_corner, 1>(box, 2.0);

    ASSERT_GEOM_EQUAL(box.min.x, 1.0);
    ASSERT_GEOM_EQUAL(box.max.y, 2.0);
}

TEST(BoostSegment, get) {
    const Segment seg({1, 2}, {3, 4});

    double x0 = bg::get<0, 0>(seg);
    double y0 = bg::get<0, 1>(seg);
    double x1 = bg::get<1, 0>(seg);
    double y1 = bg::get<1, 1>(seg);

    ASSERT_GEOM_EQUAL(x0, 1.0);
    ASSERT_GEOM_EQUAL(y0, 2.0);
    ASSERT_GEOM_EQUAL(x1, 3.0);
    ASSERT_GEOM_EQUAL(y1, 4.0);
}

TEST(BoostSegment, set) {
    Segment seg({}, {});

    bg::set<0, 0>(seg, 1.0);
    bg::set<0, 1>(seg, 2.0);
    bg::set<1, 0>(seg, 3.0);
    bg::set<1, 1>(seg, 4.0);

    double x0 = bg::get<0, 0>(seg);
    double y0 = bg::get<0, 1>(seg);
    double x1 = bg::get<1, 0>(seg);
    double y1 = bg::get<1, 1>(seg);

    ASSERT_GEOM_EQUAL(x0, 1.0);
    ASSERT_GEOM_EQUAL(y0, 2.0);
    ASSERT_GEOM_EQUAL(x1, 3.0);
    ASSERT_GEOM_EQUAL(y1, 4.0);
}

TEST(BoostPolygon, area) {
    Polygon ring;

    bg::append(ring, Vec2(0.0, 0.0));
    bg::append(ring, Vec2(0.0, 5.0));
    bg::append(ring, Vec2(5.0, 5.0));
    bg::append(ring, Vec2(5.0, 0.0));
    bg::append(ring, Vec2(0.0, 0.0));

    double a = bg::area(ring);
    ASSERT_GEOM_EQUAL(a, 25.0);
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

    double a = bg::area(poly);
    ASSERT_GEOM_EQUAL(a, 16.0);
}

TEST(BoostPolyline, length) {
    Polyline ls;

    bg::append(ls, Vec2(0.0, 0.0));
    bg::append(ls, Vec2(1.0, 0.0));
    bg::append(ls, Vec2(1.0, 2.0));

    double l = bg::length(ls);
    ASSERT_GEOM_EQUAL(l, 3.0);
}
