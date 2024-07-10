#include <algorithm>

#include <gtest/gtest.h>
#include <boost/geometry.hpp>

#include "geom/boost.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

namespace bg = boost::geometry;

TEST(Vec2, boost_geometry_register) {
    Vec2 v2;

    bg::set<0>(v2, 42.69);
    bg::set<1>(v2, 10.01);

    ASSERT_GEOM_EQUAL(v2.x, 42.69);
    ASSERT_GEOM_EQUAL(v2.y, 10.01);

    ASSERT_GEOM_EQUAL(v2.x, bg::get<0>(v2));
    ASSERT_GEOM_EQUAL(v2.y, bg::get<1>(v2));
}

TEST(Vec3, boost_geometry_register) {
    Vec3 v3;

    bg::set<0>(v3, 0.21);
    bg::set<1>(v3, 7.93);
    bg::set<2>(v3, 3.15);

    ASSERT_GEOM_EQUAL(v3.x, 0.21);
    ASSERT_GEOM_EQUAL(v3.y, 7.93);
    ASSERT_GEOM_EQUAL(v3.z, 3.15);

    ASSERT_GEOM_EQUAL(v3.x, bg::get<0>(v3));
    ASSERT_GEOM_EQUAL(v3.y, bg::get<1>(v3));
    ASSERT_GEOM_EQUAL(v3.z, bg::get<2>(v3));
}

TEST(BoundingBox, boost_geometry_register) {
    BoundingBox box({}, {});

    bg::set<bg::min_corner, 0>(box, 1.0);
    bg::set<bg::min_corner, 1>(box, 2.0);
    bg::set<bg::max_corner, 0>(box, 3.0);
    bg::set<bg::max_corner, 1>(box, 4.0);

    ASSERT_GEOM_EQUAL(box.min.x, 1.0);
    ASSERT_GEOM_EQUAL(box.min.y, 2.0);
    ASSERT_GEOM_EQUAL(box.max.x, 3.0);
    ASSERT_GEOM_EQUAL(box.max.y, 4.0);

    ASSERT_GEOM_EQUAL(box.min.x, bg::get<bg::min_corner, 0>(box));
    ASSERT_GEOM_EQUAL(box.min.y, bg::get<bg::min_corner, 1>(box));
    ASSERT_GEOM_EQUAL(box.max.x, bg::get<bg::max_corner, 0>(box));
    ASSERT_GEOM_EQUAL(box.max.y, bg::get<bg::max_corner, 1>(box));
}

TEST(Segment, boost_geometry_register) {
    Segment seg({}, {});

    bg::set<0, 0>(seg, 1.0);
    bg::set<0, 1>(seg, 2.0);
    bg::set<1, 0>(seg, 3.0);
    bg::set<1, 1>(seg, 4.0);

    ASSERT_GEOM_EQUAL(seg.begin.x, 1.0);
    ASSERT_GEOM_EQUAL(seg.begin.y, 2.0);
    ASSERT_GEOM_EQUAL(seg.end.x, 3.0);
    ASSERT_GEOM_EQUAL(seg.end.y, 4.0);

    ASSERT_GEOM_EQUAL(seg.begin.x, bg::get<0, 0>(seg));
    ASSERT_GEOM_EQUAL(seg.begin.y, bg::get<0, 1>(seg));
    ASSERT_GEOM_EQUAL(seg.end.x, bg::get<1, 0>(seg));
    ASSERT_GEOM_EQUAL(seg.end.y, bg::get<1, 1>(seg));
}

TEST(Polygon, boost_geometry_register) {
    const std::vector<Vec2> points = {{0, 0}, {0, 5}, {5, 5}, {5, 0}, {0, 0}};

    Polygon ring;

    for (const Vec2& point : points) {
        bg::append(ring, point);
    }

    for (size_t i = 0; i < ring.size(); i++) {
        ASSERT_GEOM_EQUAL(ring[i].x, points[i].x);
        ASSERT_GEOM_EQUAL(ring[i].y, points[i].y);
    }
}

TEST(ComplexPolygon, boost_geometry_register) {
    const Polygon outer = {{0, 0}, {0, 5}, {5, 5}, {5, 0}, {0, 0}};
    const Polygon inner = {{1, 1}, {4, 1}, {4, 4}, {1, 4}, {1, 1}};

    ComplexPolygon poly;

    for (const Vec2& outer_point : outer) {
        bg::append(poly.outer, outer_point);
    }

    for (size_t i = 0; i < poly.outer.size(); i++) {
        ASSERT_GEOM_EQUAL(poly.outer[i].x, outer[i].x);
        ASSERT_GEOM_EQUAL(poly.outer[i].y, outer[i].y);
    }

    poly.inners.resize(1);
    for (const Vec2& inner_point : inner) {
        bg::append(poly.inners[0], inner_point);
    }

    for (size_t i = 0; i < poly.inners[0].size(); i++) {
        ASSERT_GEOM_EQUAL(poly.inners[0][i].x, inner[i].x);
        ASSERT_GEOM_EQUAL(poly.inners[0][i].y, inner[i].y);
    }
}

TEST(Polyline, boost_geometry_register) {
    const std::vector<Vec2> points = {{0, 0}, {1, 0}, {1, 2}};

    Polyline line_string;

    for (const Vec2& point : points) {
        bg::append(line_string, point);
    }

    for (size_t i = 0; i < line_string.size(); i++) {
        ASSERT_GEOM_EQUAL(line_string[i].x, points[i].x);
        ASSERT_GEOM_EQUAL(line_string[i].y, points[i].y);
    }
}
