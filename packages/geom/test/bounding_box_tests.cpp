#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/bounding_box.h"

using namespace truck::geom;

TEST(BoundingBox, constructor) {
    // Arrange.
    const auto v1 = Vec2{1, 3};
    const auto v2 = Vec2{-10, -2};
    const auto v3 = Vec2{100, 24};

    // Act.
    const auto bb1 = BoundingBox(v1);
    const auto bb2 = BoundingBox(v2, v3);
    const auto bb3 = BoundingBox(v3, v2);

    // Assert.
    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(bb1.min.x, 1., eps);
    ASSERT_GEOM_EQUAL(bb1.min.y, 3., eps);
    ASSERT_GEOM_EQUAL(bb1.max.x, 1., eps);
    ASSERT_GEOM_EQUAL(bb1.max.y, 3., eps);

    ASSERT_GEOM_EQUAL(bb2.min.x, -10., eps);
    ASSERT_GEOM_EQUAL(bb2.min.y, -2., eps);
    ASSERT_GEOM_EQUAL(bb2.max.x, 100., eps);
    ASSERT_GEOM_EQUAL(bb2.max.y, 24., eps);

    ASSERT_GEOM_EQUAL(bb3.min.x, -10., eps);
    ASSERT_GEOM_EQUAL(bb3.min.y, -2., eps);
    ASSERT_GEOM_EQUAL(bb3.max.x, 100., eps);
    ASSERT_GEOM_EQUAL(bb3.max.y, 24., eps);
}

TEST(Vector3, extend) {
    // Arrange.
    auto bb1 = BoundingBox({1, 3});
    auto bb2 = BoundingBox({1, 3});
    auto bb3 = BoundingBox({-1, 6});
    auto bb4 = BoundingBox({-1, 6});
    const auto v1 = Vec2{7, 4};
    const auto v2 = Vec2{-1, -8};
    const auto margin = 4;
    
    // Act.
    bb1.extend(v1);
    bb1.extend(v2);

    extend(bb2, v1);
    extend(bb2, v2);

    bb3.extend(margin);

    extend(bb4, margin);

    // Assert.
    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(bb1.min.x, -1., eps);
    ASSERT_GEOM_EQUAL(bb1.min.y, -8., eps);
    ASSERT_GEOM_EQUAL(bb1.max.x, 7., eps);
    ASSERT_GEOM_EQUAL(bb1.max.y, 4., eps);

    ASSERT_GEOM_EQUAL(bb2.min.x, -1., eps);
    ASSERT_GEOM_EQUAL(bb2.min.y, -8., eps);
    ASSERT_GEOM_EQUAL(bb2.max.x, 7., eps);
    ASSERT_GEOM_EQUAL(bb2.max.y, 4., eps);

    ASSERT_GEOM_EQUAL(bb3.min.x, 3., eps);
    ASSERT_GEOM_EQUAL(bb3.min.y, -5., eps);
    ASSERT_GEOM_EQUAL(bb3.max.x, 10., eps);
    ASSERT_GEOM_EQUAL(bb3.max.y, 2., eps);

    ASSERT_GEOM_EQUAL(bb4.min.x, 3., eps);
    ASSERT_GEOM_EQUAL(bb4.min.y, -5., eps);
    ASSERT_GEOM_EQUAL(bb4.max.x, 10., eps);
    ASSERT_GEOM_EQUAL(bb4.max.y, 2., eps);
}

TEST(BoundingBox, makeBoundingBox) {
    // Arrange.
    const auto p1 = Polygon{{0, 0}, {0, 1}, {1, 0}};
    const auto p2 = Polygon{{-1, -7}, {-2, 8}, {14, 6}, {5, -1}};

    // Act.
    const auto bb1 = makeBoundingBox(p1);
    const auto bb2 = makeBoundingBox(p2);

    // Assert.
    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(bb1.min.x, 0., eps);
    ASSERT_GEOM_EQUAL(bb1.min.y, 0., eps);
    ASSERT_GEOM_EQUAL(bb1.max.x, 1., eps);
    ASSERT_GEOM_EQUAL(bb1.max.y, 1., eps);

    ASSERT_GEOM_EQUAL(bb2.min.x, -2., eps);
    ASSERT_GEOM_EQUAL(bb2.min.y, -7., eps);
    ASSERT_GEOM_EQUAL(bb2.max.x, 14., eps);
    ASSERT_GEOM_EQUAL(bb2.max.y, 5., eps);
}
