#include <gtest/gtest.h>

#include "geom/common.hpp"
#include "geom/vector.hpp"
#include "geom/line.hpp"
#include "geom/distance.hpp"

using namespace geom;

template<class T1, class T2>
::testing::AssertionResult geom_equals(const T1& a, const T2& b, double eps = 0) {
    if (eq(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " not equal to " << b << " with tolerance " << eps;
    }
}

template<class T1, class T2>
::testing::AssertionResult geom_not_equals(const T1& a, const T2& b, double eps = 0) {
    if (!eq(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " equal to " << b << " with tolerance " << eps;
    }
}

#define ASSERT_GEOM_EQ(...) EXPECT_TRUE(geom_equals(__VA_ARGS__))
#define ASSERT_GEOM_NE(...) EXPECT_TRUE(geom_not_equals(__VA_ARGS__))

TEST(Vector, addition) {
    Vec2<int> a{1, 2};
    Vec2<int> b{4, 8};
    auto c = a + b;
    ASSERT_GEOM_EQ(c.x, a.x + b.x);
    ASSERT_GEOM_EQ(c.y, a.y + b.y);
    c = a - b;
    ASSERT_GEOM_EQ(c.x, a.x - b.x);
    ASSERT_GEOM_EQ(c.y, a.y - b.y);
}

TEST(Vector, multiplication) {
    Vec2<int> a{42, 84};
    int c = 42;
    auto b = a * c;
    ASSERT_GEOM_EQ(b.x, a.x * c);
    ASSERT_GEOM_EQ(b.y, a.y * c);
    b = a / c;
    ASSERT_GEOM_EQ(b.x, a.x / c);
    ASSERT_GEOM_EQ(b.y, a.y / c);
    b = c * a;
    ASSERT_GEOM_EQ(b.x, a.x * c);
    ASSERT_GEOM_EQ(b.y, a.y * c);
}

TEST(Vector, cast) {
    Vec2<int> a{1, 3};
    Vec2<double> b = a;
    ASSERT_GEOM_EQ(b.x, 1.0);
    ASSERT_GEOM_EQ(b.y, 3.0);
    b.x = 0.3;
    b.y = 1.2;
    a = b;
    ASSERT_GEOM_EQ(a.x, 0);
    ASSERT_GEOM_EQ(a.y, 1);
}

TEST(Vector, compare) {
    Vec2<int> a{1, 3};
    Vec2<double> b{1.001, 2.999};
    ASSERT_GEOM_NE(a, b);
    ASSERT_GEOM_NE(a, b, 1e-5);
    ASSERT_GEOM_EQ(a, b, 1e-2);
}

TEST(Vector, len) {
    Vec2<int> a{1, 1};
    ASSERT_GEOM_EQ(a.len(), std::sqrt(2), 1e-9);
    a = {-1, -1};
    ASSERT_GEOM_EQ(a.len(), std::sqrt(2), 1e-9);
    a = {-1, 1};
    ASSERT_GEOM_EQ(a.len(), std::sqrt(2), 1e-9);
}

TEST(Line, make) {
    Line<int> l(1, 1, -2);

    auto l1 = Line<int>::from_two_points({0, 2}, {1, 1});
    auto l2 = Line<int>::from_point_and_normal({-1, 3}, {1, 1});
    auto l3 = Line<int>::from_point_and_collinear({4, -2}, {-1, 1});

    ASSERT_GEOM_EQ(l, l1);
    ASSERT_GEOM_EQ(l, l2);
    ASSERT_GEOM_EQ(l, l3);
}

TEST(Distance, point_point) {
    Vec2i a{1, 1};
    Vec2i b{0, 2};

    ASSERT_EQ(sqr_dist(a, b), 2);
    ASSERT_GEOM_EQ(dist(a, b), std::sqrt(2.0), 1e-9);
}

TEST(Distance, line_point) {
    Line<int> l(1, 1, 0);
    Vec2i a{1, 1};
    Vec2i b{-1, -1};

    ASSERT_GEOM_EQ(denormalized_dist(l, a), denormalized_dist(a, l));
    ASSERT_GEOM_EQ(dist(l, a), dist(a, l));
    ASSERT_LT(denormalized_dist(l, a) * denormalized_dist(l, b), 0); // a and b on different sides of the line
    ASSERT_GEOM_EQ(dist(a, l), std::sqrt(2.0), 1e-9);
    ASSERT_GEOM_EQ(dist(b, l), std::sqrt(2.0), 1e-9);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}