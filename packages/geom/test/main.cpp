#include <gtest/gtest.h>

#include "geom/common.hpp"
#include "geom/vector.hpp"
#include "geom/line.hpp"
#include "geom/distance.hpp"

using namespace geom;

template<class T1, class T2>
::testing::AssertionResult geom_near(const T1& a, const T2& b, double eps = 0) {
    if (near(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " not equal to " << b << " with tolerance " << eps;
    }
}

template<class T1, class T2>
::testing::AssertionResult geom_not_near(const T1& a, const T2& b, double eps = 0) {
    if (!near(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " equal to " << b << " with tolerance " << eps;
    }
}

#define ASSERT_GEOM_NEAR(...) EXPECT_TRUE(geom_near(__VA_ARGS__))
#define ASSERT_GEOM_NOT_NEAR(...) EXPECT_TRUE(geom_not_near(__VA_ARGS__))

TEST(Vector, addition) {
    Vec2<int> a{1, 2};
    Vec2<int> b{4, 8};
    auto c = a + b;
    ASSERT_GEOM_NEAR(c.x, a.x + b.x);
    ASSERT_GEOM_NEAR(c.y, a.y + b.y);
    c = a - b;
    ASSERT_GEOM_NEAR(c.x, a.x - b.x);
    ASSERT_GEOM_NEAR(c.y, a.y - b.y);
}

TEST(Vector, multiplication) {
    Vec2<int> a{42, 84};
    int c = 42;
    auto b = a * c;
    ASSERT_GEOM_NEAR(b.x, a.x * c);
    ASSERT_GEOM_NEAR(b.y, a.y * c);
    b = a / c;
    ASSERT_GEOM_NEAR(b.x, a.x / c);
    ASSERT_GEOM_NEAR(b.y, a.y / c);
    b = c * a;
    ASSERT_GEOM_NEAR(b.x, a.x * c);
    ASSERT_GEOM_NEAR(b.y, a.y * c);
}

TEST(Vector, cast) {
    Vec2<int> a{1, 3};
    Vec2<double> b = a;
    ASSERT_GEOM_NEAR(b.x, 1.0);
    ASSERT_GEOM_NEAR(b.y, 3.0);
    b.x = 0.3;
    b.y = 1.2;
    a = b;
    ASSERT_GEOM_NEAR(a.x, 0);
    ASSERT_GEOM_NEAR(a.y, 1);
}

TEST(Vector, compare) {
    Vec2<int> a{1, 3};
    Vec2<double> b{1.001, 2.999};
    ASSERT_GEOM_NOT_NEAR(a, b);
    ASSERT_GEOM_NOT_NEAR(a, b, 1e-5);
    ASSERT_GEOM_NEAR(a, b, 1e-2);
}

TEST(Vector, len) {
    Vec2<int> a{1, 1};
    ASSERT_GEOM_NEAR(a.len(), std::sqrt(2), 1e-9);
    a = {-1, -1};
    ASSERT_GEOM_NEAR(a.len(), std::sqrt(2), 1e-9);
    a = {-1, 1};
    ASSERT_GEOM_NEAR(a.len(), std::sqrt(2), 1e-9);
}

TEST(Line, make) {
    Line<int> l(1, 1, -2);

    auto l1 = Line<int>::from_two_points({0, 2}, {1, 1});
    auto l2 = Line<int>::from_point_and_normal({-1, 3}, {1, 1});
    auto l3 = Line<int>::from_point_and_collinear({4, -2}, {-1, 1});

    ASSERT_GEOM_NEAR(l, l1);
    ASSERT_GEOM_NEAR(l, l2);
    ASSERT_GEOM_NEAR(l, l3);
}

TEST(Distance, point_point) {
    Vec2i a{1, 1};
    Vec2i b{0, 2};

    ASSERT_EQ(sqr_dist(a, b), 2);
    ASSERT_GEOM_NEAR(dist(a, b), std::sqrt(2.0), 1e-9);
}

TEST(Distance, line_point) {
    Line<int> l(1, 1, 0);
    Vec2i a{1, 1};
    Vec2i b{-1, -1};

    ASSERT_GEOM_NEAR(denormalized_dist(l, a), denormalized_dist(a, l));
    ASSERT_GEOM_NEAR(dist(l, a), dist(a, l));
    ASSERT_LT(denormalized_dist(l, a) * denormalized_dist(l, b), 0); // a and b on different sides of the line
    ASSERT_GEOM_NEAR(dist(a, l), std::sqrt(2.0), 1e-9);
    ASSERT_GEOM_NEAR(dist(b, l), std::sqrt(2.0), 1e-9);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}