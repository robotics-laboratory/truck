#include <gtest/gtest.h>

#include "geom/common.hpp"
#include "geom/vector.hpp"
#include "geom/line.hpp"
#include "geom/distance.hpp"
#include "geom/arc.hpp"

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
    b = c * a;
    ASSERT_GEOM_NEAR(b.x, a.x * c);
    ASSERT_GEOM_NEAR(b.y, a.y * c);
}

TEST(Vector, division) {
    Vec2<int> a{1, 2};
    int c = 42;
    auto b = a / c;
    static_assert(std::is_same_v<decltype(b), Vec2d>);
    ASSERT_GEOM_NEAR(b.x, (double) a.x / c);
    ASSERT_GEOM_NEAR(b.y, (double) a.y / c);
}

template<class T1, class T2>
constexpr void checkExplicitCast() {
    static_assert(std::is_convertible_v<T1, T2> && !std::is_convertible_v<T2, T1>);
}

TEST(Vector, cast) {
    checkExplicitCast<Vec2i, Vec2f>();
    checkExplicitCast<Vec2i, Vec2d>();
    checkExplicitCast<Vec2f, Vec2d>();
    checkExplicitCast<Vec2i32, Vec2i64>();
    Vec2<int> a{1, 3};
    Vec2<double> b = a;
    ASSERT_GEOM_NEAR(b.x, 1.0);
    ASSERT_GEOM_NEAR(b.y, 3.0);
    b.x = 0.3;
    b.y = 1.2;
    a = static_cast<Vec2i>(b);
    ASSERT_GEOM_NEAR(a.x, 0);
    ASSERT_GEOM_NEAR(a.y, 1);
}

TEST(Vector, operators_with_cast) {
    Vec2i a{1, 2};
    Vec2d b{0.5, 0.5};
    auto c = a + b;
    auto d = b + a;
    static_assert(std::is_same_v<decltype(c), Vec2d> && std::is_same_v<decltype(d), Vec2d>);
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

TEST(Vector, rotate) {
    Vec2d v{1, 1};
    ASSERT_GEOM_NEAR(v.rotate(M_PI / 2), Vec2d{-1, 1}, 1e-9);
    ASSERT_GEOM_NEAR(v.rotate(-M_PI / 2), Vec2d{1, -1}, 1e-9);
    ASSERT_GEOM_NEAR(v.rotate(M_PI), -v, 1e-9);
    ASSERT_GEOM_NEAR(v.rotate(M_PI * 4), v, 1e-9);
}

TEST(Vector, radians) {
    ASSERT_GEOM_NEAR(Vec2d{1, 1}.radians(), M_PI / 4, 1e-9);
    ASSERT_GEOM_NEAR(Vec2d{-10, 0}.radians(), M_PI, 1e-9);
    ASSERT_GEOM_NEAR(Vec2d{0, -0.5}.radians(), -M_PI / 2, 1e-9);
}

TEST(Line, make) {
    Line<int> l(1, 1, -2);

    auto l1 = Line<int>::fromTwoPoints({0, 2}, {1, 1});
    auto l2 = Line<int>::fromPointAndNormal({-1, 3}, {1, 1});
    auto l3 = Line<int>::fromPointAndCollinear({4, -2}, {-1, 1});

    ASSERT_GEOM_NEAR(l, l1);
    ASSERT_GEOM_NEAR(l, l2);
    ASSERT_GEOM_NEAR(l, l3);
}

TEST(Distance, point_point) {
    Vec2i a{1, 1};
    Vec2i b{0, 2};

    ASSERT_EQ(distSq(a, b), 2);
    ASSERT_GEOM_NEAR(dist(a, b), std::sqrt(2.0), 1e-9);
}

TEST(Distance, line_point) {
    Line<int> l(1, 1, 0);
    Vec2i a{1, 1};
    Vec2i b{-1, -1};

    ASSERT_GEOM_NEAR(denormalizedDist(l, a), denormalizedDist(a, l));
    ASSERT_GEOM_NEAR(dist(l, a), dist(a, l));
    ASSERT_LT(denormalizedDist(l, a) * denormalizedDist(l, b), 0); // a and b on different sides of the line
    ASSERT_GEOM_NEAR(dist(a, l), std::sqrt(2.0), 1e-9);
    ASSERT_GEOM_NEAR(dist(b, l), std::sqrt(2.0), 1e-9);
}

TEST(Arc, make) {
    Vec2d start{0, 2}, finish{4, 2};
    std::optional<Arc> arc = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{1, 0});
    ASSERT_TRUE(arc);
    ASSERT_EQ(arc->getRadius(), std::numeric_limits<double>::infinity());
    ASSERT_GEOM_NEAR(arc->getLength(), 4, 1e-9);
    ASSERT_GEOM_NEAR(arc->getAngle(), 0, 1e-9);
    ASSERT_GEOM_NEAR(arc->getStart(), start, 1e-9);
    ASSERT_GEOM_NEAR(arc->getFinish(), finish, 1e-9);
    ASSERT_GEOM_NEAR(arc->getPoint(0), start, 1e-9);
    ASSERT_GEOM_NEAR(arc->getPoint(1), finish, 1e-9);
    ASSERT_GEOM_NEAR(arc->getPoint(0.5), Vec2d{2, 2}, 1e-9);
    ASSERT_EQ(arc->getDirection(), Arc::Direction::STRIGHT);
    arc = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{0, 1});
    ASSERT_TRUE(arc);
    ASSERT_GEOM_NEAR(arc->getRadius(), 2, 1e-9);
    ASSERT_GEOM_NEAR(arc->getLength(), 2 * M_PI, 1e-9);
    ASSERT_GEOM_NEAR(arc->getCenter(), Vec2d{2, 2}, 1e-9);
    ASSERT_GEOM_NEAR(arc->getAngle(), M_PI, 1e-9);
    ASSERT_GEOM_NEAR(arc->getPoint(0.5), Vec2d{2, 4}, 1e-9);
    ASSERT_EQ(arc->getDirection(), Arc::Direction::LEFT);
    arc = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{1, 1});
    ASSERT_TRUE(arc);
    ASSERT_GEOM_NEAR(arc->getRadius(), std::sqrt(8), 1e-9);
    ASSERT_GEOM_NEAR(arc->getLength(), std::sqrt(2) * M_PI, 1e-9);
    ASSERT_GEOM_NEAR(arc->getCenter(), Vec2d{2, 0}, 1e-9);
    ASSERT_GEOM_NEAR(arc->getAngle(), M_PI / 2, 1e-9);
    ASSERT_GEOM_NEAR(arc->getPoint(0.5), Vec2d{2, std::sqrt(8)}, 1e-9);
    ASSERT_EQ(arc->getDirection(), Arc::Direction::LEFT);
    arc = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{-1, 0});
    ASSERT_FALSE(arc);
}

TEST(Arc, unnormalized_tangental) {
    Vec2d start{0, 10}, finish{5, 12};
    auto arc1 = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{1, 1});
    auto arc2 = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{10, 10});
    auto arc3 = Arc::fromTwoPointsAndTangentalVector(start, finish, Vec2d{0.5, 0.5});
    auto arc4 = Arc::fromTwoPointsAndTangentalVector(start, finish, -Vec2d{1, 1});
    ASSERT_TRUE(arc1);
    ASSERT_TRUE(arc2);
    ASSERT_TRUE(arc3);
    ASSERT_TRUE(arc4);
    ASSERT_GEOM_NEAR(*arc1, *arc2, 1e-9);
    ASSERT_GEOM_NEAR(*arc1, *arc3, 1e-9);
    ASSERT_GEOM_NOT_NEAR(*arc1, *arc4, 1e-2);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}