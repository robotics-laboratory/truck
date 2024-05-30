#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/angle.h"

using namespace truck::geom;

TEST(Angle, constructor) {
    const auto a = Angle::fromRadians(0);
    const auto b = Angle::fromDegrees(90);
    const auto c = Angle::fromVector(-1, 0);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a, PI0, eps);
    ASSERT_GEOM_EQUAL(b, PI_2, eps);
    ASSERT_GEOM_EQUAL(c, PI, eps);
}

TEST(Angle, coversion) {
    const auto a = PI_2;

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.radians(), M_PI / 2, eps);
    ASSERT_GEOM_EQUAL(a.degrees(), 90.0, eps);
}

TEST(Angle, literals) {
    using namespace truck::geom::literals;

    const auto a = 0_rad;
    const auto b = 90_deg;

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a, PI0, eps);
    ASSERT_GEOM_EQUAL(b, PI_2, eps);
}

TEST(Angle, print) {
    std::stringstream ss;
    ss << Angle::fromDegrees(90);
    ASSERT_EQ(ss.str(), "90.00'");
}

TEST(Angle, normalization) {
    const auto a = Angle::fromDegrees(90);
    const auto b = Angle::fromDegrees(360 + 45);
    const auto c = Angle::fromDegrees(-2 * 360 - 45);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a._0_2PI(), PI_2, eps);
    ASSERT_GEOM_EQUAL(b._0_2PI(), PI_4, eps);
    ASSERT_GEOM_EQUAL(c._0_2PI(), 7 * PI_4, eps);

    ASSERT_GEOM_EQUAL(a._mPI_PI(), PI_2, eps);
    ASSERT_GEOM_EQUAL(b._mPI_PI(), PI_4, eps);
    ASSERT_GEOM_EQUAL(c._mPI_PI(), -PI_4, eps);
}
