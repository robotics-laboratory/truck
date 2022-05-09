#pragma once

#include <gtest/gtest.h>

#include "geom/common.hpp"

namespace geom {

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

}

#define ASSERT_GEOM_NEAR(...) EXPECT_TRUE(::geom::geom_near(__VA_ARGS__))
#define ASSERT_GEOM_NOT_NEAR(...) EXPECT_TRUE(::geom::geom_not_near(__VA_ARGS__))
