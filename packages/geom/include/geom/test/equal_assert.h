#pragma once

#include <gtest/gtest.h>

#include "geom/common.h"

namespace truck::geom {

template<class T>
::testing::AssertionResult geomEqual(const T& a, const T& b, double eps = 0) {
    if (equal(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " != " << b << " eps=" << eps;
    }
}

template<class T>
::testing::AssertionResult geomNotEqual(const T& a, const T& b, double eps = 0) {
    if (!equal(a, b, eps)) {
        return ::testing::AssertionSuccess();
    } else {
        return ::testing::AssertionFailure() << a << " == " << b << " eps= " << eps;
    }
}

}  // namespace truck::geom

#define ASSERT_GEOM_EQUAL(...) EXPECT_TRUE(::truck::geom::geomEqual(__VA_ARGS__))
#define ASSERT_GEOM_NOT_EQUAL(...) EXPECT_TRUE(::truck::geom::geomNotEqual(__VA_ARGS__))
