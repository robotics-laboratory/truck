#include <gtest/gtest.h>

#include "common/format.h"
#include "common/exception.h"
#include "common/lock.h"
#include "common/array_as_bit.h"

#include <numeric>
#include <vector>

using namespace truck;

/* Test is the best documentation! */

TEST(fmt, test) {
    ASSERT_EQ(fmt("Hello!"), "Hello!");
    ASSERT_EQ(fmt("%s = %d", "x", 42), "x = 42");
}

TEST(exception, constructor) {
    try {
        throw Exception("Hello, world!");
    } catch (const Exception &e) {
        const std::string what = e.what();
        ASSERT_EQ(what, "Hello, world!");
    }
}

TEST(exception, like_stream) {
    try {
        throw Exception() << "x = " << 42;
    } catch (const Exception &e) {
        const std::string what = e.what();
        ASSERT_EQ(what, "x = 42");
    }
}

TEST(verify, ok) { VERIFY(true); }

TEST(verify, to_throw) { EXPECT_THROW(VERIFY(false), Exception); }

TEST(verify, forward) {
    const int n = 42;
    const int *p = &n;

    EXPECT_EQ(VERIFY(p), p);
}

TEST(verify, stream) {
    try {
        VERIFY_STREAM(false, "x = " << 42);
    } catch (const Exception &e) {
        const std::string what = e.what();
        EXPECT_TRUE(what.ends_with("x = 42"));
        return;
    }

    FAIL();
}

TEST(verify, fmt) {
    try {
        VERIFY_FMT(false, "%s = %d", "x", 42);
    } catch (const Exception &e) {
        const std::string what = e.what();
        ASSERT_TRUE(what.ends_with("x = 42"));
        return;
    }

    FAIL();
}

TEST(lock, test) {
    auto lockable = makeLockable<int>(42);
    auto object = lockable.lock();

    ASSERT_EQ(*object, 42);
}

TEST(ArrayAsBinaryIndexedTree, Build) {
    std::vector<int> arr(42);
    std::iota(arr.begin(), arr.end(), 0);

    auto bit = ArrayAsBinaryIndexedTree(arr.data(), arr.size()).Build();
    for (size_t i = 0; i <= arr.size(); ++i) {
        EXPECT_EQ(bit.Sum(i), i * (i - 1) / 2);
    }
}

TEST(ArrayAsBinaryIndexedTree, Add) {
    std::vector<int> arr(42);
    auto bit = ArrayAsBinaryIndexedTree(arr.data(), arr.size()).Build();

    for (size_t i = 0; i < arr.size(); ++i) {
        bit.Add(i, i);
    }

    for (size_t i = 0; i <= arr.size(); ++i) {
        EXPECT_EQ(bit.Sum(i), i * (i - 1) / 2);
    }
}

TEST(ArrayAsBinaryIndexedTree, Sum) {
    std::vector<int> arr(42);
    std::iota(arr.begin(), arr.end(), 0);

    auto bit = ArrayAsBinaryIndexedTree(arr.data(), arr.size()).Build();

    for (size_t i = 0; i < arr.size(); ++i) {
        for (size_t j = i; j <= arr.size(); ++j) {
            EXPECT_EQ(bit.Sum(i, j), j * (j - 1) / 2 - i * (i - 1) / 2);
        }
    }
}

TEST(ArrayAsBinaryIndexedTree, LowerBound) {
    std::vector<int> arr(42);
    std::iota(arr.begin(), arr.end(), 1);

    auto bit = ArrayAsBinaryIndexedTree(arr.data(), arr.size()).Build();

    for (size_t i = 1; i <= arr.size(); ++i) {
        EXPECT_EQ(bit.LowerBound(static_cast<int>(i * (i + 1)) / 2 - 1), i - 1);
        EXPECT_EQ(bit.LowerBound(static_cast<int>(i * (i + 1)) / 2), i - 1);
        EXPECT_EQ(bit.LowerBound(static_cast<int>(i * (i + 1)) / 2 + 1), i);
    }
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
