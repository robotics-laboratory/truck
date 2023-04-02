#include <gtest/gtest.h>

#include "common/format.h"
#include "common/exception.h"
#include "common/lock.h"

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

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}