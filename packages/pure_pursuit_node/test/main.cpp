#include <gtest/gtest.h>

#include "simulator.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>

TEST(simulator, just_works) {
    model::Model model(std::getenv("TEST_CONFIG"));

    nav_msgs::msg::Odometry start, finish;
    start.pose.pose.position.x = 0;
    start.pose.pose.position.y = 0;

    start.pose.pose.orientation.z = 0;
    start.pose.pose.orientation.y = 0;
    start.pose.pose.orientation.z = 0;
    start.pose.pose.orientation.w = 1;

    finish.pose.pose.position.x = model.lookahead_distance / 2;
    finish.pose.pose.position.y = 0;

    finish.pose.pose.orientation.z = 0;
    finish.pose.pose.orientation.y = 0;
    finish.pose.pose.orientation.z = 0;
    finish.pose.pose.orientation.w = 1;

    auto path = pure_pursuit::simulate(start, finish, 10'000'000'000, 1'000'000, 100, model);
    ASSERT_TRUE(path) << error_to_string(path.error());
    // ASSERT_LT(std::abs(v.back().pose.pose.x - finish.pose.pose.x));
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}