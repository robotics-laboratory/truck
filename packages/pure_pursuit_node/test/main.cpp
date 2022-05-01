#include <gtest/gtest.h>

#include "simulator.hpp"

#include <cstdlib>
#include <iostream>

TEST(simulator, just_works) {
    nav_msgs::msg::Odometry start, finish;
    start.pose.pose.position.x = 0;
    start.pose.pose.position.y = 0;

    start.pose.pose.orientation.z = 0;
    start.pose.pose.orientation.y = 0;
    start.pose.pose.orientation.z = 1;
    start.pose.pose.orientation.w = 0;

    finish.pose.pose.position.x = 10;
    finish.pose.pose.position.y = 0;

    finish.pose.pose.orientation.z = 0;
    finish.pose.pose.orientation.y = 0;
    finish.pose.pose.orientation.z = 1;
    finish.pose.pose.orientation.w = 0;

    std::cerr << "Load config from " << std::getenv("TEST_CONFIG") << "\n";

    auto v = pure_pursuit::simulate(start, finish, 1'000'000, 100, model::Model(std::getenv("TEST_CONFIG")));
    ASSERT_GT(v.size(), (size_t) 0);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}