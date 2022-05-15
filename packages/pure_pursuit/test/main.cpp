#include <gtest/gtest.h>

#include "pure_pursuit/simulator.hpp"
#include "pure_pursuit/speed_planner.hpp"
#include "geom/vector.hpp"
#include "geom/test/near_assert.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace pure_pursuit;

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

    auto path = simulate(start, finish, 10'000'000'000, 1'000'000, 100, model);
    ASSERT_TRUE(path) << errorToString(path.error());
    geom::Vec2d required_finish(finish.pose.pose.position), real_finish(path->back().pose.pose.position);
    ASSERT_GEOM_NEAR(required_finish, real_finish, 0.01);
}

TEST(SpeedPlannerTimePror, as_soon_as_posible) {
    model::Model model(std::getenv("TEST_CONFIG"));
    auto plan = getPlanWithTimePrior(1, 0, 0, 0, model);
    ASSERT_GEOM_NEAR(plan.velocity, model.max_velocity, 1e-6);
    ASSERT_GEOM_NEAR(plan.acceleration, model.max_acceleration, 1e-6);
}

TEST(SpeedPlannerTimePror, smooth) {
    model::Model model(std::getenv("TEST_CONFIG"));
    auto plan = getPlanWithTimePrior(10, 20, 1, 0, model);
    ASSERT_GEOM_NEAR(plan.velocity, 1, 1e-6);
    ASSERT_GEOM_NEAR(plan.acceleration, 0.05, 1e-6);
}

TEST(SpeedPlannerTimePror, slow_down) {
    model::Model model(std::getenv("TEST_CONFIG"));
    auto plan = getPlanWithTimePrior(5, 10, 0, 1, model);
    ASSERT_GEOM_NEAR(plan.velocity, 0, 1e-6);
    ASSERT_GEOM_NEAR(plan.acceleration, -model.max_decceleration, 1e-6);
}

TEST(SpeedPlannerVelocityPrior, max_acceleration) {
    model::Model model(std::getenv("TEST_CONFIG"));
    double time = model.max_velocity / model.max_acceleration / 2;
    auto plan = getPlanWithVelocityPrior(model.max_acceleration / 2 * time * time, time, model.max_velocity, 0, model);
    ASSERT_GEOM_NEAR(plan.velocity, model.max_velocity, 1e-6);
    ASSERT_GEOM_NEAR(plan.acceleration, model.max_acceleration, 1e-6);
}

TEST(SpeedPlannerVelocityPrior, multiphse) {
    model::Model model(std::getenv("TEST_CONFIG"));
    double current_path = 0, current_time = 0;
    double requierd_path = 10, requierd_time = 10;
    double current_velocity = 0;
    double requierd_velocity = 0;
    double dt = 0.001;
    while (current_path < requierd_path) {
        auto plan = getPlanWithVelocityPrior(requierd_path - current_path, requierd_time - current_time, requierd_velocity, current_velocity, model);
        ASSERT_LE(plan.velocity, model.max_velocity);
        ASSERT_LE(plan.acceleration, model.max_acceleration);
        ASSERT_GE(plan.acceleration, -model.max_decceleration);
        current_time += dt;
        current_path += (current_velocity + plan.velocity) / 2 * dt;
        if (plan.velocity < current_velocity + plan.acceleration * dt && plan.velocity > current_velocity)
            current_velocity = plan.velocity;
        else if (plan.velocity > current_velocity + plan.acceleration * dt && plan.velocity < current_velocity)
            current_velocity = plan.velocity;
        else
            current_velocity += plan.acceleration * dt;
        ASSERT_LE(current_time, 100);
    }
    ASSERT_GEOM_NEAR(current_path, requierd_path, 1e-2);
    ASSERT_GEOM_NEAR(current_velocity, requierd_velocity, 1e-2);
    ASSERT_GEOM_NEAR(current_time, requierd_time, 1e-2);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}