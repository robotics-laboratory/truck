#include <gtest/gtest.h>

#include "model/params.h"
#include "model/model.h"
#include "geom/vector.h"
#include "geom/test/equal_assert.h"

using namespace truck::model;

TEST(Params, yaml_params) {
    // check config
    Params params("config/model.yaml");
}

TEST(Twist, angularVelocity) {
    const auto curvature = 1.7;
    const auto velocity = 1.4;
    const Twist twist = { curvature, velocity };

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(twist.angularVelocity(), curvature * velocity, eps);
}

TEST(Model, rearToArbitraryPointTwist) {
    const auto rear_curvature = 1.7;
    const auto rear_velocity = 1.4;
    const truck::geom::Vec2 rear_to_point = { 1.2, 0.4 };
    const auto expected_curvature = 0.82326627199;
    const auto expected_velocity = 2.89092372781;

    const Twist rear_twist = { rear_curvature, rear_velocity };
    const auto model = truck::model::Model("config/model.yaml");
    const auto target_twist = model.rearToArbitraryPointTwist(rear_twist, rear_to_point);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(target_twist.curvature, expected_curvature, eps);
    ASSERT_GEOM_EQUAL(target_twist.velocity, expected_velocity, eps);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
