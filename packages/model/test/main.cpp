#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/vector.h"
#include "model/model.h"
#include "model/params.h"

#include <cmath>

using namespace truck::model;

TEST(Params, yaml_params) {
    // check config
    Params const params("config/model.yaml");
}

TEST(Shape, rearPoseToShapePolygon) {
    Shape shape;
    shape.width = 1.5;
    shape.length = 3;
    const truck::geom::Vec2 rear_center{2, 2};
    const truck::geom::AngleVec2 dir(truck::geom::Angle::fromDegrees(45));
    const truck::geom::Pose rear_pose{rear_center, dir};

    const auto polygon = shape.rearPoseToShapePolygon(rear_pose);

    constexpr double eps = 1e-9;

    EXPECT_EQ(polygon.size(), 4);
    const double d = 3 * std::sqrt(2) / 2;
    const double ax = 2 - d / 4;
    const double ay = 2 + d / 4;
    ASSERT_GEOM_EQUAL(polygon[0].x, ax, eps);
    ASSERT_GEOM_EQUAL(polygon[0].y, ay, eps);
    ASSERT_GEOM_EQUAL(polygon[1].x, ax + d, eps);
    ASSERT_GEOM_EQUAL(polygon[1].y, ay + d, eps);
    ASSERT_GEOM_EQUAL(polygon[2].x, ay + d, eps);
    ASSERT_GEOM_EQUAL(polygon[2].y, ax + d, eps);
    ASSERT_GEOM_EQUAL(polygon[3].x, ay, eps);
    ASSERT_GEOM_EQUAL(polygon[3].y, ax, eps);
}

TEST(Shape, basePoseToShapePolygon) {
    Shape shape;
    shape.width = 1.5;
    shape.length = 3;
    shape.base_to_rear = 1.5;
    const double offset = 1.5 * std::cos(45 * M_PI / 180);
    const truck::geom::Vec2 base_center{2 + offset, 2 + offset};
    const truck::geom::AngleVec2 dir(truck::geom::Angle::fromDegrees(45));
    const truck::geom::Pose base_pose{base_center, dir};

    const auto polygon = shape.basePoseToShapePolygon(base_pose);

    constexpr double eps = 1e-9;

    EXPECT_EQ(polygon.size(), 4);
    const double d = 3 * std::sqrt(2) / 2;
    const double ax = 2 - d / 4;
    const double ay = 2 + d / 4;
    ASSERT_GEOM_EQUAL(polygon[0].x, ax, eps);
    ASSERT_GEOM_EQUAL(polygon[0].y, ay, eps);
    ASSERT_GEOM_EQUAL(polygon[1].x, ax + d, eps);
    ASSERT_GEOM_EQUAL(polygon[1].y, ay + d, eps);
    ASSERT_GEOM_EQUAL(polygon[2].x, ay + d, eps);
    ASSERT_GEOM_EQUAL(polygon[2].y, ax + d, eps);
    ASSERT_GEOM_EQUAL(polygon[3].x, ay, eps);
    ASSERT_GEOM_EQUAL(polygon[3].y, ax, eps);
}

TEST(Twist, angularVelocity) {
    const auto curvature = 1.7;
    const auto velocity = 1.4;
    const Twist twist = {curvature, velocity};

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(twist.angularVelocity(), curvature * velocity, eps);
}

TEST(Model, rearToArbitraryPointTwist) {
    const auto rear_curvature = 1.7;
    const auto rear_velocity = 1.4;
    const truck::geom::Vec2 rear_to_point = {1.2, 0.4};
    const auto expected_curvature = 0.82326627199;
    const auto expected_velocity = 2.89092372781;

    const Twist rear_twist = {rear_curvature, rear_velocity};
    const auto model = truck::model::Model("config/model.yaml");
    const auto target_twist = model.rearToArbitraryPointTwist(rear_twist, rear_to_point);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(target_twist.curvature, expected_curvature, eps);
    ASSERT_GEOM_EQUAL(target_twist.velocity, expected_velocity, eps);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
