#include <gtest/gtest.h>

#include "trajectory_planner/planner.h"
#include "trajectory_planner/rtree.h"
#include "trajectory_planner/state.h"

#include "common/math.h"

#include "geom/test/equal_assert.h"
#include "geom/vector.h"
#include "geom/pose.h"

#include "map/map.h"

#include "model/model.h"

#include "svg_debug_drawer/sdd.h"

#include <fmt/format.h>

#include <iostream>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;
using namespace truck::map;
using namespace truck::model;

using namespace std;

namespace {

/*
struct Constraints {
    Discretization<double> x = {.limits = Limits<double>(0, 100), .total_states = 25};
    Discretization<double> y = {.limits = Limits<double>(0, 100), .total_states = 25};
    Discretization<double> yaw = {.limits = Limits<double>(0, 2 * M_PI), .total_states = 7};
    Discretization<double> velocity = {.limits = Limits<double>(0, 0.8), .total_states = 7};
};


States GenerateStates(const Constraints& constraints) {
    States states;
    states.reserve(
        constraints.x.total_states * constraints.y.total_states * constraints.yaw.total_states *
        constraints.velocity.total_states);
    for (size_t i = 0; i < constraints.x.total_states; ++i) {
        for (size_t j = 0; j < constraints.y.total_states; ++j) {
            for (size_t k = 0; k < constraints.yaw.total_states; ++k) {
                for (size_t l = 0; l < constraints.velocity.total_states; ++l) {
                    states.push_back(
                        {.pose =
                             {.pos = Vec2(constraints.x[i], constraints.y[j]),
                              .dir = Angle(constraints.yaw[k])},
                         .velocity = constraints.velocity[l]});
                }
            }
        }
    }
    return states;
}
*/

}  // namespace

TEST(StateSpace, StatePoses) {
    const auto DrawPoses = [](sdd::SDD& img, const StateSpace& state_space) {
        for (const auto& state : state_space.GetStates()) {
            double scale = max(img.GetSize().width, img.GetSize().height) * 0.0025;
            double length = max(img.GetSize().width, img.GetSize().height) * 0.0025;
            auto color = sdd::color::fuchsia;
            if (&state == &state_space.GetStartState()) {
                scale *= 2;
                length *= 2;
                color = sdd::color::red;
            } else if (state_space.GetFinishStates().contains(&state)) {
                scale *= 2;
                length *= 2;
                color = sdd::color::green;
            }
            img.Add(
                sdd::Pose{.pose = state.pose, .scale = scale, .length = length, .color = color});
        }
    };

    {
        const auto map = Map::fromGeoJson("/truck/packages/map/data/map_1.geojson");

        sdd::SDD img(
            {.width = 50, .height = 50},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_1.svg", this->test_info_->name()));
        img.Add(
            sdd::ComplexPolygon{.complex_polygon = map.polygons()[0], .color = sdd::color::white});

        const auto ego_pose = Pose(Vec2(30, 25), AngleVec2::fromVector(Vec2(1, -1)));

        const auto route =
            Polyline{Vec2(10, 30), Vec2(20, 27), Vec2(25, 25), Vec2(33, 20), Vec2(38, 13)};
        img.Add(sdd::Polyline{.polyline = route, .thickness = 0.25, .color = sdd::color::blue});

        auto state_space =
            StateSpace(
                {.longitude = {.limits = Limits<double>(-5, 16), .total_states = 10},
                 .latitude = {.limits = Limits<double>(-5, 6), .total_states = 10},
                 .forward_yaw = {.limits = Limits<Angle>(-PI_2, PI_2), .total_states = 5},
                 .backward_yaw = {.limits = Limits<Angle>(PI_2, 3 * PI_2), .total_states = 3},
                 .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}})
                .Build(
                    {.pose = ego_pose, .velocity = 0.0},
                    {.base_state =
                         {.pose = geom::Pose(Vec2(37, 15), AngleVec2::fromVector(Vec2(1, -1))),
                          .velocity = 0.5},
                     .x_range = Limits<double>(-1, 1),
                     .y_range = Limits<double>(-1, 1),
                     .yaw_range = Limits<Angle>(-PI_4, PI_4),
                     .velocity_range = Limits<double>(-0.2, 0.2)},
                    route);

        DrawPoses(img, state_space);
    }
    {
        const auto map = Map::fromGeoJson("/truck/packages/map/data/map_2.geojson");

        sdd::SDD img(
            {.width = 100, .height = 100},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_2.svg", this->test_info_->name()));
        img.Add(
            sdd::ComplexPolygon{.complex_polygon = map.polygons()[0], .color = sdd::color::white});

        const auto ego_pose = Pose(Vec2(30, 20), AngleVec2::fromVector(Vec2(1, -1)));

        const auto route =
            Polyline{Vec2(10, 40), Vec2(20, 30), Vec2(25, 20), Vec2(35, 15), Vec2(50, 10)};
        img.Add(sdd::Polyline{.polyline = route, .thickness = 0.5, .color = sdd::color::blue});

        auto state_space =
            StateSpace(
                {.longitude = {.limits = Limits<double>(-5, 16), .total_states = 10},
                 .latitude = {.limits = Limits<double>(-5, 6), .total_states = 10},
                 .forward_yaw = {.limits = Limits<Angle>(-PI_2, PI_2), .total_states = 5},
                 .backward_yaw = {.limits = Limits<Angle>(PI_2, 3 * PI_2), .total_states = 3},
                 .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}})
                .Build(
                    {.pose = ego_pose, .velocity = 0.0},
                    {.base_state =
                         {.pose = geom::Pose(Vec2(42, 13), AngleVec2::fromVector(Vec2(1, -1))),
                          .velocity = 0.5},
                     .x_range = Limits<double>(-1, 1),
                     .y_range = Limits<double>(-1, 1),
                     .yaw_range = Limits<Angle>(-PI_4, PI_4),
                     .velocity_range = Limits<double>(-0.2, 0.2)},
                    route);

        DrawPoses(img, state_space);
    }
}

TEST(Planner, HeuriscticCost) {
    constexpr double eps = 1e-4;

    auto state_space = StateSpace({}).Build(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.base_state =
             {.pose = geom::Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5},
         .x_range = Limits<double>(-1, 1),
         .y_range = Limits<double>(-1, 1),
         .yaw_range = Limits<Angle>(-PI_4, PI_4),
         .velocity_range = Limits<double>(-0.2, 0.2)},
        Polyline{Vec2(0, 0), Vec2(1, 0)});
    auto planner = Planner({}, Model("/truck/packages/model/config/model.yaml")).Build(state_space);
    EXPECT_DOUBLE_EQ(
        *planner.HeuristicCost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        0.0);
    EXPECT_FALSE(planner.HeuristicCost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.8}));
    EXPECT_FALSE(planner.HeuristicCost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.0}));
    ASSERT_GEOM_EQUAL(
        *planner.HeuristicCost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5},
            {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.5}),
        3.2015621187,
        eps);
    ASSERT_GEOM_EQUAL(
        *planner.HeuristicCost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
            {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.8}),
        4.0019526483,
        eps);
    EXPECT_FALSE(planner.HeuristicCost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 1000}));
    EXPECT_FALSE(planner.HeuristicCost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 1), AngleVec2::fromVector(Vec2(-1, 0))), .velocity = 0.5}));
}

TEST(Planner, Cost) {
    constexpr double eps = 1e-4;

    auto state_space = StateSpace({}).Build(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.base_state =
             {.pose = geom::Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5},
         .x_range = Limits<double>(-1, 1),
         .y_range = Limits<double>(-1, 1),
         .yaw_range = Limits<Angle>(-PI_4, PI_4),
         .velocity_range = Limits<double>(-0.2, 0.2)},
        Polyline{Vec2(0, 0), Vec2(1, 0)});
    auto planner = Planner({}, Model("/truck/packages/model/config/model.yaml")).Build(state_space);
    EXPECT_DOUBLE_EQ(
        *planner.Cost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        0.0);
    EXPECT_FALSE(planner.Cost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.8}));
    EXPECT_FALSE(planner.Cost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.0}));
    ASSERT_GEOM_EQUAL(
        *planner.Cost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5},
            {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.5}),
        3.2854678967,
        eps);
    ASSERT_GEOM_EQUAL(
        *planner.Cost(
            {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
            {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.8}),
        4.1068348709,
        eps);
    EXPECT_FALSE(planner.Cost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 1000}));
    EXPECT_FALSE(planner.Cost(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.pose = Pose(Vec2(0, 1), AngleVec2::fromVector(Vec2(-1, 0))), .velocity = 0.5}));
}

TEST(Planner, HeuristicCostFromStart) {
    auto state_space = StateSpace({}).Build(
        {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
        {.base_state =
             {.pose = geom::Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5},
         .x_range = Limits<double>(-1, 1),
         .y_range = Limits<double>(-1, 1),
         .yaw_range = Limits<Angle>(-PI_4, PI_4),
         .velocity_range = Limits<double>(-0.2, 0.2)},
        Polyline{Vec2(0, 0), Vec2(1, 0)});
    auto planner = Planner({}, Model("/truck/packages/model/config/model.yaml")).Build(state_space);

    EXPECT_DOUBLE_EQ(
        *planner.HeuristicCostFromStart(
            {.pose = Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        1.85);
    EXPECT_DOUBLE_EQ(
        *planner.HeuristicCostFromStart(
            {.pose = Pose(Vec2(0.1875, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        0.75);
}

/*
TEST(RTree, Search) {
    const auto constraints = Constraints();

    auto states = GenerateStates(constraints);

    SpatioTemporalRTree rtree(constraints.velocity);

    for (size_t i = 0; i < states.size(); ++i) {
        rtree.Add(states[i]);
    }

    {
        std::vector<std::pair<double, const State*>> result;
        for (size_t i = 0; i < states.size(); ++i) {
            const double radius = 1;

            result.clear();
            rtree.RangeSearch(states[i], radius, result);

            ASSERT_GE(result.size(), 1);
            ASSERT_TRUE(std::find_if(result.begin(), result.end(), [&states, &i](const auto& p) {
                            return p.second == &states[i];
                        }) != result.end());
        }
    }
    {
        std::vector<std::pair<double, const State*>> result;
        for (size_t i = 0; i < states.size(); ++i) {
            const double radius = 20;

            result.clear();
            rtree.RangeSearch(states[i], radius, result);

            ASSERT_GE(result.size(), 2);
            ASSERT_TRUE(std::find_if(result.begin(), result.end(), [&states, &i](const auto& p) {
                            return p.second == &states[i];
                        }) != result.end());
        }
    }
}
*/

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}