#include <gtest/gtest.h>

#include "trajectory_planner/planner.h"
#include "trajectory_planner/state.h"
#include "trajectory_planner/rtree.h"

#include "common/math.h"

#include "geom/vector.h"
#include "geom/pose.h"

#include "map/map.h"

#include "svg_debug_drawer/sdd.h"

#include <fmt/format.h>

#include <iostream>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;
using namespace truck::map;

namespace {
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

}  // namespace

TEST(Planner, StatePoses) {
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

        auto planner = Planner({.track_height = 20,
                                .track_width = 10,
                                .longitude_ratio = 0.3,
                                .longitude_discretization = 10,
                                .latitude_discretization = 10,
                                .backward_yaw_discretization = 3})
                           .Build(ego_pose, route);
        for (const auto& state_pose : planner.GetStatePoses()) {
            img.Add(sdd::Pose{
                .pose = state_pose, .scale = 0.25, .length = 0.25, .color = sdd::color::fuchsia});
        }

        img.Add(
            sdd::Pose{.pose = ego_pose, .scale = 0.25, .length = 0.25, .color = sdd::color::red});
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

        auto planner = Planner({.track_height = 20,
                                .track_width = 10,
                                .longitude_ratio = 0.3,
                                .longitude_discretization = 10,
                                .latitude_discretization = 10,
                                .backward_yaw_discretization = 3})
                           .Build(ego_pose, route);
        for (const auto& state_pose : planner.GetStatePoses()) {
            img.Add(sdd::Pose{
                .pose = state_pose, .scale = 0.25, .length = 0.25, .color = sdd::color::fuchsia});
        }

        img.Add(sdd::Pose{.pose = ego_pose, .scale = 0.5, .length = 0.5, .color = sdd::color::red});
    }
}

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

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}