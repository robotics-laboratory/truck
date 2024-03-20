#include <gtest/gtest.h>

#include "trajectory_planner/state.h"

#include "common/math.h"

#include "geom/vector.h"
#include "geom/pose.h"

#include "map/map.h"

#include "svg_debug_drawer/sdd.h"

#include <fmt/format.h>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;
using namespace truck::map;

TEST(StateSpace, StatePoses) {
    const auto DrawPoses = [](sdd::SDD& img, const StateSpace& state_space) {
        for (const auto& state : state_space.GetStates()) {
            double scale = std::max(img.GetSize().width, img.GetSize().height) * 0.0025;
            double length = std::max(img.GetSize().width, img.GetSize().height) * 0.0025;
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

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}