#include <gtest/gtest.h>

#include "trajectory_planner/state.h"

#include "common/math.h"

#include "geom/test/equal_assert.h"
#include "geom/vector.h"
#include "geom/pose.h"

#include "map/map.h"

#include "model/model.h"

#include "svg_debug_drawer/sdd.h"

#include <fmt/format.h>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;
using namespace truck::map;
using namespace truck::model;

using namespace std;

TEST(StateSpace, StatePoses) {
    const auto draw_poses = [](sdd::SDD& img, const StateSpace& state_space) {
        for (int i = 0; i < state_space.regular_states.size; ++i) {
            img.Add(sdd::Pose{
                .pose = state_space.regular_states.data[i].pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.0025,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.0025,
                .color = sdd::color::fuchsia});
        }
        for (int i = 0; i < state_space.start_states.size; ++i) {
            img.Add(sdd::Pose{
                .pose = state_space.start_states.data[i].pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .color = sdd::color::red});
        }
        for (int i = 0; i < state_space.finish_states.size; ++i) {
            img.Add(sdd::Pose{
                .pose = state_space.finish_states.data[i].pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .color = sdd::color::green});
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

        auto state_space_holder = MakeStateSpace(
            {.longitude = {.limits = Limits<double>(-5, 16), .total_states = 20},
             .latitude = {.limits = Limits<double>(-5, 6), .total_states = 20},
             .total_forward_yaw_states = 5,
             .total_backward_yaw_states = 3,
             .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}});
        state_space_holder.state_space.Build(
            {.pose = ego_pose, .velocity = 0.0},
            {.base_state =
                 {.pose = geom::Pose(Vec2(37, 15), AngleVec2::fromVector(Vec2(1, -1))),
                  .velocity = 0.5},
             .x_range = Limits<double>(-1, 1),
             .y_range = Limits<double>(-1, 1),
             .yaw_range = Limits<Angle>(-PI_4, PI_4),
             .velocity_range = Limits<double>(-0.2, 0.2)},
            route);

        draw_poses(img, state_space_holder.state_space);
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

        auto state_space_holder = MakeStateSpace(
            {.longitude = {.limits = Limits<double>(-5, 16), .total_states = 20},
             .latitude = {.limits = Limits<double>(-5, 6), .total_states = 20},
             .total_forward_yaw_states = 5,
             .total_backward_yaw_states = 3,
             .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}});
        state_space_holder.state_space.Build(
            {.pose = ego_pose, .velocity = 0.0},
            {.base_state =
                 {.pose = geom::Pose(Vec2(42, 13), AngleVec2::fromVector(Vec2(1, -1))),
                  .velocity = 0.5},
             .x_range = Limits<double>(-1, 1),
             .y_range = Limits<double>(-1, 1),
             .yaw_range = Limits<Angle>(-PI_4, PI_4),
             .velocity_range = Limits<double>(-0.2, 0.2)},
            route);

        draw_poses(img, state_space_holder.state_space);
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}