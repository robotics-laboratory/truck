#include <gtest/gtest.h>

#include "trajectory_planner/planner.h"
#include "trajectory_planner/rtree.h"
#include "trajectory_planner/sampler.h"
#include "trajectory_planner/state.h"
#include "trajectory_planner/tree.h"

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
    {
        const auto map = Map::fromGeoJson("/truck/packages/map/data/map_1.geojson");

        sdd::SDD img(
            {.width = 50, .height = 50},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_3.svg", this->test_info_->name()));
        img.Add(
            sdd::ComplexPolygon{.complex_polygon = map.polygons()[0], .color = sdd::color::white});

        const auto ego_pose = Pose(Vec2(28.5, 22.5), AngleVec2::fromVector(Vec2(1, -1)));

        const auto route =
            Polyline{Vec2(10, 30), Vec2(20, 27), Vec2(25, 25), Vec2(33, 20), Vec2(38, 13)};
        img.Add(sdd::Polyline{.polyline = route, .thickness = 0.25, .color = sdd::color::blue});

        auto state_space_holder = MakeStateSpace(
            {.longitude = {.limits = Limits<double>(-2, 3), .total_states = 5},
             .latitude = {.limits = Limits<double>(-2, 3), .total_states = 5},
             .total_forward_yaw_states = 5,
             .total_backward_yaw_states = 3,
             .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}});
        state_space_holder.state_space.Build(
            {.pose = ego_pose, .velocity = 0.0},
            {.base_state =
                 {.pose = geom::Pose(Vec2(30, 21), AngleVec2::fromVector(Vec2(1, -1))),
                  .velocity = 0.5},
             .x_range = Limits<double>(-1, 1),
             .y_range = Limits<double>(-1, 1),
             .yaw_range = Limits<Angle>(-PI_4, PI_4),
             .velocity_range = Limits<double>(-0.2, 0.2)},
            route);

        draw_poses(img, state_space_holder.state_space);
    }
}

TEST(Node, Estimator) {
    auto start_state =
        State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
    auto finish_state =
        State{.pose = Pose(Vec2(0.1875, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
    auto model = model::Model("/truck/packages/model/config/model.yaml");
    auto truck_state = TruckState();
    truck_state.model = &model;

    auto state_space = StateSpace{
        .start_states = {.data = &start_state, .size = 1},
        .finish_states = {.data = &finish_state, .size = 1},
        .regular_states = {.data = nullptr, .size = 0},
        .truck_state = truck_state};
    auto node_estimator = Node::Estimator().Reset(state_space);
    EXPECT_DOUBLE_EQ(
        node_estimator.HeuristicCostFromStart(
            State{.pose = Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        1.85);
    EXPECT_DOUBLE_EQ(
        node_estimator.HeuristicCostToFinish(
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0}),
        0.75);
}

TEST(Edge, Estimator) {
    constexpr double eps = 1e-4;
    const double inf = std::numeric_limits<double>::max() * 0.33;

    auto model = model::Model("/truck/packages/model/config/model.yaml");
    auto truck_state = TruckState();
    truck_state.model = &model;

    auto edge_estimator = Edge::Estimator(3, 0.05).Reset({.truck_state = truck_state});

    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}), 0.0, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.8};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.0};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5};
        auto to_state =
            State{.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.5};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}),
            3.2015621187,
            eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(1, 1), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.8};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}),
            4.0019526483,
            eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(5, 1), AngleVec2::fromVector(Vec2(2, 1))), .velocity = 1000};
        ASSERT_GEOM_EQUAL(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 1), AngleVec2::fromVector(Vec2(-1, 0))), .velocity = 0.5};
        EXPECT_LE(
            edge_estimator.HeuristicCost({.state = &from_state}, {.state = &to_state}), inf - eps);
    }

    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), 0.0, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.8};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(0, 1))), .velocity = 0.0};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.5};
        auto to_state =
            State{.pose = Pose(Vec2(5, 1), AngleVec2::fromVector(Vec2(2, 1))), .velocity = 0.5};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), 10.373307661, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(5, 1), AngleVec2::fromVector(Vec2(2, 1))), .velocity = 0.8};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), 12.966634576, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(5, 1), AngleVec2::fromVector(Vec2(2, 1))), .velocity = 1000};
        ASSERT_GEOM_EQUAL(
            edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), inf, eps);
    }
    {
        auto from_state =
            State{.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0};
        auto to_state =
            State{.pose = Pose(Vec2(0, 1), AngleVec2::fromVector(Vec2(-1, 0))), .velocity = 0.5};
        EXPECT_LE(edge_estimator.Cost({.state = &from_state}, {.state = &to_state}), inf - eps);
    }
}

TEST(RTree, Search) {
    auto state_space_holder = MakeStateSpace(
        {.longitude = {.limits = Limits<double>(0, 100), .total_states = 25},
         .latitude = {.limits = Limits<double>(0, 100), .total_states = 25},
         .total_forward_yaw_states = 4,
         .total_backward_yaw_states = 4,
         .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 7}});
    const auto& params = state_space_holder.state_space.params;
    int pos = 0;

    auto nodes = MakeNodes(params.Size());
    for (int i = 0; i < params.longitude.total_states; ++i) {
        for (int j = 0; j < params.latitude.total_states; ++j) {
            Discretization<geom::Angle> forward_yaw = {
                .limits = Limits<geom::Angle>(-geom::PI_2, geom::PI_2),
                .total_states = params.total_forward_yaw_states};

            for (int k = 0; k < params.total_forward_yaw_states; ++k) {
                for (int l = 0; l < params.velocity.total_states; ++l) {
                    state_space_holder.states_ptr[pos] = {
                        .pose = Pose(Vec2(params.longitude[i], params.latitude[j]), forward_yaw[k]),
                        .velocity = params.velocity[l]};
                    nodes.nodes.AddNode(state_space_holder.states_ptr[pos], Node::Type::REGULAR);
                    ++pos;
                }
            }

            Discretization<geom::Angle> backward_yaw = {
                .limits = Limits<geom::Angle>(geom::PI_2, 3 * geom::PI_2),
                .total_states = params.total_backward_yaw_states};

            for (int k = 0; k < params.total_backward_yaw_states; ++k) {
                for (int l = 0; l < params.velocity.total_states; ++l) {
                    state_space_holder.states_ptr[pos] = {
                        .pose =
                            Pose(Vec2(params.longitude[i], params.latitude[j]), backward_yaw[k]),
                        .velocity = params.velocity[l]};
                    nodes.nodes.AddNode(state_space_holder.states_ptr[pos], Node::Type::REGULAR);
                    ++pos;
                }
            }
        }
    }

    auto model = model::Model("/truck/packages/model/config/model.yaml");
    auto truck_state = TruckState();
    truck_state.model = &model;

    auto edge_estimator = Edge::Estimator(3, 0.05).Reset({.truck_state = truck_state});

    SpatioTemporalRTree rtree(params.velocity, edge_estimator);

    for (int i = 0; i < nodes.nodes.size; ++i) {
        rtree.Add(nodes.nodes.data[i]);
    }

    {
        for (int i = 0; i < nodes.nodes.size; ++i) {
            const double radius = 1;

            const auto& result = rtree.RangeSearch(nodes.nodes.data[i], radius);

            ASSERT_GE(result.size(), 1);
            ASSERT_TRUE(std::find_if(result.begin(), result.end(), [&](const auto& p) {
                            return p.second == (nodes.nodes.data + i);
                        }) != result.end());
        }
    }
    {
        for (int i = 0; i < nodes.nodes.size; ++i) {
            const double radius = 20;

            const auto& result = rtree.RangeSearch(nodes.nodes.data[i], radius);

            ASSERT_GE(result.size(), 2);
            ASSERT_TRUE(std::find_if(result.begin(), result.end(), [&](const auto& p) {
                            return p.second == (nodes.nodes.data + i);
                        }) != result.end());
        }
    }

    for (int i = 0; i < nodes.nodes.size; ++i) {
        rtree.Remove(nodes.nodes.data[i]);
    }
}

TEST(Sampler, Sample) {
    Sampler sampler(2);

    {
        auto nodes = MakeNodes(2);
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 0.5;
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 0.5;

        sampler.Build(nodes.nodes);

        EXPECT_FALSE(sampler.Empty());

        const auto& node_1 = sampler.Sample();
        sampler.Remove(node_1);
        const auto& node_2 = sampler.Sample();
        sampler.Remove(node_2);

        EXPECT_TRUE(&node_1 == nodes.nodes.data || &node_1 == &nodes.nodes.data[1]);
        EXPECT_TRUE(&node_2 == nodes.nodes.data || &node_2 == &nodes.nodes.data[1]);
        EXPECT_NE(&node_1, &node_2);
        EXPECT_TRUE(sampler.Empty());
    }
    {
        auto nodes = MakeNodes(2);
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 0.0;
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 1.0;

        sampler.Build(nodes.nodes);

        EXPECT_FALSE(sampler.Empty());
        EXPECT_EQ(sampler.Size(), 1);

        const auto& node_1 = sampler.Sample();
        sampler.Remove(node_1);

        EXPECT_EQ(&node_1, &nodes.nodes.data[1]);
        EXPECT_TRUE(sampler.Empty());
    }
    {
        auto nodes = MakeNodes(2);
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(0, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 1.0;
        nodes.nodes
            .AddNode(
                {.pose = Pose(Vec2(1, 0), AngleVec2::fromVector(Vec2(1, 0))), .velocity = 0.0},
                Node::Type::REGULAR)
            .probability = 0.0;

        sampler.Build(nodes.nodes);

        EXPECT_FALSE(sampler.Empty());
        EXPECT_EQ(sampler.Size(), 1);

        const auto& node_1 = sampler.Sample();
        sampler.Remove(node_1);

        EXPECT_EQ(&node_1, nodes.nodes.data);
        EXPECT_TRUE(sampler.Empty());
    }
}

TEST(Planner, Search) {
    const auto draw_motion = [](sdd::SDD& img, const Pose& from, const Pose& to) {
        const auto dist = truck::geom::distance(from.pos, to.pos);
        const double gamma = dist * 0.5;
        const truck::geom::Vec2 from_ref = from.pos + from.dir * gamma;
        const truck::geom::Vec2 to_ref = to.pos - to.dir * gamma;
        img.Add(sdd::Bezier{
            .points = {from.pos, from_ref, to_ref, to.pos},
            .thickness = 0.1,
            .color = sdd::color::gray});
    };

    const auto draw_geom_profile = [&draw_motion](sdd::SDD& img, const Plan& plan) {
        for (auto it = plan.begin(); it + 1 != plan.end(); ++it) {
            draw_motion(img, (*it)->state->pose, (*(it + 1))->state->pose);
        }
        {
            img.Add(sdd::Pose{
                .pose = plan.front()->state->pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .color = sdd::color::red});
        }
        {
            img.Add(sdd::Pose{
                .pose = plan.back()->state->pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .color = sdd::color::green});
        }
        for (auto it = plan.begin() + 1; it + 1 != plan.end(); ++it) {
            img.Add(sdd::Pose{
                .pose = (*it)->state->pose,
                .scale = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .length = max(img.GetSize().width, img.GetSize().height) * 0.005,
                .color = sdd::color::fuchsia});
        }
    };

    const auto draw_velocity_profile = [](sdd::SDD& img, const Plan& plan) {
        for (auto it = plan.begin(); it != plan.end(); ++it) {
            img.Add(sdd::Marker{
                .point = Vec2((*it)->cost_to_come, (*it)->state->velocity),
                .shape = sdd::Marker::Shape::Circle,
                .scale = 0.5,
                .color = sdd::color::red});
        }
    };

    {
        const auto map = Map::fromGeoJson("/truck/packages/map/data/map_1.geojson");

        sdd::SDD img_geom(
            {.width = 50, .height = 50},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_Geom_1.svg",
                this->test_info_->name()));
        img_geom.Add(
            sdd::ComplexPolygon{.complex_polygon = map.polygons()[0], .color = sdd::color::white});

        const auto ego_pose = Pose(Vec2(30, 25), AngleVec2::fromVector(Vec2(1, -1)));

        const auto route =
            Polyline{Vec2(10, 30), Vec2(20, 27), Vec2(25, 25), Vec2(33, 20), Vec2(38, 13)};
        img_geom.Add(
            sdd::Polyline{.polyline = route, .thickness = 0.25, .color = sdd::color::blue});

        auto model =
            std::make_shared<model::Model>(model::Model("/truck/packages/model/config/model.yaml"));

        auto collision_checker =
            std::make_shared<collision::StaticCollisionChecker>(model->shape());

        Planner planner(
            {.truck_state_params = {.min_dist_to_obstacle = 0.1},
             .state_space_params = {
                 .longitude = {.limits = Limits<double>(-5, 16), .total_states = 20},
                 .latitude = {.limits = Limits<double>(-5, 6), .total_states = 20},
                 .total_forward_yaw_states = 5,
                 .total_backward_yaw_states = 3,
                 .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}}});
        planner.SetModel(model).SetCollisionChecker(collision_checker);

        const auto& plan =
            planner
                .Build(
                    {.pose = ego_pose, .velocity = 0.0},
                    {.base_state =
                         {.pose = geom::Pose(Vec2(37, 15), AngleVec2::fromVector(Vec2(1, -1))),
                          .velocity = 0.5},
                     .x_range = Limits<double>(-1, 1),
                     .y_range = Limits<double>(-1, 1),
                     .yaw_range = Limits<Angle>(-PI_4, PI_4),
                     .velocity_range = Limits<double>(-0.2, 0.2)},
                    route)
                .GetPlan();

        EXPECT_FALSE(plan.empty());

        draw_geom_profile(img_geom, plan);

        sdd::SDD img_vel(
            {.width = 50, .height = 50},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_Vel_1.svg",
                this->test_info_->name()));

        draw_velocity_profile(img_vel, plan);
    }

    {
        const auto map = Map::fromGeoJson("/truck/packages/map/data/map_2.geojson");

        sdd::SDD img_geom(
            {.width = 100, .height = 100},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_Geom_2.svg",
                this->test_info_->name()));
        img_geom.Add(
            sdd::ComplexPolygon{.complex_polygon = map.polygons()[0], .color = sdd::color::white});

        const auto ego_pose = Pose(Vec2(30, 20), AngleVec2::fromVector(Vec2(1, -1)));

        const auto route =
            Polyline{Vec2(10, 40), Vec2(20, 30), Vec2(25, 20), Vec2(35, 15), Vec2(50, 10)};
        img_geom.Add(sdd::Polyline{.polyline = route, .thickness = 0.5, .color = sdd::color::blue});

        auto model =
            std::make_shared<model::Model>(model::Model("/truck/packages/model/config/model.yaml"));

        auto collision_checker =
            std::make_shared<collision::StaticCollisionChecker>(model->shape());

        Planner planner(
            {.truck_state_params = {.min_dist_to_obstacle = 0.1},
             .state_space_params = {
                 .longitude = {.limits = Limits<double>(-5, 16), .total_states = 20},
                 .latitude = {.limits = Limits<double>(-5, 6), .total_states = 20},
                 .total_forward_yaw_states = 5,
                 .total_backward_yaw_states = 3,
                 .velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10}}});
        planner.SetModel(model).SetCollisionChecker(collision_checker);

        const auto& plan =
            planner
                .Build(
                    {.pose = ego_pose, .velocity = 0.0},
                    {.base_state =
                         {.pose = geom::Pose(Vec2(42, 13), AngleVec2::fromVector(Vec2(1, -1))),
                          .velocity = 0.5},
                     .x_range = Limits<double>(-1, 1),
                     .y_range = Limits<double>(-1, 1),
                     .yaw_range = Limits<Angle>(-PI_4, PI_4),
                     .velocity_range = Limits<double>(-0.2, 0.2)},
                    route)
                .GetPlan();

        EXPECT_FALSE(plan.empty());

        draw_geom_profile(img_geom, plan);

        sdd::SDD img_vel(
            {.width = 50, .height = 50},
            fmt::format(
                "/truck/packages/trajectory_planner/test/data/{}_Vel_2.svg",
                this->test_info_->name()));

        draw_velocity_profile(img_vel, plan);
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}