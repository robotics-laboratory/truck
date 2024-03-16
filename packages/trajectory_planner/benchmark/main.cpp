#include <benchmark/benchmark.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/function_output_iterator.hpp>

#include "trajectory_planner/state.h"
#include "trajectory_planner/hnsw.h"

#include "common/math.h"

#include "geom/angle.h"
#include "geom/distance.h"
#include "geom/pose.h"
#include "geom/bezier.h"
#include "geom/vector.h"

#include <algorithm>
#include <random>
#include <memory>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;

BOOST_GEOMETRY_REGISTER_POINT_2D(truck::geom::Vec2, double, cs::cartesian, x, y)

namespace bc = boost::container;
namespace bg = boost::geometry;

namespace {

const struct Constraints {
    Limits<double> x = Limits<double>(0, 100);
    Limits<double> y = Limits<double>(0, 100);
    Limits<double> yaw = Limits<double>(0, 2 * M_PI);
    Limits<double> velocity = Limits<double>(0, 0.8);
    Limits<double> radius = Limits<double>(0.1, 10);
} constraints;

const struct Params {
    size_t total_xs = 50;
    size_t total_ys = 50;
    size_t total_yaws = 10;
    size_t total_velocities = 10;
} params;

const size_t total_queries = 1;

struct Query {
    static Query Random() noexcept {
        auto random_device = std::random_device();
        auto gen = std::mt19937(random_device());

        return {
            .state =
                {.pose =
                     {.pos = geom::Vec2(
                          std::uniform_real_distribution<>(
                              constraints.x.min, constraints.x.max)(gen),
                          std::uniform_real_distribution<>(
                              constraints.y.min, constraints.y.max)(gen)),
                      .dir = geom::Angle(std::uniform_real_distribution<>(
                          constraints.yaw.min, constraints.yaw.max)(gen))},
                 .velocity = std::uniform_real_distribution<>(
                     constraints.velocity.min, constraints.velocity.max)(gen)},
            .radius = std::uniform_real_distribution<>(
                constraints.radius.min, constraints.radius.max)(gen)};
    }

    State state;
    double radius;
};

const struct BMData {
    BMData() {
        states.reserve(
            params.total_xs * params.total_ys * params.total_yaws * params.total_velocities);
        for (size_t i = 0; i < params.total_xs; ++i) {
            for (size_t j = 0; j < params.total_ys; ++j) {
                for (size_t k = 0; k < params.total_yaws; ++k) {
                    for (size_t l = 0; l < params.total_velocities; ++l) {
                        states.push_back(
                            {.pose =
                                 {.pos = Vec2(
                                      constraints.x.min +
                                          i * (constraints.x.max - constraints.x.min) /
                                              params.total_xs,
                                      constraints.y.min +
                                          j * (constraints.y.max - constraints.y.min) /
                                              params.total_ys),
                                  .dir = Angle(
                                      constraints.yaw.min +
                                      k * (constraints.yaw.max - constraints.yaw.min) /
                                          params.total_yaws)},
                             .velocity = constraints.velocity.min +
                                         (constraints.velocity.max - constraints.velocity.min) /
                                             params.total_velocities * l});
                    }
                }
            }
        }
        std::shuffle(states.begin(), states.end(), std::mt19937());
        queries.reserve(total_queries);
        for (size_t i = 0; i < total_queries; ++i) {
            queries.emplace_back(Query::Random());
        }
    }

    std::vector<State> states;
    std::vector<Query> queries;
} bm_data;

}  // namespace

struct RTreeFixture : public ::benchmark::Fixture {
    using RStarTreeValue = std::pair<geom::Vec2, const State*>;
    using RStarTreeBox = bg::model::box<geom::Vec2>;
    using RStarTree = bg::index::rtree<RStarTreeValue, bg::index::rstar<16>>;

    using RStarTreeQueryResult = std::pair<double, const State*>;

    std::vector<RStarTreeQueryResult> MakeQuery(const Query& query) noexcept {
        std::vector<RStarTreeQueryResult> result;
        for (size_t v = 0; v < params.total_velocities; ++v) {
            const double velocity =
                constraints.velocity.min +
                (constraints.velocity.max - constraints.velocity.min) / params.total_velocities * v;
            const double dist_radius = query.radius * (query.state.velocity + velocity) / 2;
            const RStarTreeBox query_box(
                geom::Vec2(
                    query.state.pose.pos.x - dist_radius, query.state.pose.pos.y - dist_radius),
                geom::Vec2(
                    query.state.pose.pos.x + dist_radius, query.state.pose.pos.y + dist_radius));
            r_star_trees[v].query(
                bg::index::intersects(query_box) &&
                    bg::index::satisfies([&query, &dist_radius](const RStarTreeValue& value) {
                        return MotionLength(FindMotion(query.state.pose, value.second->pose, 3)) <=
                               dist_radius;
                    }),
                boost::make_function_output_iterator([&query, &result](const auto& value) {
                    const double dist = MotionTime(
                        MotionLength(FindMotion(query.state.pose, value.second->pose, 3)),
                        query.state.velocity,
                        value.second->velocity);
                    result.push_back({dist, value.second});
                }));
        }
        return result;
    }

    std::vector<RStarTree> r_star_trees;
};

struct HNSWFixture : public ::benchmark::Fixture {
    auto MakeQuery(const Query& query) { return hnsw.RangeSearch(query.state, query.radius); }

    truck::trajectory_planner::HNSW hnsw;
};

BENCHMARK_DEFINE_F(RTreeFixture, RTree)
(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();

        r_star_trees = std::vector<RStarTree>(params.total_velocities);

        state.ResumeTiming();

        for (const auto& cur_state : bm_data.states) {
            r_star_trees[static_cast<size_t>(
                             (cur_state.velocity - constraints.velocity.min) /
                             (constraints.velocity.max - constraints.velocity.min))]
                .insert(std::make_pair(cur_state.pose.pos, &cur_state));
        }

        for (const auto& query : bm_data.queries) {
            auto result = MakeQuery(query);
            std::cerr << result.size() << '\n';
        }
    }
}

BENCHMARK_DEFINE_F(HNSWFixture, HNSW)
(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();

        hnsw = truck::trajectory_planner::HNSW(bm_data.states.size());

        state.ResumeTiming();

        for (size_t i = 0; i < bm_data.states.size(); ++i) {
            hnsw.AddPoint(bm_data.states[i], i);
        }

        for (const auto& query : bm_data.queries) {
            auto result = MakeQuery(query);
            std::cerr << result.size() << '\n';
        }
    }
}

BENCHMARK_REGISTER_F(RTreeFixture, RTree)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(3)
    ->UseRealTime();

BENCHMARK_REGISTER_F(HNSWFixture, HNSW)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(3)
    ->UseRealTime();

BENCHMARK_MAIN();