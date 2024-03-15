#include <benchmark/benchmark.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/function_output_iterator.hpp>

#include <hnswlib/hnswlib.h>

#include "geom/angle.h"
#include "geom/distance.h"
#include "geom/pose.h"
#include "geom/bezier.h"
#include "geom/vector.h"

#include <memory>

BOOST_GEOMETRY_REGISTER_POINT_2D(truck::geom::Vec2, double, cs::cartesian, x, y)

using namespace truck;
namespace bc = boost::container;
namespace bg = boost::geometry;

namespace {

constexpr struct Constraints {
    const std::pair<double, double> xs = {0, 100};
    const std::pair<double, double> ys = {0, 100};
    const std::pair<double, double> yaws = {0, 2 * M_PI};
    const std::pair<double, double> velocities = {0, 0.8};
    const std::pair<double, double> radiuses = {0.01, 10};
} constraints;

constexpr struct Params {
    const size_t total_xs = 100;
    const size_t total_ys = 100;
    const size_t total_yaws = 10;
    const size_t total_velocities = 10;
} params;

constexpr auto total_queries = 10000;

struct State {
    geom::Pose pose;
    double velocity;
};

struct Query {
    State state;
    double radius;
};

struct QueryGen {
    QueryGen(const Constraints& constraints = Constraints())
        : random_device(std::random_device())
        , gen(random_device())
        , x_dist(constraints.xs.first, constraints.xs.second)
        , y_dist(constraints.ys.first, constraints.ys.second)
        , yaws_dist(constraints.yaws.first, constraints.yaws.second)
        , velocities_dist(constraints.velocities.first, constraints.velocities.second)
        , radiuses_dist(constraints.radiuses.first, constraints.radiuses.second) {}

    Query operator()() noexcept {
        return {
            .state =
                {.pose =
                     geom::Pose(geom::Vec2(x_dist(gen), y_dist(gen)), geom::Angle(yaws_dist(gen))),
                 .velocity = velocities_dist(gen)},
            .radius = radiuses_dist(gen)};
    }

    std::random_device random_device;
    std::mt19937 gen;

    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    std::uniform_real_distribution<> yaws_dist;
    std::uniform_real_distribution<> velocities_dist;
    std::uniform_real_distribution<> radiuses_dist;
};

struct BMData {
    BMData(const Constraints& constraints, const Params& params, size_t total_queries) {
        states.reserve(
            params.total_xs * params.total_ys * params.total_yaws * params.total_velocities);
        for (size_t i = 0; i < params.total_xs; ++i) {
            const double x = constraints.xs.first +
                             i * (constraints.xs.second - constraints.xs.first) / params.total_xs;
            for (size_t j = 0; j < params.total_ys; ++j) {
                const double y =
                    constraints.ys.first +
                    j * (constraints.ys.second - constraints.ys.first) / params.total_ys;
                for (size_t k = 0; k < params.total_yaws; ++k) {
                    const double yaw =
                        constraints.yaws.first +
                        k * (constraints.yaws.second - constraints.yaws.first) / params.total_yaws;
                    for (size_t v = 0; v < params.total_velocities; ++v) {
                        const double velocity =
                            constraints.velocities.first +
                            v * (constraints.velocities.second - constraints.velocities.first) /
                                params.total_velocities;
                        states.push_back(State{
                            .pose = geom::Pose(geom::Vec2(x, y), geom::Angle(yaw)),
                            .velocity = velocity});
                    }
                }
            }
        }
        queries.reserve(total_queries);
        QueryGen query_gen(constraints);
        for (size_t i = 0; i < total_queries; ++i) {
            queries.emplace_back(query_gen());
        }
    }

    std::vector<State> states;
    std::vector<Query> queries;
};

const auto bm_data = BMData(constraints, params, total_queries);

geom::Poses FindMotion(const geom::Pose& from, const geom::Pose& to, size_t step) {
    const auto dist = geom::distance(from.pos, to.pos);
    const geom::Vec2 from_ref = from.pos + from.dir * 0.5 * dist;
    const geom::Vec2 to_ref = to.pos - to.dir * 0.5 * dist;
    return geom::bezier3(from.pos, from_ref, to_ref, to.pos, step);
}

}  // namespace

namespace hnswlib {

static double StatesDist(const void* pVect1v, const void* pVect2v, const void* qty_ptr) {
    const State& pVect1 = *static_cast<const State*>(pVect1v);
    const State& pVect2 = *static_cast<const State*>(pVect2v);
    auto motion = FindMotion(pVect1.pose, pVect2.pose, 3);
    return 2 *
           (geom::distance(motion[0].pos, motion[1].pos) +
            geom::distance(motion[1].pos, motion[2].pos)) /
           (pVect1.velocity + pVect2.velocity);
}

class StateSpace final : public SpaceInterface<double> {
  public:
    StateSpace() {
        fstdistfunc_ = StatesDist;
        data_size_ = sizeof(State);
        dim_ = 1;
    }

    size_t get_data_size() final override { return data_size_; }

    DISTFUNC<double> get_dist_func() final override { return fstdistfunc_; }

    void* get_dist_func_param() final override { return &dim_; }

  private:
    DISTFUNC<double> fstdistfunc_;
    size_t data_size_;
    size_t dim_;
};

}  // namespace hnswlib

struct RTreeFixture : public ::benchmark::Fixture {
    using RStarTreeValue = std::pair<geom::Vec2, const State*>;
    using RStarTreeBox = bg::model::box<geom::Vec2>;
    using RStarTree = bg::index::rtree<RStarTreeValue, bg::index::rstar<16>>;

    using RStarTreeQueryResult = std::pair<double, const State*>;

    void SetUp(const ::benchmark::State&) override {
        r_star_trees.resize(params.total_velocities);
        for (const auto& state : bm_data.states) {
            r_star_trees[static_cast<size_t>(
                             (state.velocity - constraints.velocities.first) /
                             (constraints.velocities.second - constraints.velocities.first))]
                .insert(std::make_pair(state.pose.pos, &state));
        }
    }

    std::vector<RStarTreeQueryResult> MakeQuery(const Query& query) noexcept {
        std::vector<RStarTreeQueryResult> result;
        for (size_t v = 0; v < params.total_velocities; ++v) {
            const double velocity =
                constraints.velocities.first +
                v * (constraints.velocities.second - constraints.velocities.first) /
                    params.total_velocities;
            const double dist_radius = query.radius * (query.state.velocity + velocity) / 2;
            const RStarTreeBox query_box(
                geom::Vec2(
                    query.state.pose.pos.x - dist_radius, query.state.pose.pos.y - dist_radius),
                geom::Vec2(
                    query.state.pose.pos.x + dist_radius, query.state.pose.pos.y + dist_radius));
            r_star_trees[v].query(
                bg::index::intersects(query_box) &&
                    bg::index::satisfies([&query, &dist_radius](const RStarTreeValue& value) {
                        const auto motion = FindMotion(query.state.pose, value.second->pose, 3);
                        return (geom::distance(motion[0].pos, motion[1].pos) +
                                geom::distance(motion[1].pos, motion[2].pos)) <= dist_radius;
                    }),
                boost::make_function_output_iterator([&query, &result](const auto& value) {
                    const auto motion = FindMotion(query.state.pose, value.second->pose, 3);
                    const double dist = 2 *
                                        (geom::distance(motion[0].pos, motion[1].pos) +
                                         geom::distance(motion[1].pos, motion[2].pos)) /
                                        (query.state.velocity + value.second->velocity);
                    result.push_back({dist, value.second});
                }));
        }
        return result;
    }

    std::vector<RStarTree> r_star_trees;
};

struct HNSWFixture : public ::benchmark::Fixture {
    using HNSWQueryResult = std::pair<double, hnswlib::labeltype>;

    void SetUp(const ::benchmark::State&) override {
        state_space = hnswlib::StateSpace();
        alg_hnsw = std::make_unique<hnswlib::HierarchicalNSW<double>>(
            &state_space, bm_data.states.size(), M, ef_construction);
        for (size_t i = 0; i < bm_data.states.size(); ++i) {
            hnswlib::labeltype label = i;
            alg_hnsw->addPoint(&bm_data.states[i], label);
        }
    }

    std::vector<HNSWQueryResult> MakeQuery(const Query& query) {
        hnswlib::EpsilonSearchStopCondition<double> stop_condition(
            query.radius, 0, bm_data.states.size());
        return alg_hnsw->searchStopConditionClosest(&query.state, stop_condition);
    }

    const size_t M = 6;
    const size_t ef_construction = 50;

    hnswlib::StateSpace state_space;
    std::unique_ptr<hnswlib::HierarchicalNSW<double>> alg_hnsw;
};

BENCHMARK_DEFINE_F(RTreeFixture, RTree)
(benchmark::State& state) {
    for (auto _ : state) {
        for (const auto& query : bm_data.queries) {
            MakeQuery(query);
        }
    }
}

BENCHMARK_DEFINE_F(HNSWFixture, HNSW)
(benchmark::State& state) {
    for (auto _ : state) {
        for (const auto& query : bm_data.queries) {
            MakeQuery(query);
        }
    }
}

BENCHMARK_REGISTER_F(RTreeFixture, RTree)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(5)
    ->UseRealTime();

BENCHMARK_REGISTER_F(HNSWFixture, HNSW)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(5)
    ->UseRealTime();

BENCHMARK_MAIN();