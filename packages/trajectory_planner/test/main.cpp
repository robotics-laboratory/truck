#include <gtest/gtest.h>

#include "trajectory_planner/state.h"
#include "trajectory_planner/rtree.h"

#include "common/math.h"

#include "geom/vector.h"
#include "geom/pose.h"

#include <iostream>

using namespace truck;
using namespace truck::geom;
using namespace truck::trajectory_planner;

namespace {

struct Constraints {
    Discretization<double> x = {.limits = Limits<double>(0, 100), .total_states = 25};
    Discretization<double> y = {.limits = Limits<double>(0, 100), .total_states = 25};
    Discretization<double> yaw = {.limits = Limits<double>(0, 2 * M_PI), .total_states = 7};
    Discretization<double> velocity = {.limits = Limits<double>(0, 0.8), .total_states = 7};
};

States GenerateStates(const Constraints& constraints = {}) {
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