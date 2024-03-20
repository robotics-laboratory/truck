#include "trajectory_planner/planner.h"

#include "common/math.h"

#include "geom/distance.h"
#include "geom/segment.h"
#include "geom/uniform_stepper.h"

#include <algorithm>

namespace truck::trajectory_planner {

Planner::Planner(const Params& params) : params_(params) {}

Planner& Planner::Build(const StateSpace& state_space) noexcept {
    search_tree_.nodes.clear();
    search_tree_.edges.clear();

    search_tree_.nodes.reserve(state_space.GetStates().size());

    search_tree_.start_node =
        &search_tree_.nodes.emplace_back(Node{.state = &state_space.GetStartState()});

    for (const auto& state : state_space.GetStates()) {
        search_tree_.nodes.push_back(Node{.state = &state});
        if (state_space.GetFinishStates().contains(&state)) {
            search_tree_.finish_nodes.insert(&search_tree_.nodes.back());
        }
    }

    return *this;
}

}  // namespace truck::trajectory_planner