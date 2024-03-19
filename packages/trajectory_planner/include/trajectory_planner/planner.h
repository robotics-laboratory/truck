#pragma once

#include "trajectory_planner/state.h"

#include "common/math.h"

#include "geom/pose.h"
#include "geom/polyline.h"

#include <vector>

namespace truck::trajectory_planner {

struct Edge {
    State* from;
    State* to;
    double cost;
    double heuristic_cost;
};

struct Node {
    State* state;
    std::vector<const Edge*> edges;
};

struct SearchTree {
    Node* root;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
};

class Planner {
  public:
    struct Params {
        Discretization<double> longitude = {.limits = Limits<double>(-10, 11), .total_states = 10};
        Discretization<double> latitude = {.limits = Limits<double>(-5, 6), .total_states = 10};
        Discretization<double> forward_yaw = {
            .limits = Limits<double>(-M_PI_2, M_PI_2), .total_states = 5};
        Discretization<double> backward_yaw = {
            .limits = Limits<double>(M_PI_2, 3 * M_PI_2), .total_states = 3};
        Discretization<double> velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10};
    };

    Planner(const Params& params);

    Planner& Build(const geom::Pose& pose, const geom::Polyline& route);

    const std::vector<geom::Pose>& GetStatePoses() const noexcept;

  private:
    Params params_;

    std::vector<geom::Pose> state_poses_;
};

}  // namespace truck::trajectory_planner