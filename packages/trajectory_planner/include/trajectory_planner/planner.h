#pragma once

#include "trajectory_planner/state.h"

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
        double track_height = 10;
        double track_width = 10;
        double longitude_ratio = 0.5;
        size_t longitude_discretization = 25;
        size_t latitude_discretization = 25;
        size_t forward_yaw_discretization = 5;
        size_t backward_yaw_discretization = 5;
        size_t velocity_discretization = 10;
    };

    Planner(const Params& params);

    Planner& Build(const geom::Pose& pose, const geom::Polyline& route);

    const std::vector<geom::Pose>& GetStatePoses() const noexcept;

  private:
    Params params_;

    std::vector<geom::Pose> state_poses_;
};

}  // namespace truck::trajectory_planner