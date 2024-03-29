#include "trajectory_planner/planner.h"

#include "common/math.h"
#include "common/exception.h"

#include "geom/bezier.h"
#include "geom/distance.h"

#include <limits>

namespace {

truck::geom::Poses FindMotion(
    const truck::geom::Pose& from, const truck::geom::Pose& to, size_t max_steps,
    double eps = 1e-7) noexcept {
    const auto dist = truck::geom::distance(from.pos, to.pos);

    if (dist < eps && std::abs(from.dir.angle().radians() - to.dir.angle().radians()) < eps) {
        return truck::geom::Poses({from});
    }

    if (dist < eps) {
        return truck::geom::Poses();
    }

    const double gamma = dist * 0.5;
    const truck::geom::Vec2 from_ref = from.pos + from.dir * gamma;
    const truck::geom::Vec2 to_ref = to.pos - to.dir * gamma;

    return truck::geom::bezier3(from.pos, from_ref, to_ref, to.pos, max_steps);
}

truck::geom::Poses FindMotion(
    const truck::geom::Pose& from, const truck::geom::Pose& to, double step,
    double eps = 1e-7) noexcept {
    const auto dist = truck::geom::distance(from.pos, to.pos);

    if (dist < eps && std::abs(from.dir.angle().radians() - to.dir.angle().radians()) < eps) {
        return truck::geom::Poses({from});
    }

    if (dist < eps) {
        return truck::geom::Poses();
    }

    const double gamma = dist * 0.5;
    const truck::geom::Vec2 from_ref = from.pos + from.dir * gamma;
    const truck::geom::Vec2 to_ref = to.pos - to.dir * gamma;

    return truck::geom::bezier3(from.pos, from_ref, to_ref, to.pos, step);
}

double MotionLength(const truck::geom::Poses& motion) noexcept {
    double motion_lenght = 0;

    if (motion.size() < 2) {
        return motion_lenght;
    }

    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        motion_lenght += truck::geom::distance(it->pos, (it + 1)->pos);
    }

    return motion_lenght;
}

double ExactMotionTime(
    double motion_length, double from_velocity, double to_velocity, double eps = 1e-7,
    double inf = std::numeric_limits<double>::max()) noexcept {
    if (motion_length < eps && std::abs(from_velocity - to_velocity) < eps) {
        return 0;
    }

    const auto av_velocity = (from_velocity + to_velocity) * 0.5;

    if (motion_length < eps || std::abs(av_velocity) < eps) {
        return inf;
    }

    return motion_length / av_velocity;
}

}  // namespace

namespace truck::trajectory_planner {

NodeId SearchTree::AddNode(Node node) noexcept {
    nodes.push_back(std::move(node));
    return nodes.size() - 1;
}

EdgeId SearchTree::AddEdge(Edge edge) noexcept {
    edges.push_back(std::move(edge));
    return edges.size() - 1;
}

void SearchTree::Clear() noexcept {
    nodes.clear();
    node_probabilities.clear();
    edges.clear();
    start_node = 0;
    finish_nodes.clear();
}

Planner::Planner(const Params& params, const model::Model& model)
    : params_(params), model_(model) {}

Planner& Planner::SetCollisionChecker(
    std::shared_ptr<const collision::StaticCollisionChecker> collision_checker) {
    collision_checker_ = std::move(collision_checker);
    return *this;
}

Planner& Planner::Build(const StateSpace& state_space) noexcept {
    search_tree_.Clear();

    search_tree_.nodes.reserve(state_space.GetStates().size());

    VERIFY(IsCollisionFree(state_space.GetStartState().pose));
    search_tree_.start_node = search_tree_.AddNode(Node{.state = &state_space.GetStartState()});

    for (const auto& state : state_space.GetStates()) {
        if (!IsCollisionFree(state.pose)) {
            continue;
        }
        const auto node_id = search_tree_.AddNode(Node{.state = &state});
        if (state_space.GetFinishStates().contains(&state)) {
            search_tree_.nodes[node_id].is_finish = true;
            search_tree_.finish_nodes.push_back(node_id);
        }
    }

    return *this;
}

std::optional<double> Planner::HeuristicCost(const State& from, const State& to) const noexcept {
    const auto motion = FindMotion(from.pose, to.pose, params_.total_heuristic_steps);
    const auto motion_length = MotionLength(motion);
    if (motion.empty() || !IsDifferentiallyFeasible(motion)) {
        return std::nullopt;
    }
    const auto motion_time = ExactMotionTime(motion_length, from.velocity, to.velocity);
    if (!IsKinodynamicFeasible(motion_time, from.velocity, to.velocity)) {
        return std::nullopt;
    }
    return motion_time;
}

std::optional<double> Planner::Cost(const State& from, const State& to) const noexcept {
    const auto motion = FindMotion(from.pose, to.pose, params_.step_resolution);
    const auto motion_length = MotionLength(motion);
    if (motion.empty() || !IsDifferentiallyFeasible(motion)) {
        return std::nullopt;
    }
    const auto motion_time = ExactMotionTime(motion_length, from.velocity, to.velocity);
    if (!IsKinodynamicFeasible(motion_time, from.velocity, to.velocity)) {
        return std::nullopt;
    }
    return motion_time;
}

std::optional<double> Planner::HeuristicCostFromStart(const State& state) const noexcept {
    VERIFY(search_tree_.start_node);

    const auto* start_state = search_tree_.nodes[search_tree_.start_node].state;
    const auto cost = AdmissibleMotionTime(
        geom::distance(start_state->pose.pos, state.pose.pos),
        start_state->velocity,
        state.velocity);
    if (cost > params_.planning_horizon) {
        return std::nullopt;
    }
    return cost;
}

std::optional<double> Planner::HeuristicCostToFinish(const State& state) const noexcept {
    double min_cost = std::numeric_limits<double>::max();
    for (const auto& finish_node_id : search_tree_.finish_nodes) {
        const auto cost = AdmissibleMotionTime(
            geom::distance(state.pose.pos, search_tree_.nodes[finish_node_id].state->pose.pos),
            state.velocity,
            search_tree_.nodes[finish_node_id].state->velocity);
        min_cost = std::min(min_cost, cost);
    }

    if (min_cost > params_.planning_horizon) {
        return std::nullopt;
    }

    return min_cost;
}

bool Planner::IsCollisionFree(const geom::Pose& pose) const noexcept {
    if (!collision_checker_) {
        return true;
    }
    return collision_checker_->distance(pose) > params_.min_dist_to_obstacle;
}

bool Planner::IsDifferentiallyFeasible(const geom::Poses& motion) const noexcept {
    if (motion.empty()) {
        return true;
    }

    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        if (!IsCollisionFree(*it)) {
            return false;
        }
        if (!model_.middleSteeringLimits().isMet(((it + 1)->dir - it->dir).angle().radians())) {
            return false;
        }
    }
    if (!IsCollisionFree(motion.back())) {
        return false;
    }

    return true;
}

bool Planner::IsKinodynamicFeasible(
    double motion_time, double from_velocity, double to_velocity) const noexcept {
    if (!model_.baseVelocityLimits().isMet(from_velocity) &&
        !model_.baseVelocityLimits().isMet(to_velocity)) {
        return false;
    }

    if (motion_time < 0 || motion_time > params_.planning_horizon) {
        return false;
    }

    const double acceleration = (to_velocity - from_velocity) / motion_time;
    if (acceleration < model_.baseMaxDeceleration() ||
        acceleration > model_.baseMaxAcceleration()) {
        return false;
    }

    return true;
}

double Planner::AdmissibleMotionTime(
    double motion_length, double from_velocity, double to_velocity) const noexcept {
    const double a_min = model_.baseMaxDeceleration();
    const double a_max = model_.baseMaxAcceleration();

    const double mid_velocity = std::min(
        std::sqrt(
            2 * motion_length * a_min * a_max + from_velocity * from_velocity * a_min -
            to_velocity * to_velocity * a_max),
        model_.baseVelocityLimits().max);

    const double acceleration_time = (mid_velocity - from_velocity) / a_max;
    const double decelaration_time = (to_velocity - mid_velocity) / a_min;
    const double constant_speed_time =
        (motion_length - acceleration_time * (from_velocity + mid_velocity) * 0.5 -
         decelaration_time * (mid_velocity + to_velocity) * 0.5) /
        mid_velocity;

    return acceleration_time + constant_speed_time + decelaration_time;
}

}  // namespace truck::trajectory_planner