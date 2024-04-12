#include "trajectory_planner/tree.h"

#include "common/exception.h"

#include "geom/bezier.h"
#include "geom/distance.h"

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
    double inf = truck::trajectory_planner::INF_COST) noexcept {
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

Node::Estimator::Estimator(const StateSpace& state_space) : state_space_(state_space) {
    VERIFY(state_space_.truck_state.model);
}

double Node::Estimator::HeuristicCostFromStart(const State& to) const noexcept {
    return HeuristicCost(state_space_.start_states, to);
}

double Node::Estimator::HeuristicCostToFinish(const State& from) const noexcept {
    return HeuristicCost(state_space_.finish_states, from);
}

double Node::Estimator::AdmissibleMotionTime(
    double motion_length, double from_velocity, double to_velocity) const noexcept {
    const double a_min = -state_space_.truck_state.model->baseMaxDeceleration();
    const double a_max = state_space_.truck_state.model->baseMaxAcceleration();

    const double mid_velocity = std::min(
        std::sqrt(
            (2 * motion_length * a_min * a_max + from_velocity * from_velocity * a_min -
             to_velocity * to_velocity * a_max) /
            (a_min - a_max)),
        state_space_.truck_state.model->baseVelocityLimits().max);

    const double acceleration_time = (mid_velocity - from_velocity) / a_max;
    const double decelaration_time = (to_velocity - mid_velocity) / a_min;
    const double constant_speed_time =
        (motion_length - from_velocity * acceleration_time -
         a_max * acceleration_time * acceleration_time * 0.5 - mid_velocity * decelaration_time -
         a_min * decelaration_time * decelaration_time * 0.5) /
        mid_velocity;

    return acceleration_time + constant_speed_time + decelaration_time;
}

double Node::Estimator::HeuristicCost(const States& from, const State& to) const noexcept {
    if (!state_space_.truck_state.IsCollisionFree(to.pose)) {
        return INF_COST;
    }

    double min_cost = INF_COST;
    for (int i = 0; i < from.size; ++i) {
        const auto& from_state = from.data[i];
        if (!state_space_.truck_state.IsCollisionFree(from_state.pose)) {
            continue;
        }
        min_cost = std::min(
            min_cost,
            AdmissibleMotionTime(
                geom::distance(from_state.pose.pos, to.pose.pos),
                from_state.velocity,
                to.velocity));
    }

    return min_cost;
}

Node& Nodes::AddNode(const State& state, Node::Type type) noexcept {
    VERIFY(size < capacity);

    auto& node = data[size++];
    node.state = &state;
    node.type = type;
    if (estimator) {
        node.heuristic_cost_from_start =
            (type == Node::Type::START ? 0 : estimator->HeuristicCostFromStart(state));
        node.heuristic_cost_to_finish =
            (type == Node::Type::FINISH ? 0 : estimator->HeuristicCostToFinish(state));
    }
    node.heuristic_pass_cost = node.heuristic_cost_from_start + node.heuristic_cost_to_finish;

    return node;
}

Nodes& Nodes::FillProbabilities() noexcept {
    const double heuristics_sum =
        std::accumulate(data, data + size, 0.0, [&](double sum, const auto& node) {
            return sum + (node.type == Node::Type::REGULAR
                              ? 1 / (node.heuristic_cost_from_start + node.heuristic_cost_to_finish)
                              : 0.0);
        });

    for (int i = 0; i < size; ++i) {
        switch (data[i].type) {
            case Node::Type::START:
                data[i].probability = 0.0;
                break;
            case Node::Type::FINISH:
                data[i].probability = 0.0;
                break;
            default:
                data[i].probability = heuristics_sum / (data[i].heuristic_cost_from_start +
                                                        data[i].heuristic_cost_to_finish);
                break;
        }
    }

    return *this;
}

Nodes& Nodes::Clear() noexcept {
    size = 0;
    estimator = std::nullopt;
    return *this;
}

NodesHolder::NodesHolder(int capacity) {
    nodes_ptr = std::make_unique<Node[]>(capacity);
    nodes = {nodes_ptr.get(), 0, capacity};
}

Edge::Estimator::Estimator(
    const StateSpace& state_space, size_t total_heuristic_steps, double step_resolution)
    : state_space_(state_space)
    , total_heuristic_steps_(total_heuristic_steps)
    , step_resolution_(step_resolution) {
    VERIFY(state_space_.truck_state.model);
}

double Edge::Estimator::HeuristicCost(const State& from, const State& to) const noexcept {
    const auto motion = FindMotion(from.pose, to.pose, total_heuristic_steps_);
    const auto motion_length = MotionLength(motion);
    if (motion.empty()) {
        return INF_COST;
    }
    const auto motion_time = ExactMotionTime(motion_length, from.velocity, to.velocity);
    if (!IsKinodynamicFeasible(motion, motion_time, from.velocity, to.velocity)) {
        return INF_COST;
    }
    return motion_time;
}

double Edge::Estimator::Cost(const State& from, const State& to) const noexcept {
    const auto motion = FindMotion(from.pose, to.pose, step_resolution_);
    const auto motion_length = MotionLength(motion);
    if (motion.empty() || !IsDifferentiallyFeasible(motion)) {
        return INF_COST;
    }
    const auto motion_time = ExactMotionTime(motion_length, from.velocity, to.velocity);
    if (!IsKinodynamicFeasible(motion, motion_time, from.velocity, to.velocity)) {
        return INF_COST;
    }
    return motion_time;
}

bool Edge::Estimator::IsDifferentiallyFeasible(const geom::Poses& motion) const noexcept {
    if (motion.empty()) {
        return true;
    }

    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        if (!state_space_.truck_state.IsCollisionFree(*it)) {
            return false;
        }
        if (!state_space_.truck_state.model->middleSteeringLimits().isMet(
                ((it + 1)->dir - it->dir).angle().degrees())) {
            return false;
        }
        // TODO: Добавить проверку кривизны кривой Безье
    }
    if (!state_space_.truck_state.IsCollisionFree(motion.back())) {
        return false;
    }

    return true;
}

bool Edge::Estimator::IsKinodynamicFeasible(
    const geom::Poses& motion, double motion_time, double from_velocity, double to_velocity,
    double eps) const noexcept {
    if (!state_space_.truck_state.model->baseVelocityLimits().isMet(from_velocity) ||
        !state_space_.truck_state.model->baseVelocityLimits().isMet(to_velocity)) {
        return false;
    }

    if (motion_time < 0) {
        return false;
    }

    if (motion_time < eps && std::abs(from_velocity - to_velocity) < eps) {
        return true;
    }

    if (motion_time < eps) {
        return false;
    }

    const double acceleration = (to_velocity - from_velocity) / motion_time;
    if (acceleration < -state_space_.truck_state.model->baseMaxDeceleration() ||
        acceleration > state_space_.truck_state.model->baseMaxAcceleration()) {
        return false;
    }

    if (motion.size() < 2) {
        return true;
    }

    const double dt = motion_time / (motion.size() - 1);
    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        if (std::abs((it + 1)->dir.angle().radians() - it->dir.angle().radians()) / dt >
            state_space_.truck_state.model->steeringVelocity()) {
            return false;
        }
    }

    return true;
}

Edge& Edges::AddEdge(Node& from, Node& to) noexcept {
    VERIFY(size < capacity);

    auto& edge = data[size++];
    edge.from = &from;
    edge.to = &to;
    if (estimator) {
        edge.heuristic_cost = estimator->HeuristicCost(*from.state, *to.state);
        edge.cost = estimator->Cost(*from.state, *to.state);
    }
    if (from.cost_to_come + edge.cost < to.cost_to_come) {
        to.cost_to_come = from.cost_to_come + edge.cost;
        to.parent_node = &from;
    }

    return edge;
}

Edges& Edges::Clear() noexcept {
    size = 0;
    estimator = std::nullopt;
    return *this;
}

EdgesHolder::EdgesHolder(int capacity) {
    edges_ptr = std::make_unique<Edge[]>(capacity);
    edges = {edges_ptr.get(), 0, capacity};
}

Tree& Tree::Build(const StateSpace& state_space) noexcept {
    Clear();

    nodes.estimator = Node::Estimator(state_space);
    edges.estimator =
        Edge::Estimator(state_space, params.total_heuristic_steps, params.step_resolution);

    for (int i = 0; i < state_space.start_states.size; ++i) {
        nodes.AddNode(state_space.start_states.data[i], Node::Type::START);
    }

    for (int i = 0; i < state_space.finish_states.size; ++i) {
        nodes.AddNode(state_space.finish_states.data[i], Node::Type::FINISH);
    }

    for (int i = 0; i < state_space.regular_states.size; ++i) {
        nodes.AddNode(state_space.regular_states.data[i], Node::Type::REGULAR);
    }

    nodes.FillProbabilities();

    return *this;
}

void Tree::Clear() noexcept {
    nodes.Clear();
    edges.Clear();
}

TreeHolder::TreeHolder(const Tree::Params& params, int nodes_capacity, int edges_capacity)
    : nodes_holder(nodes_capacity), edges_holder(edges_capacity) {
    tree.params = params;
    tree.nodes = nodes_holder.nodes;
    tree.edges = edges_holder.edges;
}

}  // namespace truck::trajectory_planner