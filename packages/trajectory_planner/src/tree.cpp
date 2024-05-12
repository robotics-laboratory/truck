#include "trajectory_planner/tree.h"

#include "common/exception.h"

#include "geom/distance.h"

namespace {

truck::geom::CurvePoses FindMotion(
    const truck::geom::Pose& from, const truck::geom::Pose& to, size_t max_steps,
    double eps = 1e-7) noexcept {
    const auto dist = truck::geom::distance(from.pos, to.pos);

    if (dist < eps && std::abs(from.dir.angle().radians() - to.dir.angle().radians()) < eps) {
        return truck::geom::CurvePoses({{.pose = from, .curvature = 0.0}});
    }

    if (dist < eps) {
        return truck::geom::CurvePoses();
    }

    const double gamma = dist * 0.5;
    const truck::geom::Vec2 from_ref = from.pos + from.dir * gamma;
    const truck::geom::Vec2 to_ref = to.pos - to.dir * gamma;

    return truck::geom::bezier3(from.pos, from_ref, to_ref, to.pos, max_steps);
}

truck::geom::CurvePoses FindMotion(
    const truck::geom::Pose& from, const truck::geom::Pose& to, double step,
    double eps = 1e-7) noexcept {
    const auto dist = truck::geom::distance(from.pos, to.pos);

    if (dist < eps && std::abs(from.dir.angle().radians() - to.dir.angle().radians()) < eps) {
        return truck::geom::CurvePoses({{.pose = from, .curvature = 0.0}});
    }

    if (dist < eps) {
        return truck::geom::CurvePoses();
    }

    const double gamma = dist * 0.5;
    const truck::geom::Vec2 from_ref = from.pos + from.dir * gamma;
    const truck::geom::Vec2 to_ref = to.pos - to.dir * gamma;

    return truck::geom::bezier3(from.pos, from_ref, to_ref, to.pos, step);
}

double MotionLength(const truck::geom::CurvePoses& motion) noexcept {
    double motion_lenght = 0;

    if (motion.size() < 2) {
        return motion_lenght;
    }

    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        motion_lenght += truck::geom::distance(it->pose.pos, (it + 1)->pose.pos);
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

double Node::Estimator::HeuristicCostFromStart(const State& state) const noexcept {
    return HeuristicCost(state_space_.start_states, state);
}

double Node::Estimator::HeuristicCostToFinish(const State& state) const noexcept {
    return HeuristicCost(state_space_.finish_states, state);
}

Node::Estimator& Node::Estimator::Reset(StateSpace state_space) noexcept {
    state_space_ = std::move(state_space);
    return *this;
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
    auto& node = data[size++];
    node.state = &state;
    node.type = type;
    node.heuristic_cost_from_start =
        (type == Node::Type::START ? 0.0 : estimator.HeuristicCostFromStart(state));
    node.heuristic_cost_to_finish =
        (type == Node::Type::FINISH ? 0.0 : estimator.HeuristicCostToFinish(state));
    node.heuristic_pass_cost = node.heuristic_cost_from_start + node.heuristic_cost_to_finish;
    return node;
}

Nodes& Nodes::FillProbabilities() noexcept {
    const double heuristics_sum =
        std::accumulate(data, data + size, 0.0, [&](double sum, const auto& node) {
            return sum + (1 / node.heuristic_pass_cost);
        });

    for (int i = 0; i < size; ++i) {
        data[i].probability = (1 / data[i].heuristic_pass_cost) / heuristics_sum;
    }
    return *this;
}

Nodes& Nodes::Clear() noexcept {
    size = 0;
    estimator.Reset({});
    return *this;
}

Nodes& Nodes::Reset(Node* ptr) noexcept {
    data = ptr;
    return *this;
}

NodesHolder::NodesHolder(Nodes&& nodes, NodesDataPtr&& nodes_ptr) noexcept
    : nodes(std::move(nodes)), nodes_ptr(std::move(nodes_ptr)) {}

NodesHolder MakeNodes(int capacity) {
    auto nodes_ptr = std::make_unique<Node[]>(capacity);
    auto nodes = Nodes{.data = nodes_ptr.get(), .size = 0};
    nodes.Reset(nodes_ptr.get());
    return {std::move(nodes), std::move(nodes_ptr)};
}

Edge::Estimator::Estimator(size_t total_heuristic_steps, double step_resolution)
    : total_heuristic_steps_(total_heuristic_steps), step_resolution_(step_resolution) {}

double Edge::Estimator::HeuristicCost(const Node& from, const Node& to) const noexcept {
    const auto motion = FindMotion(from.state->pose, to.state->pose, total_heuristic_steps_);
    const auto motion_length = MotionLength(motion);
    if (motion.empty()) {
        return INF_COST;
    }
    const auto motion_time =
        ExactMotionTime(motion_length, from.state->velocity, to.state->velocity);
    if (!IsKinodynamicFeasible(
            motion, motion_time, from.state->velocity, to.state->velocity, false)) {
        return INF_COST;
    }
    return motion_time;
}

double Edge::Estimator::Cost(const Node& from, const Node& to) const noexcept {
    const auto motion = FindMotion(from.state->pose, to.state->pose, step_resolution_);
    const auto motion_length = MotionLength(motion);
    if (motion.empty() || !IsDifferentiallyFeasible(motion)) {
        return INF_COST;
    }
    const auto motion_time =
        ExactMotionTime(motion_length, from.state->velocity, to.state->velocity);
    if (!IsKinodynamicFeasible(motion, motion_time, from.state->velocity, to.state->velocity)) {
        return INF_COST;
    }
    return motion_time;
}

Edge::Estimator& Edge::Estimator::Reset(StateSpace state_space) noexcept {
    state_space_ = std::move(state_space);
    return *this;
}

bool Edge::Estimator::IsDifferentiallyFeasible(const geom::CurvePoses& motion) const noexcept {
    if (motion.empty()) {
        return true;
    }

    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        if (!state_space_.truck_state.IsCollisionFree(it->pose)) {
            return false;
        }
        if (!state_space_.truck_state.model->middleSteeringLimits().isMet(
                ((it + 1)->pose.dir - it->pose.dir).angle().degrees())) {
            return false;
        }
        if (state_space_.truck_state.model->baseMaxAbsCurvature() < std::abs(it->curvature)) {
            return false;
        }
    }
    if (!state_space_.truck_state.IsCollisionFree(motion.back().pose) ||
        state_space_.truck_state.model->baseMaxAbsCurvature() < std::abs(motion.back().curvature)) {
        return false;
    }

    return true;
}

bool Edge::Estimator::IsKinodynamicFeasible(
    const geom::CurvePoses& motion, double motion_time, double from_velocity, double to_velocity,
    bool check_steering, double eps) const noexcept {
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

    if (check_steering) {
        double l = 0;
        double last_time = 0;
        for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
            l += geom::distance(it->pose.pos, (it + 1)->pose.pos);
            const double D = from_velocity * from_velocity + 2 * acceleration * l;
            if (D < 0) {
                return false;
            }
            const double cur_time = (-from_velocity + std::sqrt(D)) / acceleration;
            const double dt = cur_time - last_time;
            if (std::abs((it + 1)->pose.dir.angle().radians() - it->pose.dir.angle().radians()) /
                    dt >
                state_space_.truck_state.model->steeringVelocity()) {
                return false;
            }
            last_time = cur_time;
        }
    }

    return true;
}

Edge& Edges::AddEdge(Node& from, Node& to) noexcept {
    auto& edge = data[size++];
    edge.from = &from;
    edge.to = &to;
    edge.heuristic_cost = estimator.HeuristicCost(from, to);
    return edge;
}

Edges& Edges::Clear() noexcept {
    size = 0;
    estimator.Reset({});
    return *this;
}

Edges& Edges::Reset(Edge* ptr) noexcept {
    data = ptr;
    return *this;
}

EdgesHolder::EdgesHolder(Edges&& edges, EdgesDataPtr&& edges_ptr) noexcept
    : edges(std::move(edges)), edges_ptr(std::move(edges_ptr)) {}

EdgesHolder MakeEdges(int capacity) {
    auto edges_ptr = std::make_unique<Edge[]>(capacity);
    auto edges = Edges{.data = edges_ptr.get(), .size = 0};
    edges.Reset(edges_ptr.get());
    return {std::move(edges), std::move(edges_ptr)};
}

Tree& Tree::Build(const StateSpace& state_space) noexcept {
    Clear();

    nodes.estimator.Reset(state_space);
    edges.estimator =
        Edge::Estimator(params.total_heuristic_steps, params.step_resolution).Reset(state_space);

    start_nodes.data = nodes.data + nodes.size;
    for (int i = 0; i < state_space.start_states.size; ++i) {
        nodes.AddNode(state_space.start_states.data[i], Node::Type::START);
    }
    start_nodes.size = nodes.size;

    finish_nodes.data = nodes.data + nodes.size;
    for (int i = 0; i < state_space.finish_states.size; ++i) {
        nodes.AddNode(state_space.finish_states.data[i], Node::Type::FINISH);
    }
    finish_nodes.size = nodes.size - start_nodes.size;

    regular_nodes.data = nodes.data + nodes.size;
    for (int i = 0; i < state_space.regular_states.size; ++i) {
        nodes.AddNode(state_space.regular_states.data[i], Node::Type::REGULAR);
    }
    regular_nodes.size = nodes.size - start_nodes.size - finish_nodes.size;

    regular_nodes.FillProbabilities();

    return *this;
}

Tree& Tree::Clear() noexcept {
    nodes.Clear();
    edges.Clear();
    return *this;
}

Tree& Tree::Reset(Nodes nodes, Edges edges) noexcept {
    this->nodes = std::move(nodes);
    this->edges = std::move(edges);
    return *this;
}

TreeHolder::TreeHolder(Tree&& tree, NodesHolder&& nodes_holder, EdgesHolder&& edges_holder) noexcept
    : tree(std::move(tree))
    , nodes_holder(std::move(nodes_holder))
    , edges_holder(std::move(edges_holder)) {}

TreeHolder MakeTree(const Tree::Params& params, int nodes_capacity, int edges_capacity) {
    auto nodes_holder = MakeNodes(nodes_capacity);
    auto edges_holder = MakeEdges(edges_capacity);
    auto tree = Tree{.params = params}.Reset(nodes_holder.nodes, edges_holder.edges);
    return {std::move(tree), std::move(nodes_holder), std::move(edges_holder)};
}

}  // namespace truck::trajectory_planner