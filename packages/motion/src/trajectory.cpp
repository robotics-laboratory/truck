#include "motion/trajectory.h"

#include "geom/distance.h"
#include "geom/msg.h"

#include "common/exception.h"

namespace truck::motion {

void Trajectory::fillDistance() {
    if (states.empty()) {
        return;
    }

    auto it = states.begin();
    it->distance = 0;

    for (auto next = it + 1; next != states.end(); ++next) {
        next->distance = it->distance + geom::distance(it->pose.pos, next->pose.pos);
        it = next;
    }
}

namespace {

void throwIfVelocityViolated(const model::Model& model, const Trajectory& trajectory) {
    const Limits<double> limit{.0, model.baseVelocityLimits().max};

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& state = trajectory[i];

        VERIFY_FMT(
            limit.isMet(state.velocity),
            "State %i violates velocity limits: %f not in [%f, %f]",
            i,
            state.velocity,
            limit.min,
            limit.max);
    }
}

void throwIfAccelerationViolated(const model::Model& model, const Trajectory& trajectory) {
    const Limits<double> limit{
        trajectory.overbraking ? -std::numeric_limits<double>::infinity()
                               : model.baseAccelerationLimits().min,
        model.baseAccelerationLimits().max};

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& state = trajectory[i];

        VERIFY_FMT(
            limit.isMet(state.acceleration),
            "State %i violates acceleration limits: %f not in [%f, %f]",
            i,
            state.acceleration,
            limit.min,
            limit.max);
    }
}

inline bool eq(double a, double b, double e=1e-6) { return std::abs(a - b) < e; }

void throwIfInvalidDistance(const Trajectory& trajectory) {
    if (trajectory.empty()) {
        return;
    }

    VERIFY_FMT(
        trajectory.states.front().distance == 0,
        "First state has non-zero distance: %f",
        trajectory.states.front().distance);

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& prev = trajectory[i - 1];
        const auto& state = trajectory[i];

        const double expected = geom::distance(prev.pose.pos, state.pose.pos);
        const double distance = state.distance - prev.distance;

        VERIFY_FMT(
            eq(distance, expected, 1e-3),
            "State %i has incorrect distance: %f != %f",
            i,
            distance,
            expected);
    }
}

void throwIfInvalidTime(const Trajectory& trajectory) {
    if (trajectory.empty()) {
        return;
    }

    const auto& front = trajectory.states.front();
    VERIFY_FMT(front.time == 0, "First state has non-zero time: %f", front.time);

    if (front.collision) {
        VERIFY_FMT(eq(front.velocity, .0),
            "First state is a collision but has non-zero velocity: %f", front.velocity);

        VERIFY_FMT(eq(front.acceleration, .0),
            "First state is a collision but has non-zero acceleration: %f", front.acceleration);
    }

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& prev = trajectory[i - 1];
        const auto& state = trajectory[i];

        if (std::isinf(state.time)) {
            VERIFY_FMT(state.collision,
                "State %i has infinite time, but is not a collision", i);

            VERIFY_FMT(state.velocity == .0,
                "State %i has non-zero velocity %f",  i, state.velocity);

            VERIFY_FMT(state.acceleration == .0,
                "State %i has non-zero acceleration %f", i, state.acceleration);

            continue;
        }

        VERIFY_FMT(prev.time < state.time,
            "State %i has non-increasing time: %f -> %f",
            i, prev.time, state.time);

        const double dt = state.time - prev.time;

        const double expected_distance = state.distance - prev.distance;
        const double distance = dt * prev.velocity + 0.5 * dt * dt * prev.acceleration;

        VERIFY_FMT(eq(distance, expected_distance, 1e-3),
            "State %i has incorrect velocity: %f != %f",
            i, distance, expected_distance);

        const double expected_velocity = prev.velocity + prev.acceleration * dt;

        VERIFY_FMT(eq(state.velocity, expected_velocity, 1e-3),
            "State %i has incorrect velocity: %f != %f",
            i, state.velocity, expected_velocity);
    }
}

void throwIfInvalid(const model::Model& model, const Trajectory& trajectory) {
    throwIfVelocityViolated(model, trajectory);
    throwIfAccelerationViolated(model, trajectory);
    throwIfInvalidDistance(trajectory);
    throwIfInvalidTime(trajectory);
}

}  // namespace

void Trajectory::throwIfInvalid(const model::Model& model) { motion::throwIfInvalid(model, *this); }

State Trajectory::operator()(double time) const {
    VERIFY(!states.empty());

    const auto end = std::find_if(
        states.begin(), states.end(), [time](const auto& state) { return state.time > time; });

    if (end == states.begin()) {
        return states.front();
    }

    if (end == states.end()) {
        return states.back();
    }

    const auto begin = end - 1;
    const double dt = time - begin->time;
    const double distance = end->distance - begin->distance;
    const double delta = dt * begin->velocity + 0.5 * dt * dt * begin->acceleration;

    return State{
        .pose = geom::interpolate(begin->pose, end->pose, delta / distance),
        // use max max for safety reasons
        .margin = std::max(begin->margin, end->margin),
        .collision = std::max(begin->collision, end->collision),
        .time = time,
        .distance = begin->distance + delta,
        .velocity = begin->velocity + dt * begin->acceleration,
        .acceleration = begin->acceleration,
    };
}

State toState(const truck_msgs::msg::TrajectoryState& msg) {
    return State{
        .pose = geom::toPose(msg.pose),
        .margin = msg.margin,
        .collision = msg.collision,
        .time = msg.time,
        .distance = msg.distance,
        .velocity = msg.velocity,
        .acceleration = msg.acceleration,
    };
}

States toStates(const std::vector<truck_msgs::msg::TrajectoryState>& msgs) {
    States states;
    states.reserve(msgs.size());
    for (const auto& msg : msgs) {
        states.push_back(toState(msg));
    }
    return states;
}

Trajectory toTrajectory(const truck_msgs::msg::Trajectory& msg) {
    return Trajectory{
        .overbraking = msg.overbraking,
        .states = toStates(msg.states),
    };
}

namespace msg {

truck_msgs::msg::TrajectoryState toTrajectoryState(const State& state) {
    truck_msgs::msg::TrajectoryState msg;
    msg.pose = geom::msg::toPose(state.pose);
    msg.time = state.time;
    msg.distance = state.distance;
    msg.velocity = state.velocity;
    msg.acceleration = state.acceleration;
    msg.margin = state.margin;
    msg.collision = state.collision;
    return msg;
}

truck_msgs::msg::Trajectory toTrajectory(
    const std_msgs::msg::Header& header, const Trajectory& trajectory) {
    truck_msgs::msg::Trajectory msg;

    msg.header = header;
    msg.overbraking = trajectory.overbraking;

    msg.states.reserve(trajectory.states.size());
    for (const auto& state : trajectory.states) {
        msg.states.push_back(toTrajectoryState(state));
    }
    return msg;
}

}  // namespace msg
}  // namespace truck::motion