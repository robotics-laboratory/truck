#include "motion/trajectory.h"

#include "geom/distance.h"
#include "geom/msg.h"

#include "common/exception.h"

#include <iomanip>
#include <optional>
#include <sstream>

namespace truck::motion {

std::string toString(std::optional<double> v) { return v ? std::to_string(*v) : "null"; }

std::ostream& operator<<(std::ostream& out, const State& state) noexcept {
    return out << std::fixed << std::setprecision(2) << std::boolalpha << "{"
               << "t=" << toString(state.time) << ", "
               << "p=" << state.pose << ", "
               << "m=" << state.margin << ":" << state.collision << ", "
               << "d=" << state.distance << ", "
               << "v=" << state.velocity << ":" << state.acceleration << "}";
}

std::string State::toString() const noexcept {
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

void Trajectory::fillDistance() {
    if (states.empty()) {
        return;
    }

    auto it = states.begin();
    it->distance = 0;

    for (auto next = it + 1; next != states.end();) {
        next->distance = it->distance + geom::distance(it->pose.pos, next->pose.pos);
        it = next;
        ++next;
    }
}

namespace {

void throwIfVelocityViolated(const model::Model& model, const Trajectory& trajectory) {
    const Limits<double> limit{.0, model.baseVelocityLimits().max};

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& state = trajectory[i];

        VERIFY_FMT(
            limit.isMet(state.velocity),
            "Violates velocity: [%i]:%s not in [%f, %f]",
            i,
            state.toString().c_str(),
            limit.min,
            limit.max);
    }
}

void throwIfAccelerationViolated(const model::Model& model, const Trajectory& trajectory) {
    const Limits<double> limit{
        trajectory.overbraking ? -std::numeric_limits<double>::infinity()
                               : -model.baseMaxDeceleration(),
        model.baseMaxAcceleration()};

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& state = trajectory[i];

        VERIFY_FMT(
            limit.isMet(state.acceleration),
            "Violates acceleration: [%i]:%s not in [%f, %f]",
            i,
            state.toString().c_str(),
            limit.min,
            limit.max);
    }
}

inline bool eq(double a, double b, double e = 1e-6) { return std::abs(a - b) < e; }

void throwIfInvalidDistance(const Trajectory& trajectory) {
    if (trajectory.empty()) {
        return;
    }

    VERIFY_FMT(
        trajectory.states.front().distance == 0,
        "Ego has non-zero distance: [0]:%s",
        trajectory.states.front().toString().c_str());

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& prev = trajectory[i - 1];
        const auto& state = trajectory[i];

        const double expected = geom::distance(prev.pose.pos, state.pose.pos);
        const double distance = state.distance - prev.distance;

        VERIFY_FMT(
            eq(distance, expected, 1e-3),
            "State has incorrect distance: [%i]:%s != %f",
            i,
            state.toString().c_str(),
            expected);
    }
}

void throwIfInvalidTime(const Trajectory& trajectory) {
    if (trajectory.empty()) {
        return;
    }

    const auto& front = trajectory.states.front();

    if (front.collision) {
        VERIFY_FMT(
            !front.reachable(),
            "State has collision, but reachable [0]:%s",
            front.toString().c_str());
    }

    if (front.reachable()) {
        VERIFY_FMT(front.getTime() == 0, "Ego has non-zero time: [0]:%s", front.toString().c_str());
    } else {
        VERIFY_FMT(
            eq(front.velocity, .0),
            "Ego is unreachable, but velocity: [0]:%s",
            front.toString().c_str());

        VERIFY_FMT(
            eq(front.acceleration, .0),
            "Ego is unreachable, but acceleration: [0]:%s",
            front.toString().c_str());
    }

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& prev = trajectory[i - 1];
        const auto& state = trajectory[i];

        if (state.collision) {
            VERIFY_FMT(
                !state.reachable(),
                "State has collision, but reachable [%i]:%s",
                i,
                state.toString().c_str());
        }

        if (!state.reachable()) {
            VERIFY_FMT(
                state.velocity == .0,
                "State is unreachable, but velocity [%i]:%s",
                i,
                state.toString().c_str());

            VERIFY_FMT(
                state.acceleration == .0,
                "State is unreachable, but acceleration [%i]:%f",
                i,
                state.toString().c_str());

            continue;
        }

        VERIFY(prev.reachable());

        VERIFY_FMT(
            prev.getTime() < state.getTime(),
            "Non-increasing time: [%i]:%s -> [%i]:%s",
            i - 1,
            prev.toString().c_str(),
            i,
            state.toString().c_str());

        const double dt = state.getTime() - prev.getTime();
        const double expected_velocity = prev.velocity + prev.acceleration * dt;

        VERIFY_FMT(
            eq(state.velocity, expected_velocity, 1e-3),
            "State [%i]:%s has incorrect velocity, expected %f",
            i,
            state.toString().c_str(),
            expected_velocity);

        const double expected_distance = state.distance - prev.distance;
        const double distance = dt * prev.velocity + 0.5 * dt * dt * prev.acceleration;

        VERIFY_FMT(
            eq(distance, expected_distance, 1e-3),
            "State [%i]:%s has incorrect distance, expected %f",
            i,
            state.toString().c_str(),
            expected_distance);
    }
}

void throwIfInvalid(
    const TrajectoryValidations& validations, const model::Model& model,
    const Trajectory& trajectory) {
    if (validations.distance) {
        throwIfInvalidDistance(trajectory);
    }

    if (validations.velocity) {
        throwIfVelocityViolated(model, trajectory);
    }

    if (validations.acceleration) {
        throwIfAccelerationViolated(model, trajectory);
    }

    if (validations.time) {
        throwIfInvalidTime(trajectory);
    }
}

}  // namespace

void Trajectory::throwIfInvalid(
    const TrajectoryValidations& validations, const model::Model& model) {
    motion::throwIfInvalid(validations, model, *this);
}

namespace {

constexpr double eps = 1e-6;

std::optional<size_t> getEgoSegmentIndex(
    const motion::States& states, const geom::Vec2& pos, double max_distance) {
    double min_distnace_sq = squared(max_distance);
    std::optional<size_t> ego_segment_idx = std::nullopt;

    for (size_t end = 1; end < states.size(); ++end) {
        const size_t begin = end - 1;

        const geom::Segment segment = {states[begin].pose.pos, states[end].pose.pos};
        const double distance_sq = geom::distanceSq(pos, segment);
        if (distance_sq < min_distnace_sq) {
            min_distnace_sq = distance_sq;
            ego_segment_idx = begin;
        }
    }

    return ego_segment_idx;
}

double findTime(double dist, double v, double a) {
    VERIFY(dist >= 0);
    VERIFY(v >= 0);

    if (std::abs(a) < eps) {
        return dist / v;
    }

    double d = v * v + 2 * a * dist;
    if (-eps <= d && d <= 0) {
        d = 0;
    }

    VERIFY_FMT(d >= 0.0, "%.10f: dist=%f v=%f a=%f", d, dist, v, a);
    return (-v + std::sqrt(d)) / a;
}

}  // namespace

State Trajectory::byTime(double time) const {
    VERIFY(!states.empty());
    VERIFY_FMT(time >= 0, "time = %.10f", "time");

    if (states.size() < 2) {
        return states.front();
    }

    const auto end = [&] {
        for (auto it = states.begin(); it != states.end(); ++it) {
            if (!it->reachable() || it->getTime() >= time) {
                return it;
            }
        }

        return states.end();
    }();

    if (end == states.begin()) {
        return states.front();
    }

    if (end == states.end()) {
        return states.back();
    }

    const auto begin = end - 1;
    if (!end->reachable()) {
        VERIFY(begin->reachable());
        return *begin;
    }

    VERIFY(begin->reachable());

    const double distance = end->distance - begin->distance;
    const double dt = time - begin->getTime();
    const double delta = dt * begin->velocity + 0.5 * dt * dt * begin->acceleration;
    const double ratio = clamp(delta / distance, 0.0, 1.0);

    return State{
        .pose = geom::interpolate(begin->pose, end->pose, ratio),
        .margin = std::min(begin->margin, end->margin),
        .collision = std::max(begin->collision, end->collision),
        .time = time,
        .distance = begin->distance + delta,
        .velocity = begin->velocity + dt * begin->acceleration,
        .acceleration = begin->acceleration,
    };
}

State Trajectory::byDistance(double distance) const {
    VERIFY(!states.empty());

    const auto end = std::find_if(states.begin(), states.end(), [distance](const auto& state) {
        return state.distance > distance;
    });

    if (end == states.begin()) {
        return states.front();
    }

    if (end == states.end()) {
        return states.back();
    }

    const auto begin = end - 1;
    const double delta = distance - begin->distance;
    const double ratio = clamp(delta / (end->distance - begin->distance), 0.0, 1.0);

    if (!begin->reachable() || !end->reachable()) {
        return State{
            .pose = geom::interpolate(begin->pose, end->pose, ratio),
            .margin = std::min(begin->margin, end->margin),
            .collision = std::max(begin->collision, end->collision),
            .time = std::nullopt,
            .distance = distance,
            .velocity = 0,
            .acceleration = 0,
        };
    }

    const double dt = findTime(delta, begin->velocity, begin->acceleration);

    return State{
        .pose = geom::interpolate(begin->pose, end->pose, ratio),
        .margin = std::min(begin->margin, end->margin),
        .collision = std::max(begin->collision, end->collision),
        .time = begin->getTime() + dt,
        .distance = distance,
        .velocity = begin->velocity + dt * begin->acceleration,
        .acceleration = begin->acceleration,
    };
}

std::optional<State> Trajectory::byProjection(const geom::Vec2& point, double max_distance) const {
    VERIFY(!states.empty());

    const auto index = getEgoSegmentIndex(states, point, max_distance);
    if (!index) {
        return std::nullopt;
    }

    const auto& begin = states[*index];
    const auto& end = states[*index + 1];

    const geom::Segment segment = {begin.pose.pos, end.pose.pos};
    const geom::Vec2 proj = geom::projection(point, segment);

    const double delta = geom::distance(begin.pose.pos, proj);
    const double ratio = clamp(delta / (end.distance - begin.distance), 0.0, 1.0);

    if (!begin.reachable() || !end.reachable()) {
        return State{
            .pose = geom::interpolate(begin.pose, end.pose, ratio),
            .margin = std::min(begin.margin, end.margin),
            .collision = std::max(begin.collision, end.collision),
            .time = std::nullopt,
            .distance = begin.distance + delta,
            .velocity = 0,
            .acceleration = 0,
        };
    }

    const double dt = findTime(delta, begin.velocity, begin.acceleration);

    return State{
        .pose = geom::interpolate(begin.pose, end.pose, ratio),
        .margin = std::min(begin.margin, end.margin),
        .collision = false,
        .time = begin.getTime() + dt,
        .distance = begin.distance + delta,
        .velocity = begin.velocity + dt * begin.acceleration,
        .acceleration = begin.acceleration,
    };
}

namespace {

std::optional<double> getTime(const truck_msgs::msg::TrajectoryState& msg) {
    return msg.reachable ? std::optional{msg.time} : std::nullopt;
}

}  // namespace

State toState(const truck_msgs::msg::TrajectoryState& msg) {
    return State{
        .pose = geom::toPose(msg.pose),
        .margin = msg.margin,
        .collision = msg.collision,
        .time = getTime(msg),
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

    msg.margin = state.margin;
    msg.collision = state.collision;

    msg.reachable = state.time.has_value();
    if (msg.reachable) {
        msg.time = *state.time;
    }

    msg.distance = state.distance;
    msg.velocity = state.velocity;
    msg.acceleration = state.acceleration;
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
