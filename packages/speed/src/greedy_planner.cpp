#include "speed/greedy_planner.h"

#include "common/exception.h"

#include <algorithm>
#include <cmath>

namespace truck::speed {

namespace {

constexpr double eps = 1e-6;

double findTime(double dist, double v, double a) {
    VERIFY(dist >= 0);
    VERIFY(v >= 0);

    if (std::abs(a) < eps) {
        return dist / v;
    }

    double d = v * v + 2 * a * dist;
    if (-eps <= d && d <= 0) {
        d = 0.0;
    }

    VERIFY_FMT(d >= 0.0, "%.10f: dist=%f v=%f a=%f", d, dist, v, a);
    return (-v + std::sqrt(d)) / a;
}

double findAcceleration(double dist, double v_start, double v_finish) {
    VERIFY(dist >= 0);
    VERIFY(v_start >= 0);
    VERIFY(v_finish >= 0);

    return (v_finish * v_finish - v_start * v_start) / (2 * dist);
}

using It = motion::States::iterator;

It findTrajectoryEnd(It begin, It end, double distance_to_obstacle) {
    auto it = std::find_if(begin, end, [](const auto& state) { return state.collision; });

    if (it == end) {
        return end;
    }

    return std::find_if(begin, it, [&](const auto& state) {
        return it->distance - state.distance < distance_to_obstacle;
    });
}

// return last acceleration state
It fillAcceleration(
    const model::Model& model, double scheduled_velocity, double desired_acceleration, It begin,
    It end) {
    VERIFY(begin != end);

    if (scheduled_velocity + eps > model.baseVelocityLimits().max) {
        begin->time = 0;
        begin->velocity = model.baseVelocityLimits().max;
        begin->acceleration = 0;
        return begin;
    }

    auto it = begin;
    it->time = 0;
    it->velocity = scheduled_velocity;
    it->acceleration = 0;

    for (auto next = it + 1; next != end;) {
        const double distance = next->distance - it->distance;
        double dt = findTime(distance, it->velocity, desired_acceleration);

        VERIFY(it->reachable());

        it->acceleration = desired_acceleration;
        next->velocity = it->velocity + it->acceleration * dt;
        next->time = it->getTime() + dt;

        // reach max velocity
        if (next->velocity + eps > model.baseVelocityLimits().max) {
            next->velocity = model.baseVelocityLimits().max;
            it->acceleration = findAcceleration(distance, it->velocity, next->velocity);

            dt = abs(it->acceleration) > eps ? (next->velocity - it->velocity) / it->acceleration
                                             : distance / it->velocity;

            next->time = it->getTime() + dt;
            next->acceleration = 0;

            return next;
        }

        it = next;
        ++next;
    }

    return it;
}

// copy velocity from begin state
void fillConstVelocity(It begin, It end) {
    VERIFY(begin != end);

    auto it = begin;
    for (auto next = it + 1; next != end;) {
        VERIFY(it->reachable());

        const double distance = next->distance - it->distance;
        next->velocity = it->velocity;
        next->acceleration = 0;
        next->time = it->getTime() + distance / it->velocity;

        it = next;
        ++next;
    }
}

It findBrakingBegin(double desired_deceleration, It begin, It end) {
    VERIFY(begin != end);
    const auto last = end - 1;

    const auto it = std::find_if(begin, last, [&](const auto& state) {
        return findAcceleration(last->distance - state.distance, state.velocity, 0)
               <= desired_deceleration;
    });

    return it == begin ? begin : it - 1;
}

bool fillBraking(const model::Model& model, It begin, It end) {
    VERIFY(begin != end);
    const auto last = end - 1;

    if (begin == last) {
        begin->velocity = 0;
        begin->acceleration = 0;
        return false;
    }

    const double a = findAcceleration(last->distance - begin->distance, begin->velocity, 0);

    auto it = begin;
    for (auto next = it + 1; next != end;) {
        VERIFY(it->reachable());

        const double distance = next->distance - it->distance;
        double dt = findTime(distance, it->velocity, a);

        it->acceleration = a;
        next->velocity = it->velocity + a * dt;
        next->time = it->getTime() + dt;

        it = next;
        ++next;
    }

    // double check for last state
    it->acceleration = 0;
    it->velocity = 0;

    // check exactly model limits, but not comfort deceleration
    return a < -model.baseMaxDeceleration();
}

void fillAfterEnd(It begin, It end) {
    for (auto it = begin; it != end; ++it) {
        it->time = std::nullopt;
        it->velocity = 0;
        it->acceleration = 0;
    }
}

}  // namespace

GreedyPlanner::GreedyPlanner(const Params& params, const model::Model& model) :
    params_(params), model_(model) {}

void GreedyPlanner::fill(motion::Trajectory& trajectory) const {
    auto& states = trajectory.states;
    if (states.size() < 2) {
        return;  // impossible plan velocity
    }

    const auto begin = states.begin();
    const auto end = findTrajectoryEnd(begin, states.end(), params_.distance_to_obstacle);

    // check collision at the first pose
    if (begin != end) {
        const auto acceleration_last =
            fillAcceleration(model_, scheduled_velocity_, params_.acceleration.max, begin, end);

        fillConstVelocity(acceleration_last, end);

        const auto braking_begin = findBrakingBegin(params_.acceleration.min, begin, end);

        trajectory.overbraking = fillBraking(model_, braking_begin, end);
    }

    fillAfterEnd(end, trajectory.states.end());
}

}  // namespace truck::speed
