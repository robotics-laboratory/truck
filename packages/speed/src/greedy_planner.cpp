#include "speed/greedy_planner.h"

#include "common/exception.h"

#include <algorithm>
#include <cmath>

namespace truck::speed {

namespace {

constexpr double eps = 1e-6;

double findTime(double dist, double v, double a) {
    return std::abs(a) > eps ? (-v + std::sqrt(v * v + 2 * a * dist)) / a : dist / v;
}

double findAcceleration(double dist, double v_start, double v_finish) {
    VERIFY(dist > 0);
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
It fillAcceleration(const model::Model& model, double scheduled_velocity, It begin, It end) {
    VERIFY(begin < end);

    if (scheduled_velocity + eps > model.baseVelocityLimits().max) {
        begin->velocity = model.baseVelocityLimits().max;
        begin->acceleration = 0;
        return begin;
    }

    auto it = begin;
    it->velocity = scheduled_velocity;

    for (auto next = it + 1; next != end; ++it, ++next) {
        const double distance = next->distance - it->distance;
        it->acceleration = model.baseAccelerationLimits().max;
        double dt = findTime(distance, it->velocity, it->acceleration);
        next->velocity = it->velocity + it->acceleration * dt;
        next->time = it->time + dt;

        // reach max velocity
        if (next->velocity + eps > model.baseVelocityLimits().max) {
            next->velocity = model.baseVelocityLimits().max;
            it->acceleration = findAcceleration(distance, it->velocity, next->velocity);

            VERIFY(abs(it->acceleration) > eps);
            dt = (next->velocity - it->velocity) / it->acceleration;
            next->time = it->time + dt;
            next->acceleration = 0;

            return next;
        }
    }

    return it;
}

// copy velocity from begin state
void fillConstVelocity(It begin, It end) {
    VERIFY(begin < end);

    auto it = begin;
    for (auto next = it + 1; next != end; ++it, ++next) {
        const double distance = next->distance - it->distance;
        next->velocity = it->velocity;
        next->acceleration = 0;
        next->time = it->time + distance / it->velocity;
    }
}

It findBrakingBegin(const model::Model& model, It begin, It end) {
    VERIFY(begin < end);
    const auto last = end - 1;

    const auto it = std::find_if(begin, last, [&](const auto& state) {
        return findAcceleration(last->distance - state.distance, state.velocity, 0) <=
               model.baseAccelerationLimits().min;
    });

    return it == begin ? begin : it - 1;
}

bool fillBraking(const model::Model& model, It begin, It end) {
    VERIFY(begin < end);
    const auto last = end - 1;

    const double a = findAcceleration(last->distance - begin->distance, begin->velocity, 0);

    auto it = begin;
    for (auto next = it + 1; next != end; ++it, ++next) {
        const double distance = next->distance - it->distance;
        it->acceleration = a;
        double dt = findTime(distance, it->velocity, it->acceleration);
        next->velocity = it->velocity + it->acceleration * dt;
        next->time = it->time + dt;
    }

    // double check
    it->acceleration = 0;
    it->velocity = 0;

    return a < model.baseAccelerationLimits().min;
}

void fillAfterCollision(It begin, It end) {
     for (auto it = begin; it != end; ++it) {
        it->velocity = 0;
        it->acceleration = 0;
        it->time = std::numeric_limits<double>::infinity();
    }
}

}  // namespace

GreedyPlanner::GreedyPlanner(const Parameters& params, const model::Model& model)
    : params_(params), model_(model) {}

void GreedyPlanner::fill(motion::Trajectory& trajectory) const {
    if (trajectory.states.size() < 2) {
        return;  // impossible plan velocity
    }

    const auto begin = trajectory.states.begin();
    const auto end =
        findTrajectoryEnd(begin, trajectory.states.end(), params_.distance_to_obstacle);

    if (begin == end) {
        return;  // collision at the first pose
    }

    const auto acceleration_last = fillAcceleration(model_, scheduled_velocity_, begin, end);
    fillConstVelocity(acceleration_last, end);

    const auto braking_begin = findBrakingBegin(model_, begin, end);
    trajectory.overbraking = fillBraking(model_, braking_begin, end);

    fillAfterCollision(end, trajectory.states.end());
}

}  // namespace truck::speed