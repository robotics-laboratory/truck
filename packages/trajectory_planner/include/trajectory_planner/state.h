#pragma once

#include "common/math.h"

#include "geom/pose.h"

#include <optional>
#include <vector>

namespace truck::trajectory_planner {

template<typename T>
struct Discretization {
    T operator[](size_t index) const {
        VERIFY(index < total_states);
        return limits.min + (limits.max - limits.min) / total_states * index;
    }

    size_t operator()(T value) const {
        VERIFY(limits.min <= value && value < limits.max);
        return (value - limits.min) / (limits.max - limits.min) * total_states;
    }

    Limits<T> limits;
    size_t total_states;
};

struct State {
    geom::Pose pose;
    double velocity;
};

using States = std::vector<State>;

geom::Poses FindMotion(
    const geom::Pose& from, const geom::Pose& to, size_t max_step, double eps = 1e-7);

double MotionLength(const geom::Poses& motion, double inf = 1e18);

double MotionTime(
    double motion_length, double form_velocity, double to_velocity, double eps = 1e-7,
    double inf = 1e18);

}  // namespace truck::trajectory_planner