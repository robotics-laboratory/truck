#pragma once

#include "common/math.h"

#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/polyline.h"

#include "collision/collision_checker.h"

#include "model/model.h"

#include <memory>

namespace truck::trajectory_planner {

struct TruckState {
    struct Params {
        double min_dist_to_obstacle = 0.1;
    };

    TruckState() = default;

    TruckState(const Params& params);

    bool IsCollisionFree(const geom::Pose& pose) const noexcept;

    Params params;
    const collision::StaticCollisionChecker* collision_checker = nullptr;
    const model::Model* model = nullptr;
};

struct State {
    class Estimator;

    geom::Pose pose;
    double velocity;
};

struct StateArea {
    bool IsInside(const State& state) const noexcept;

    State base_state;
    Limits<double> x_range;
    Limits<double> y_range;
    Limits<geom::Angle> yaw_range;
    Limits<double> velocity_range;
};

struct States {
    State* data = nullptr;
    int size = 0;
};

struct StateSpace {
    struct Params {
        size_t Size() const noexcept;

        Discretization<double> longitude = {.limits = Limits<double>(-10, 11), .total_states = 10};
        Discretization<double> latitude = {.limits = Limits<double>(-5, 6), .total_states = 10};
        size_t total_forward_yaw_states = 5;
        size_t total_backward_yaw_states = 3;
        Discretization<double> velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10};
    };

    StateSpace& Build(
        const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept;

    int Size() const noexcept;

    StateSpace& Clear() noexcept;

    Params params;

    States start_states;
    States finish_states;
    States regular_states;

    TruckState truck_state;

    States data;
};

struct StateSpaceHolder {
    StateSpaceHolder(int size);

    StateSpaceHolder(const StateSpace::Params& params);

    StateSpaceHolder(const StateSpaceHolder& other) = delete;

    StateSpaceHolder(StateSpaceHolder&& other) = default;

    StateSpaceHolder& operator=(const StateSpaceHolder&) = delete;

    StateSpaceHolder& operator=(StateSpaceHolder&& other) & = default;

    ~StateSpaceHolder() = default;

    StateSpace state_space;
    std::unique_ptr<State[]> states_ptr = nullptr;
};

}  // namespace truck::trajectory_planner