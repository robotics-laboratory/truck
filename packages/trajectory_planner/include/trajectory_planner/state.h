#pragma once

#include "common/math.h"

#include "geom/angle.h"
#include "geom/localization.h"
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

template<typename T>
struct Discretization {
    T Step() const noexcept { return (limits.max - limits.min) / total_states; }

    T operator[](int index) const {
        VERIFY(index < total_states);
        return limits.min + Step() * index;
    }

    int operator()(const T& value) const {
        VERIFY(limits.min <= value && value < limits.max);
        return static_cast<size_t>(limits.ratio(value) * total_states);
    }

    Limits<T> limits;
    int total_states;
};

using State = geom::Localization;

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
        int Size() const noexcept;

        Discretization<double> longitude = {.limits = Limits<double>(-10, 11), .total_states = 10};
        Discretization<double> latitude = {.limits = Limits<double>(-5, 6), .total_states = 10};
        int total_forward_yaw_states = 5;
        int total_backward_yaw_states = 3;
        Discretization<double> velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10};
    };

    StateSpace& Build(
        const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept;

    int Size() const noexcept;

    StateSpace& Clear() noexcept;

    StateSpace& Reset(State* ptr) noexcept;

    Params params;

    States start_states;
    States finish_states;
    States regular_states;

    TruckState truck_state;

    State* data = nullptr;
};

using StateSpaceDataPtr = std::unique_ptr<State[]>;

struct StateSpaceHolder {
    StateSpaceHolder() = default;

    StateSpaceHolder(StateSpace&& state_space, StateSpaceDataPtr&& states_ptr) noexcept;

    StateSpaceHolder(const StateSpaceHolder& other) = delete;

    StateSpaceHolder(StateSpaceHolder&& other) = default;

    StateSpaceHolder& operator=(const StateSpaceHolder&) = delete;

    StateSpaceHolder& operator=(StateSpaceHolder&& other) & = default;

    ~StateSpaceHolder() = default;

    StateSpace state_space;
    StateSpaceDataPtr states_ptr = nullptr;
};

StateSpaceHolder MakeStateSpace(const StateSpace::Params& params);

}  // namespace truck::trajectory_planner