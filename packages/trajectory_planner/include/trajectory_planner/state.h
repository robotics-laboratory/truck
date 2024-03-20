#pragma once

#include "common/math.h"
#include "common/exception.h"

#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/polyline.h"

#include <unordered_set>
#include <vector>

namespace truck::trajectory_planner {

struct State {
    geom::Pose pose;
    double velocity;
};

using States = std::vector<State>;

struct StateArea {
    bool IsInside(const State& state) const noexcept;

    State base_state;
    Limits<double> x_range;
    Limits<double> y_range;
    Limits<geom::Angle> yaw_range;
    Limits<double> velocity_range;
};

class StateSpace {
  public:
    struct Params {
        Discretization<double> longitude = {.limits = Limits<double>(-10, 11), .total_states = 10};
        Discretization<double> latitude = {.limits = Limits<double>(-5, 6), .total_states = 10};
        Discretization<geom::Angle> forward_yaw = {
            .limits = Limits<geom::Angle>(-geom::PI_2, geom::PI_2), .total_states = 5};
        Discretization<geom::Angle> backward_yaw = {
            .limits = Limits<geom::Angle>(geom::PI_2, 3 * geom::PI_2), .total_states = 3};
        Discretization<double> velocity = {.limits = Limits<double>(0.0, 0.8), .total_states = 10};
    };

    StateSpace(const Params& params);

    StateSpace(const StateSpace& other);

    StateSpace(StateSpace&& other) = default;

    StateSpace& operator=(StateSpace other) &;

    StateSpace& Build(
        const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept;

    const StateSpace::Params& GetParams() const noexcept;

    const State& GetStartState() const noexcept;

    const std::unordered_set<const State*>& GetFinishStates() const noexcept;

    const States& GetStates() const noexcept;

    ~StateSpace() = default;

  private:
    Params params_;

    States states_;
    std::unordered_set<const State*> finish_states_;
};

geom::Poses FindMotion(
    const geom::Pose& from, const geom::Pose& to, size_t max_step, double eps = 1e-7);

double MotionLength(const geom::Poses& motion, double inf = 1e18);

double MotionTime(
    double motion_length, double form_velocity, double to_velocity, double eps = 1e-7,
    double inf = 1e18);

}  // namespace truck::trajectory_planner