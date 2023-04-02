#pragma once

#include "geom/pose.h"
#include "model/model.h"

#include "truck_msgs/msg/trajectory.hpp"

namespace truck::motion {

struct State {
    geom::Pose pose;

    double margin = 0;
    bool collision = false;

    double time = 0;
    double distance = 0;
    double velocity = 0;
    double acceleration = 0;
};

using States = std::vector<State>;

struct Trajectory {
    bool empty() const { return states.empty(); }
    size_t size() const { return states.size(); }

    const State& operator[](size_t index) const { return states.at(index); }

    // linear interpolation by time
    State operator()(double time) const;

    // contruction helpers
    void fillDistance();

    // check main requqirements
    void throwIfInvalid(const model::Model& model);

    bool overbraking = false;
    States states = {};
};

State toState(const truck_msgs::msg::TrajectoryState& state);
States toStates(const std::vector<truck_msgs::msg::TrajectoryState>& states);
Trajectory toTrajectory(const truck_msgs::msg::Trajectory& trajectory);

namespace msg {

truck_msgs::msg::TrajectoryState toTrajectoryState(const State& state);

truck_msgs::msg::Trajectory toTrajectory(const Trajectory& trajectory);

truck_msgs::msg::Trajectory toTrajectory(
    const std_msgs::msg::Header& header, const Trajectory& trajectory);

}  // namespace msg
}  // namespace truck::motion