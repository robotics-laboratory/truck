#pragma once

#include "geom/pose.h"
#include "common/exception.h"
#include "model/model.h"

#include "truck_msgs/msg/trajectory.hpp"

#include <optional>
#include <ostream>

namespace truck::motion {

struct State {
    geom::Pose pose;

    double margin = 0;
    bool collision = false;

    std::optional<double> time = std::nullopt;
    double distance = 0;
    double velocity = 0;
    double acceleration = 0;

    bool reachable() const noexcept { return time.has_value(); }

    double getTime() const { return *VERIFY(time); }

    std::string toString() const noexcept;
};

std::ostream& operator<<(std::ostream& out, const State& state) noexcept;

using States = std::vector<State>;

struct TrajectoryValidations {
    bool distance = false;
    bool velocity = false;
    bool acceleration = false;
    bool time = false;

    static auto enableAll() {
        return TrajectoryValidations{
            .distance = true,
            .velocity = true,
            .acceleration = true,
            .time = true,
        };
    }
};

struct Trajectory {
    bool empty() const { return states.empty(); }
    size_t size() const { return states.size(); }

    const State& operator[](size_t index) const { return states.at(index); }

    State byDistance(double distance) const;

    std::optional<State> byProjection(
        const geom::Vec2& point, double max_distance = std::numeric_limits<double>::max()) const;

    State byTime(double time) const;

    // contruction helpers
    void fillDistance();

    // check main requqirements
    void throwIfInvalid(const TrajectoryValidations& validations, const model::Model& model) const;

    bool overbraking = false;
    States states = {};
};

State toState(const truck_msgs::msg::TrajectoryState& msg);
States toStates(const std::vector<truck_msgs::msg::TrajectoryState>& msgs);
Trajectory toTrajectory(const truck_msgs::msg::Trajectory& msg);

namespace msg {

truck_msgs::msg::TrajectoryState toTrajectoryState(const State& state);

truck_msgs::msg::Trajectory toTrajectory(const Trajectory& trajectory);

truck_msgs::msg::Trajectory toTrajectory(
    const std_msgs::msg::Header& header, const Trajectory& trajectory);

}  // namespace msg
}  // namespace truck::motion
