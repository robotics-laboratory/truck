#pragma once

#include <vector>
#include <cinttypes>
#include <variant>
#include <utility>
#include <string>

#include "model/model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit/controller.hpp"
#include "pure_pursuit/result.hpp"

namespace pure_pursuit {

enum class SimulationError : int8_t { kControllerFailed = 0, kFinishPointNotArrived = 1 };

inline std::string errorToString(SimulationError e) {
    switch (SimulationError{static_cast<int>(e) & 1}) {
        case SimulationError::kControllerFailed:
            return "Controller failed. Reason: "
                   + errorToString(ControllerError{static_cast<int>(e) >> 1});
        case SimulationError::kFinishPointNotArrived:
            return "Finish point is not arrived in time";
        default:
            return "Unknown error";
    }
}

using SimulationResult = Result<std::vector<nav_msgs::msg::Odometry>, SimulationError>;

SimulationResult simulate(
    const nav_msgs::msg::Odometry& start, const nav_msgs::msg::Odometry& finish,
    uint64_t sim_timeout_ns, uint64_t sim_dt_ns, uint64_t controller_period,
    const model::Model& params);

};  // namespace pure_pursuit
