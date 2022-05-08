#pragma once

#include <vector>
#include <cinttypes>
#include <variant>
#include <utility>
#include <string>

#include "model/model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "controller.hpp"
#include "result.hpp"

namespace pure_pursuit {

enum class SimulationError {
    CONTROLLER_FAILED = 0,
    FINISH_POINT_IS_NOT_ARRIVED = 1
};

inline std::string error_to_string(SimulationError e) {
    switch (SimulationError{static_cast<int>(e) & 1}) {
    case SimulationError::CONTROLLER_FAILED:
        return "Controller failed. Reason: " + error_to_string(ControllerError{static_cast<int>(e) >> 1});
    case SimulationError::FINISH_POINT_IS_NOT_ARRIVED:
        return "Finish point is not arrived in time";
    default:
        return "Unknown error";
    }
}

using SimulationResult = Result<std::vector<nav_msgs::msg::Odometry>, SimulationError>;

SimulationResult simulate(nav_msgs::msg::Odometry start,
                                              nav_msgs::msg::Odometry finish, uint64_t sim_timeout_ns, uint64_t sim_dt_ns, uint64_t controller_period,
                                              const model::Model &params);

};
