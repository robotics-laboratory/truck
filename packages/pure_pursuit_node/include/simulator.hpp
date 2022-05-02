#pragma once

#include <vector>
#include <cinttypes>
#include <variant>
#include <utility>
#include <string>

#include "model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "controller.hpp"
#include "result.hpp"

namespace pure_pursuit {

using SimulationResult = Result<std::vector<nav_msgs::msg::Odometry>, std::string>;

SimulationResult simulate(nav_msgs::msg::Odometry start,
                                              nav_msgs::msg::Odometry finish, uint64_t sim_timeout_ns, uint64_t sim_dt_ns, uint64_t controller_period,
                                              const model::Model &params);

};
