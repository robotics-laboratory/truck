#pragma once

#include <vector>
#include <cinttypes>

#include "model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "controller.hpp"

namespace pure_pursuit {

std::vector<nav_msgs::msg::Odometry> simulate(nav_msgs::msg::Odometry start,
                                              nav_msgs::msg::Odometry finish, uint64_t sim_dt_ns, uint64_t controller_period,
                                              const model::Model &params);

};
