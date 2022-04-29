#pragma once

#include <vector>

#include "model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "controller.hpp"

namespace pure_pursuit {

std::vector<nav_msgs::msg::Odometry> simulate(nav_msgs::msg::Odometry start,
                                              nav_msgs::msg::Odometry finish, double sim_dt, int controller_freq,
                                              const model::Model &params);

};
