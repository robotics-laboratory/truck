#pragma once

#include "planning_interfaces/msg/point.hpp"
#include "pursuit_interfaces/msg/state.hpp"
#include "pursuit_interfaces/msg/command.hpp"

#include <vector>
#include <optional>

namespace pursuit {

struct Parameters {
    double max_velocity;
    double max_accel;
    double lookahead_distance;
};

class Controller {
private:
    Parameters params;
public:
    Controller(const Parameters &params): params{params} {}
    std::optional<pursuit_interfaces::msg::Command> get_motion(
          const pursuit_interfaces::msg::State &state
        , const std::vector<planning_interfaces::msg::Point> &path
    );
};

};
