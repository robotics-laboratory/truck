#pragma once

#include "truck_interfaces/msg/control.hpp"

#include "common/math.h"
#include "common/result.h"
#include "model/model.h"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <vector>
#include <optional>
#include <string_view>

namespace truck::pure_pursuit {

enum class ControllerError: uint8_t {
    kUnknown = 0,
    kUnreachablePath = 1,
    kImpossibleBuildArc = 2,
};

std::string_view toString(ControllerError e);

struct Command {
    double curvature = 0;
    double velocity = 0;

    std::optional<geom::Vec2> target = std::nullopt;

    static Command stop() { return Command{}; }
};

using ControllerResult = common::Result<Command, ControllerError>;

struct Params {
    Limits<double> radius = {0.15, 0.5};
    double velocity = 0.4;
    double velocity_factor = 0.2;
    double tolerance = 0.1;
};

class Controller {
  public:
    Controller(const Params& params, const model::Model& model)
        : params_{params}
        , model_(model)
    {}

    ControllerResult
    operator()(const nav_msgs::msg::Odometry& odometry, const nav_msgs::msg::Path& path);

  private:
    double getRadius(double velocity) const;

    const Params params_;
    const model::Model model_;
};

}  // namespace truck::pure_pursuit
