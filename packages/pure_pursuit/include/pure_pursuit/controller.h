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
    double acceleration = 0;

    static Command stop() { return Command{}; }
};

struct ControllerResultData {
    truck_interfaces::msg::Control cmd;
};

using ControllerResult = common::Result<Command, ControllerError>;

struct Params {
    std::string model_path = "";
    Limits<double> radius = {0.1, 0.3};
    double velocity_factor = 0.1;
    double tolerance = 0.15;
};

class Controller {
  public:
    Controller(const Params& params) : params_{params}, model_(params.model_path) {}

    ControllerResult operator()(
        const nav_msgs::msg::Odometry& odometry,
        const nav_msgs::msg::Path& path);

  private:
    double getRadius(double velocity) const;

    Params params_;
    model::Model model_;
};

};  // namespace truck::pure_pursuit
