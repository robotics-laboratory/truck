#pragma once

#include "common/math.h"
#include "common/result.h"
#include "geom/localization.h"
#include "model/model.h"
#include "motion/trajectory.h"

#include <nav_msgs/msg/path.hpp>

#include <chrono>
#include <cstdint>
#include <vector>
#include <optional>
#include <string_view>

namespace truck::pure_pursuit {

enum class Error : uint8_t {
    kUnknown,
    kNoProjection,
    kUnreachableProjection,
    kImpossibleBuildArc,
};

std::string_view toString(Error e);

struct Command {
    double curvature = 0;
    double velocity = 0;
    double acceleration = 0;

    std::optional<geom::Vec2> target = std::nullopt;

    static Command stop() { return Command{}; }
};

using Result = common::Result<Command, Error>;

using namespace std::chrono_literals;

struct Parameters {
    std::chrono::duration<double> period = 0.1s;
    Limits<double> radius = {0.15, 0.5};
    double velocity_factor = 0.2;
    double tolerance = 0.2;
    double max_distance = 0.30;
};

class PurePursuit {
  public:
    PurePursuit(const Parameters& params, const model::Model& model)
        : params_{params}, model_(model) {}

    Result operator()(const geom::Localization& localization, const motion::Trajectory& trajectory);

    Result command(const geom::Localization& localization, const motion::Trajectory& trajectory) {
        return (*this)(localization, trajectory);
    }

  private:
    double getRadius(double velocity) const;

    const Parameters params_;
    const model::Model model_;
};

}  // namespace truck::pure_pursuit
