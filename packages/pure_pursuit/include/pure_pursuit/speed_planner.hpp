#pragma once

#include "model/model.hpp"

namespace pure_pursuit {

struct SpeedPlan {
    double velocity;
    double acceleration;
};

SpeedPlan getPlanWithTimePrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model);

SpeedPlan getPlanWithVelocityPrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model);

}
