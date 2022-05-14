#pragma once

#include "model/model.hpp"

namespace pure_pursuit {

struct MovingPlan {
    double velocity;
    double acceleration;
};

MovingPlan getPlanWithTimePrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model);

MovingPlan getPlanWithVelocityPrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model);

}
