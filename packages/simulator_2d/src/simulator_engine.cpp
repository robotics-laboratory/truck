#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

void SimulatorEngine::start(std::unique_ptr<model::Model> &model, 
    const double integration_step, const double precision) {

    model_ = std::unique_ptr<model::Model>(std::move(model));
    params_.integration_step = integration_step;
    params_.precision = precision;
    params_.steering_velocity = model_->steeringVelocity();
    params_.wheelbase = model_->wheelBase().length;
    params_.base_to_rear = model_->wheelBase().base_to_rear;
    params_.steering_limit = model_->leftSteeringLimits().max.radians();
    reset();
}


void SimulatorEngine::reset() {
    state_ = Eigen::Matrix<double, 6, 1>::Zero();
    state_[StateIndex::x] = -params_.base_to_rear;
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    pose.dir = geom::Vec2(cos(state_[StateIndex::rotation]), sin(state_[StateIndex::rotation]));
    pose.pos.x = state_[StateIndex::x] + params_.base_to_rear * pose.dir.x;
    pose.pos.y = state_[StateIndex::y] + params_.base_to_rear * pose.dir.y;
    return pose;
}

geom::Angle SimulatorEngine::getSteering() const {
    return geom::Angle(state_[StateIndex::steering]);
}

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2(cos(state_[StateIndex::linear_velocity]), sin(state_[StateIndex::linear_velocity]));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2(cos(state_[StateIndex::angular_velocity]), sin(state_[StateIndex::angular_velocity]));
}

void SimulatorEngine::setControl(
    const double velocity, const double acceleration, const double curvature) {

    control_.velocity = model_->baseVelocityLimits().clamp(velocity);
    control_.acceleration = model_->baseAccelerationLimits().clamp(acceleration);
    const auto curvature_limit = model_->baseMaxAbsCurvature();
    control_.curvature 
        = std::clamp(curvature, -curvature_limit, curvature_limit);
}

void SimulatorEngine::setControl(
    const double velocity, const double curvature) {
    
    const auto limits = model_->baseAccelerationLimits();
    const double acceleration = abs(velocity - state_[StateIndex::linear_velocity]) < params_.precision
        ? 0 
        : state_[StateIndex::linear_velocity] < velocity
            ? limits.max
            : limits.min;
    setControl(velocity, acceleration, curvature);
}

Eigen::Matrix<double, 6, 1> SimulatorEngine::calculate_state_delta(
    const Eigen::Matrix<double, 6, 1> &state, 
    const double velocity_delta, const double steering_delta) {
    
    Eigen::Matrix<double, 6, 1> delta;
    delta[StateIndex::x] = cos(state[StateIndex::rotation]) * state[StateIndex::linear_velocity];
    delta[StateIndex::y] = sin(state[StateIndex::rotation]) * state[StateIndex::linear_velocity];
    delta[StateIndex::rotation] = tan(state[StateIndex::steering]) 
        * state[StateIndex::linear_velocity] / params_.wheelbase;
    delta[StateIndex::steering] = steering_delta;
    delta[StateIndex::linear_velocity] = velocity_delta;
    return delta;
}

void SimulatorEngine::advance(const double time) {
    const int integration_steps = time / params_.integration_step;

    const double old_rotation = state_[StateIndex::rotation];

    const double steering_final = abs(control_.curvature) < params_.precision 
        ? 0 
        : std::clamp(control_.curvature > 0
            ? atan2(params_.wheelbase, control_.curvature)
            : -atan2(params_.wheelbase, -control_.curvature), 
        -params_.steering_limit, +params_.steering_limit);
        
    double steering_rest = abs(steering_final - state_[StateIndex::steering]);
    double steering_delta = steering_rest < params_.precision 
        ? 0 
        : state_[StateIndex::steering] < steering_final
            ? params_.steering_velocity
            : -params_.steering_velocity;
    steering_delta *= params_.integration_step;

    double velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]);
    double velocity_delta = velocity_rest < params_.precision 
        ? 0 
        : control_.acceleration;
    velocity_delta *= params_.integration_step;

    for (auto i = 0; i < integration_steps; ++i) {
        steering_rest 
            = abs(steering_final - state_[StateIndex::steering]) - abs(steering_delta);
        if (steering_rest < params_.precision) {
            steering_delta = steering_final - state_[StateIndex::steering];
        }

        velocity_rest 
            = abs(control_.velocity - state_[StateIndex::linear_velocity]) - abs(velocity_delta);
        if (velocity_rest < params_.precision) {
            velocity_delta = control_.velocity - state_[StateIndex::linear_velocity];
        }
        
        auto k1 = calculate_state_delta(state_, velocity_delta, steering_delta);
        auto k2 = calculate_state_delta(state_ + k1 * (params_.integration_step / 2),
            velocity_delta, steering_delta);
        auto k3 = calculate_state_delta(state_ + k2 * (params_.integration_step / 2),
            velocity_delta, steering_delta);
        auto k4 = calculate_state_delta(state_ + k3 * params_.integration_step,
            velocity_delta, steering_delta);

        state_ += (k1 + 2 * k2 + 2 * k3 + k4) * params_.integration_step / 6;

        state_[StateIndex::rotation] = fmod(state_[StateIndex::rotation], 2 * M_PI);
        if (state_[StateIndex::rotation] < 0) {
            state_[StateIndex::rotation] += 2 * M_PI;
        }
    }

    state_[StateIndex::angular_velocity] = (state_[StateIndex::rotation] - old_rotation) / time;
}

} // namespace truck::simulator