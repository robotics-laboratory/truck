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
    state_.x = -params_.base_to_rear;
    state_.y = state_.rotation = state_.steering 
        = state_.linear_velocity = state_.angular_velocity = 0;
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    pose.dir = geom::Vec2(cos(state_.rotation), sin(state_.rotation));
    pose.pos.x = state_.x + params_.base_to_rear * pose.dir.x;
    pose.pos.y = state_.y + params_.base_to_rear * pose.dir.y;
    return pose;
}

geom::Angle SimulatorEngine::getSteering() const {
    return geom::Angle(state_.steering);
}

double SimulatorEngine::getSpeed() const { return state_[StateIndex::linear_velocity]; }

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2(cos(state_.linear_velocity), sin(state_.linear_velocity));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2(cos(state_.angular_velocity), sin(state_.angular_velocity));
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
    const double acceleration = abs(velocity - state_.linear_velocity) < params_.precision
        ? 0 
        : state_.linear_velocity < velocity
            ? limits.max
            : limits.min;
    setControl(velocity, acceleration, curvature);
}

void SimulatorEngine::calculate_state_delta(const SimulationState &state,
    const double velocity_delta, const double &steering_delta, SimulationState &delta) {
    
    delta.x = cos(state.rotation) * state.linear_velocity;
    delta.y = sin(state.rotation) * state.linear_velocity;
    delta.rotation = tan(state.steering) * state.linear_velocity / params_.wheelbase;
    delta.steering = steering_delta;
    delta.linear_velocity = velocity_delta;
}

void SimulatorEngine::advance(const double time) {
    const int integration_steps = time / params_.integration_step;

    const double old_rotation = state_.rotation;

    const double steering_final = abs(control_.curvature) < params_.precision 
        ? 0 
        : std::clamp(control_.curvature > 0
            ? atan2(params_.wheelbase, control_.curvature)
            : -atan2(params_.wheelbase, -control_.curvature), 
        -params_.steering_limit, +params_.steering_limit);
        
    double steering_delta = abs(steering_final - state_.steering) < params_.precision 
        ? 0 
        : state_.steering < steering_final
            ? params_.steering_velocity
            : -params_.steering_velocity;
    steering_delta = steering_delta * time / integration_steps;

    double velocity_delta = abs(control_.velocity - state_.linear_velocity) < params_.precision 
        ? 0 
        : control_.acceleration;
    velocity_delta = velocity_delta * time / integration_steps;

    SimulationState k[4];
    for (auto i = 0; i < integration_steps; ++i) {
        const double steering_rest 
            = abs(steering_final - state_.steering) - abs(steering_delta);
        if (steering_rest < params_.precision) {
            steering_delta = steering_final - state_.steering;
        }

        const double velocity_rest 
            = abs(control_.velocity - state_.linear_velocity) - abs(velocity_delta);
        if (velocity_rest < params_.precision) {
            velocity_delta = control_.velocity - state_.linear_velocity;
        }
        
        calculate_state_delta(state_, velocity_delta, steering_delta, k[0]);
        calculate_state_delta(state_ + k[0] * (params_.integration_step / 2),
            velocity_delta, steering_delta, k[1]);
        calculate_state_delta(state_ + k[1] * (params_.integration_step / 2),
            velocity_delta, steering_delta, k[2]);
        calculate_state_delta(state_ + k[2] * params_.integration_step,
            velocity_delta, steering_delta, k[3]);
        k[1] *= 2;
        k[2] *= 2;
        SimulationState::addSum(state_, k, 4, params_.integration_step / 6);

        state_.rotation = fmod(state_.rotation, 2 * M_PI);
        if (state_.rotation < 0) {
            state_.rotation += 2 * M_PI;
        }
    }

    state_.angular_velocity = (state_.rotation - old_rotation) / time;
}

} // namespace truck::simulator