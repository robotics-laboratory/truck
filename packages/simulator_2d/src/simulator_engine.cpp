#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

void SimulatorEngine::start(
    std::unique_ptr<model::Model> &model, const double integration_step, const double precision) {
    
    model_ = std::unique_ptr<model::Model>(std::move(model));
    params_.integration_step = integration_step;
    params_.precision = precision;
    params_.max_steering_velocity = model_->steeringVelocity();
    params_.wheelbase = model_->wheelBase().length;
    params_.base_to_rear = model_->wheelBase().base_to_rear;
    params_.steering_limit = model_->leftSteeringLimits().max.radians();
    reset();
}

void SimulatorEngine::reset() {
    state_ = SimulatorEngine::State::Zero();
    state_[StateIndex::x] = -params_.base_to_rear;
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    pose.dir = geom::AngleVec2(geom::Angle::fromRadians(state_[StateIndex::yaw]));
    pose.pos.x = state_[StateIndex::x] + params_.base_to_rear * pose.dir.x();
    pose.pos.y = state_[StateIndex::y] + params_.base_to_rear * pose.dir.y();
    return pose;
}

geom::Angle SimulatorEngine::getSteering() const {
    return geom::Angle(state_[StateIndex::steering]);
}

double SimulatorEngine::getSpeed() const { return state_[StateIndex::linear_velocity]; }

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2(
        cos(state_[StateIndex::linear_velocity]), sin(state_[StateIndex::linear_velocity]));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2(
        cos(state_[StateIndex::angular_velocity]), sin(state_[StateIndex::angular_velocity]));
}

void SimulatorEngine::setControl(
    const double velocity, const double acceleration, const double curvature) {
    
    control_.velocity = model_->baseVelocityLimits().clamp(velocity);
    control_.acceleration = model_->baseAccelerationLimits().clamp(acceleration);
    const auto curvature_limit = model_->baseMaxAbsCurvature();
    control_.curvature = std::clamp(curvature, -curvature_limit, curvature_limit);
    const auto product = control_.curvature * params_.base_to_rear;
    control_.curvature /= sqrt(1 - product * product);
}

void SimulatorEngine::setControl(const double velocity, const double curvature) {
    const auto limits = model_->baseAccelerationLimits();
    const double acceleration =
        abs(velocity - state_[StateIndex::linear_velocity]) < params_.precision ? 0
        : state_[StateIndex::linear_velocity] < velocity                        ? limits.max
                                                                                : limits.min;
    setControl(velocity, acceleration, curvature);
}

SimulatorEngine::State SimulatorEngine::calculateStateDelta(
    const SimulatorEngine::State &state, const double acceleration,
    const double steering_velocity) {
    
    SimulatorEngine::State delta;
    delta[StateIndex::x] = cos(state[StateIndex::yaw]) * state[StateIndex::linear_velocity];
    delta[StateIndex::y] = sin(state[StateIndex::yaw]) * state[StateIndex::linear_velocity];
    delta[StateIndex::yaw] =
        tan(state[StateIndex::steering]) * state[StateIndex::linear_velocity] / params_.wheelbase;
    delta[StateIndex::steering] = steering_velocity;
    delta[StateIndex::linear_velocity] = acceleration;
    return delta;
}

void SimulatorEngine::advance(const double time) {
    const int integration_steps = time / params_.integration_step;

    const double old_yaw = state_[StateIndex::yaw];

    const double steering_final = abs(control_.curvature) < params_.precision
                                      ? 0
                                      : std::clamp(
                                            atan(params_.wheelbase * control_.curvature),
                                            -params_.steering_limit,
                                            +params_.steering_limit);

    double steering_rest = abs(steering_final - state_[StateIndex::steering]);
    double steering_velocity = steering_rest < params_.precision ? 0
                               : state_[StateIndex::steering] < steering_final
                                   ? params_.max_steering_velocity
                                   : -params_.max_steering_velocity;

    double velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]);
    double acceleration = velocity_rest < params_.precision ? 0 : control_.acceleration;

    for (auto i = 0; i < integration_steps; ++i) {
        steering_rest = abs(steering_final - state_[StateIndex::steering]) -
                        abs(steering_velocity) * params_.integration_step / 6;
        if (steering_rest < params_.precision) {
            state_[StateIndex::steering] = steering_final;
            steering_velocity = 0;
        }

        velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]) -
                        abs(acceleration) * params_.integration_step / 6;
        if (velocity_rest < params_.precision) {
            state_[StateIndex::linear_velocity] = control_.velocity;
            acceleration = 0;
        }

        auto k1 = calculateStateDelta(state_, acceleration, steering_velocity);
        auto k2 = calculateStateDelta(
            state_ + k1 * (params_.integration_step / 2), acceleration, steering_velocity);
        auto k3 = calculateStateDelta(
            state_ + k2 * (params_.integration_step / 2), acceleration, steering_velocity);
        auto k4 = calculateStateDelta(
            state_ + k3 * params_.integration_step, acceleration, steering_velocity);

        state_ += (k1 + 2 * k2 + 2 * k3 + k4) * (params_.integration_step / 6);
        state_[StateIndex::yaw] = geom::Angle::_0_2PI(state_[StateIndex::yaw]);
    }

    state_[StateIndex::angular_velocity] = (state_[StateIndex::yaw] - old_yaw) / time;
}

}  // namespace truck::simulator
