#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::~SimulatorEngine() {
    isRunning_ = false;
    running_thread_.join();
}

void SimulatorEngine::start(std::unique_ptr<model::Model> &model, const double simulation_tick, 
    const int integration_steps, const double precision) {
    
    isRunning_ = isResumed_ = false;
    if (running_thread_.joinable()) {
        running_thread_.join();
    }

    model_ = std::unique_ptr<model::Model>(std::move(model));
    params_.simulation_tick = simulation_tick;
    params_.integration_steps = integration_steps;
    params_.precision = precision;
    params_.steering_velocity = model_->steeringVelocity();
    params_.wheelbase = model_->wheelBase().length;
    params_.base_to_rear = model_->wheelBase().base_to_rear;
    params_.steering_limit = model_->leftSteeringLimits().max.radians();
    state_.x = -params_.base_to_rear;
    isRunning_ = isResumed_ = true;
    last_update_= std::chrono::system_clock::now();
    running_thread_ = std::thread(&SimulatorEngine::processSimulation, this);
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

void SimulatorEngine::suspend() {
    isResumed_ = false;
}

void SimulatorEngine::resume() {
    isResumed_ = true;
}

void SimulatorEngine::calculate_state_delta(const SimulationState &state,
    const double velocity_delta, const double &steering_delta, SimulationState &delta) {
    
    delta.x = cos(state.rotation) * state.linear_velocity;
    delta.y = sin(state.rotation) * state.linear_velocity;
    delta.rotation = tan(state.steering) * state.linear_velocity / params_.wheelbase;
    delta.steering = steering_delta;
    delta.linear_velocity = velocity_delta;
}

void SimulatorEngine::updateState() {
    const auto now = std::chrono::system_clock::now();
    const double time = (now - last_update_).count() / 1e9;
    last_update_ = now;
    const double integration_step = time / params_.integration_steps;

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
    //steering_delta = steering_delta * time / params_.integration_steps;

    double velocity_delta = abs(control_.velocity - state_.linear_velocity) < params_.precision 
        ? 0 
        : control_.acceleration;
    //velocity_delta = velocity_delta * time / params_.integration_steps;

    SimulationState k[4];
    for (auto i = 0; i < params_.integration_steps; ++i) {
        if (abs(steering_final - state_.steering) - params_.precision < abs(steering_delta)) {
            steering_delta = steering_final - state_.steering;
        }

        if (abs(control_.velocity - state_.linear_velocity) - params_.precision < abs(velocity_delta)) {
            velocity_delta = control_.velocity - state_.linear_velocity;
        }
        
        calculate_state_delta(state_, velocity_delta, steering_delta, k[0]);
        calculate_state_delta(state_ + k[0] * (integration_step / 2),
            velocity_delta, steering_delta, k[1]);
        calculate_state_delta(state_ + k[1] * (integration_step / 2),
            velocity_delta, steering_delta, k[2]);
        calculate_state_delta(state_ + k[2] * integration_step,
            velocity_delta, steering_delta, k[3]);
        k[1] *= 2;
        k[2] *= 2;
        SimulationState::addSum(state_, k, 4, integration_step / 6);

        state_.rotation = fmod(state_.rotation, 2 * M_PI);
        if (state_.rotation < 0) {
            state_.rotation += 2 * M_PI;
        }
    }

    state_.angular_velocity = (state_.rotation - old_rotation) / time;
}

void SimulatorEngine::processSimulation() {
    auto wait_time = std::chrono::duration<double>(params_.simulation_tick);
    while (isRunning_) {
        if (isResumed_) {
            updateState();
        }

        std::this_thread::sleep_for(wait_time);
    }
}

} // namespace truck::simulator