#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(const std::string& model_config_path, 
    double integration_step, double precision): model_(model_config_path) {
    
    params_.integration_step = integration_step;
    params_.precision = precision;

    cache_.integration_step_2 = integration_step / 2;
    cache_.integration_step_6 = integration_step / 6;
    cache_.inverse_integration_step = 1 / integration_step;
    cache_.inverse_wheelbase_length = 1 / model_.wheelBase().length;
    cache_.wheelbase_width_2 = model_.wheelBase().width / 2;

    reset();
}

void SimulatorEngine::reset(double x, double y, double yaw, double steering, 
        double linear_velocity, double angular_velocity) {
    state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity, angular_velocity)
        .finished();
}

void SimulatorEngine::reset() {
    reset(-model_.wheelBase().base_to_rear, 0, 0, 0, 0, 0);
}

rclcpp::Time SimulatorEngine::getTime() const {
    return time_;
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    pose.pos.x = state_[StateIndex::x] + model_.wheelBase().base_to_rear;
    return pose;
}

double SimulatorEngine::getCurrentCurvature() const {
    return model_.wheelBase().length * tan(state_[StateIndex::steering]);
}

double SimulatorEngine::getMiddleSteering() const {
    return state_[StateIndex::steering];
}

model::Steering SimulatorEngine::getCurrentSteering() const {
    return model_.rearCurvatureToSteering(getCurrentCurvature());
}

model::Steering SimulatorEngine::getTargetSteering() const {
    return model_.rearCurvatureToSteering(control_.curvature);
}

model::Twist SimulatorEngine::getTwist() const { 
    const auto twist = model::Twist {getCurrentCurvature(), state_[StateIndex::linear_velocity]};
    return model_.rearToBaseTwist(twist);
}

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2::fromAngle(geom::Angle(getTwist().velocity));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2::fromAngle(geom::Angle(state_[StateIndex::angular_velocity]));
}

void SimulatorEngine::setControl(
    double velocity, double acceleration, double curvature) {

    control_.velocity 
        = model_.baseToLimitedRearVelocity(velocity, curvature);
    control_.acceleration 
        = model_.baseToLimitedRearAcceleration(acceleration, curvature);
    control_.curvature 
        = model_.baseToLimitedRearCurvature(curvature);
}

void SimulatorEngine::setControl(double velocity, double curvature) {
    control_.velocity 
        = model_.baseToLimitedRearVelocity(velocity, curvature);
    control_.curvature 
        = model_.baseToLimitedRearCurvature(curvature);

    const auto limits = model_.baseAccelerationLimits();
    if (state_[StateIndex::linear_velocity] - params_.precision < control_.velocity) {
        control_.acceleration = limits.max;
    } else {
        control_.acceleration = limits.min;
    }
}

SimulatorEngine::State SimulatorEngine::calculateStateDelta(
    const SimulatorEngine::State &state, double acceleration) {
    
    SimulatorEngine::State delta;
    delta[StateIndex::x] = state[StateIndex::linear_velocity];
    delta[StateIndex::linear_velocity] = acceleration;
    return delta;
}

namespace {

rclcpp::Duration convertFromSecondsToDuration(double seconds) {
    auto int_seconds = int(seconds);
    auto nanoseconds = (seconds - int_seconds) * 1e9;
    return rclcpp::Duration(int_seconds, int(nanoseconds));
}

bool isOutOfRange(double value, double limit, double precision) {
    return (limit > 0 && value - precision > limit)
        || (limit < 0 && value + precision < limit);
}

} // namespace

void SimulatorEngine::advance(double time) {
    time_ += convertFromSecondsToDuration(time);

    const int integration_steps = time * cache_.inverse_integration_step;

    double acceleration = control_.acceleration;
    // Acceleration is actually constant, so delta is a constant.
    const double velocity_delta = acceleration * params_.integration_step;

    for (auto i = 0; i < integration_steps; ++i) {
        const double new_velocity = state_[StateIndex::linear_velocity] + velocity_delta;
        if (isOutOfRange(new_velocity, control_.velocity, params_.precision)) {
            state_[StateIndex::linear_velocity] = control_.velocity;
            acceleration = 0;
        }

        auto k1 = calculateStateDelta(state_, acceleration);
        auto k2 = calculateStateDelta(
            state_ + k1 * cache_.integration_step_2, acceleration);
        auto k3 = calculateStateDelta(
            state_ + k2 * cache_.integration_step_2, acceleration);
        auto k4 = calculateStateDelta(
            state_ + k3 * params_.integration_step, acceleration);

        state_ += (k1 + 2 * k2 + 2 * k3 + k4) * cache_.integration_step_6;
    }
}

}  // namespace truck::simulator
