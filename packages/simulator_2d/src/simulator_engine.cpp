#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(const std::string& model_config_path, 
    double integration_step, double precision): model_(model_config_path) {
    
    params_.integration_step = integration_step;
    params_.integration_step_2 = integration_step / 2;
    params_.integration_step_6 = integration_step / 6;
    params_.inverse_integration_step = 1 / integration_step;
    params_.precision = precision;
    params_.inverse_wheelbase_length = 1 / model_.wheelBase().length;
    params_.wheelbase_width_2 = model_.wheelBase().width / 2;
    reset();
}

void SimulatorEngine::reset(const simulator::State &state) {
    state_ = state;
}

void SimulatorEngine::reset() {
    state_ = simulator::State::Zero();
    state_[StateIndex::x] = -model_.wheelBase().base_to_rear;
}

rclcpp::Time SimulatorEngine::getTime() const {
    return time_;
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    pose.dir = geom::AngleVec2(geom::Angle::fromRadians(state_[StateIndex::yaw]));
    pose.pos.x = state_[StateIndex::x] + model_.wheelBase().base_to_rear * pose.dir.x();
    pose.pos.y = state_[StateIndex::y] + model_.wheelBase().base_to_rear * pose.dir.y();
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
    return model_.baseToRearTwist(twist);
}

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2::fromAngle(geom::Angle(getTwist().velocity));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2::fromAngle(geom::Angle(state_[StateIndex::angular_velocity]));
}

void SimulatorEngine::setControl(
    double velocity, double acceleration, double curvature) {
    
    control_.velocity = model_.baseVelocityLimits().clamp(velocity);
    control_.acceleration = model_.baseAccelerationLimits().clamp(acceleration);
    const auto curvature_limit = model_.baseMaxAbsCurvature();
    control_.curvature = std::clamp(curvature, -curvature_limit, curvature_limit);
    const auto product = control_.curvature * model_.wheelBase().base_to_rear;
    control_.curvature /= sqrt(1 - product * product);
}

void SimulatorEngine::setControl(double velocity, double curvature) {
    const auto limits = model_.baseAccelerationLimits();
    const double velocity_rest = abs(velocity - state_[StateIndex::linear_velocity]);
    double acceleration;
    if (velocity_rest < params_.precision) {
        acceleration = 0;
    } else if (state_[StateIndex::linear_velocity] < velocity) {
        acceleration = limits.max;
    } else {
        acceleration = limits.min;
    }

    setControl(velocity, acceleration, curvature);
}

simulator::State SimulatorEngine::calculateStateDelta(
    const simulator::State &state, double acceleration,
    double steering_velocity) {
    
    simulator::State delta;
    delta[StateIndex::x] = cos(state[StateIndex::yaw]) * state[StateIndex::linear_velocity];
    delta[StateIndex::y] = sin(state[StateIndex::yaw]) * state[StateIndex::linear_velocity];
    delta[StateIndex::yaw] =
        tan(state[StateIndex::steering]) * state[StateIndex::linear_velocity] 
            * params_.inverse_wheelbase_length;
    delta[StateIndex::steering] = steering_velocity;
    delta[StateIndex::linear_velocity] = acceleration;
    return delta;
}

namespace {

rclcpp::Duration convertFromSecondsToDuration(double seconds) {
    auto int_seconds = int(seconds);
    auto nanoseconds = (seconds - int_seconds) * 1e9;
    return rclcpp::Duration(int_seconds, int(nanoseconds));
}

} // namespace

void SimulatorEngine::advance(double time) {
    time_ += convertFromSecondsToDuration(time);

    const int integration_steps = time * params_.inverse_integration_step;

    const double old_yaw = state_[StateIndex::yaw];

    const double target_steering 
        = std::clamp(atan2(control_.curvature, params_.inverse_wheelbase_length), 
                                -model_.leftSteeringLimits().max.radians(), 
                                model_.leftSteeringLimits().max.radians());
    double steering_velocity;
    if (state_[StateIndex::steering] - params_.precision < target_steering) {
        steering_velocity = model_.steeringVelocity();
    } else {
        steering_velocity = -model_.steeringVelocity();
    }

    double velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]);
    double acceleration = velocity_rest < params_.precision ? 0 : control_.acceleration;

    for (auto i = 0; i < integration_steps; ++i) {
        const double steering_rest = abs(target_steering - state_[StateIndex::steering]);
        const double steering_delta = abs(steering_velocity) * params_.integration_step_6;
        if (steering_rest - steering_delta < params_.precision) {
            state_[StateIndex::steering] = target_steering;
            steering_velocity = 0;
        }

        velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]);
        const double velocity_delta = abs(acceleration) * params_.integration_step_6;
        if (velocity_rest - velocity_delta < params_.precision) {
            state_[StateIndex::linear_velocity] = control_.velocity;
            acceleration = 0;
        }

        auto k1 = calculateStateDelta(state_, acceleration, steering_velocity);
        auto k2 = calculateStateDelta(
            state_ + k1 * params_.integration_step_2, acceleration, steering_velocity);
        auto k3 = calculateStateDelta(
            state_ + k2 * params_.integration_step_2, acceleration, steering_velocity);
        auto k4 = calculateStateDelta(
            state_ + k3 * params_.integration_step, acceleration, steering_velocity);

        state_ += (k1 + 2 * k2 + 2 * k3 + k4) * params_.integration_step_6;
        state_[StateIndex::yaw] = geom::Angle::_0_2PI(state_[StateIndex::yaw]);
    }

    state_[StateIndex::angular_velocity] = (state_[StateIndex::yaw] - old_yaw) / time;
}

}  // namespace truck::simulator
