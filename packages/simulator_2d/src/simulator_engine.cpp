#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(const model::Model& model, 
    double integration_step, double precision): model_(model) {
    
    params_.integration_step = integration_step;
    params_.precision = precision;

    cache_.integration_step_2 = integration_step / 2;
    cache_.integration_step_6 = integration_step / 6;
    cache_.inverse_integration_step = 1 / integration_step;
    cache_.inverse_wheelbase_length = 1 / model_.wheelBase().length;
    cache_.wheelbase_width_2 = model_.wheelBase().width / 2;

    reset_rear();
}

void SimulatorEngine::reset_rear(double x, double y, double yaw,
    double steering, double linear_velocity) {

    rear_ax_state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity)
        .finished();
}

void SimulatorEngine::reset_rear() {
    reset_rear(-model_.wheelBase().base_to_rear, 0, 0, 0, 0);
}

void SimulatorEngine::reset_base(double x, double y, double yaw,
    double steering, double linear_velocity) {
    
    const auto dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    const double rear_x = x 
        - model_.wheelBase().base_to_rear * dir.x();
    const double rear_y = y
        - model_.wheelBase().base_to_rear * dir.y();

    const double base_curvature = std::tan(steering) * cache_.inverse_wheelbase_length;
    const auto base_twist = model::Twist {base_curvature, linear_velocity};
    const auto rear_twist = model_.baseToRearTwist(base_twist);
    const double rear_steering = model_.rearTwistToSteering(rear_twist).middle.radians();

    reset_rear(rear_x, rear_y, yaw, rear_steering, rear_twist.velocity);
}

const rclcpp::Time& SimulatorEngine::getTime() const {
    return time_;
}

geom::Pose SimulatorEngine::getBasePose() const {
    geom::Pose pose;
    pose.pos.x = rear_ax_state_[StateIndex::x] + model_.wheelBase().base_to_rear;
    return pose;
}

double SimulatorEngine::getCurrentRearCurvature() const {
    return tan(rear_ax_state_[StateIndex::steering]) * cache_.inverse_wheelbase_length;
}

model::Steering SimulatorEngine::getCurrentSteering() const {
    return model_.rearCurvatureToSteering(getCurrentRearCurvature());
}

model::Steering SimulatorEngine::getTargetSteering() const {
    return model_.rearCurvatureToSteering(control_.curvature);
}

model::Twist SimulatorEngine::getBaseTwist() const { 
    const auto twist = model::Twist {getCurrentRearCurvature(), 
        rear_ax_state_[StateIndex::linear_velocity]};
    return model_.rearToBaseTwist(twist);
}

geom::Vec2 SimulatorEngine::getBaseLinearVelocity() const {
    return geom::Vec2::fromAngle(geom::Angle(getBaseTwist().velocity));
}

geom::Vec2 SimulatorEngine::getBaseAngularVelocity() const {
    const double angular_velocity = getBaseTwist().velocity * getCurrentRearCurvature();
    return geom::Vec2::fromAngle(geom::Angle(angular_velocity));
}

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    velocity = model_.baseVelocityLimits().clamp(velocity);
    acceleration = model_.baseAccelerationLimits().clamp(acceleration);
    curvature = model_.baseCurvatureLimits().clamp(curvature);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_.baseToRearTwist(base_twist);

    control_.velocity = rear_twist.velocity;
    control_.acceleration 
        = model_.baseToRearAcceleration(acceleration, curvature);
    control_.curvature = rear_twist.curvature;
}

void SimulatorEngine::setBaseControl(double velocity, double curvature) {
    double acceleration;
    
    const auto limits = model_.baseAccelerationLimits();
    if (rear_ax_state_[StateIndex::linear_velocity] * velocity > 0) {
        const double current_speed 
            = abs(rear_ax_state_[StateIndex::linear_velocity] - params_.precision);
        if (current_speed < abs(velocity)) {
            acceleration = limits.max;
        } else {
            acceleration = limits.min;
        }

        if (velocity < 0) {
            acceleration *= -1;
        }
    }
    else {
        acceleration = limits.min;

        if (velocity > 0) {
            acceleration *= -1;
        }
    }

    setBaseControl(velocity, acceleration, curvature);
}

SimulatorEngine::State SimulatorEngine::calculateStateDerivative(
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
        const double new_velocity = rear_ax_state_[StateIndex::linear_velocity] + velocity_delta;
        if (isOutOfRange(new_velocity, control_.velocity, params_.precision)) {
            rear_ax_state_[StateIndex::linear_velocity] = control_.velocity;
            acceleration = 0;
        }

        auto k1 = calculateStateDerivative(rear_ax_state_, acceleration);
        auto k2 = calculateStateDerivative(
            rear_ax_state_ + k1 * cache_.integration_step_2, acceleration);
        auto k3 = calculateStateDerivative(
            rear_ax_state_ + k2 * cache_.integration_step_2, acceleration);
        auto k4 = calculateStateDerivative(
            rear_ax_state_ + k3 * params_.integration_step, acceleration);

        rear_ax_state_ += (k1 + 2 * k2 + 2 * k3 + k4) * cache_.integration_step_6;
    }
}

}  // namespace truck::simulator
