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

namespace {

bool isOutOfRange(double value, double limit, double precision) {
    return (limit >= 0 && value + precision >= limit)
        || (limit < 0 && value - precision <= limit);
}

} // namespace

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    curvature = model_.baseCurvatureLimits().clamp(curvature);
    velocity = model_.baseVelocityLimits().clamp(velocity);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_.baseToRearTwist(base_twist);

    const bool is_acceleration_incorrect 
        = (rear_twist.velocity < control_.velocity && acceleration > 0)
        || (rear_twist.velocity > control_.velocity && acceleration < 0);
    if (is_acceleration_incorrect) {
        acceleration *= -1;
    }

    const bool is_speed_up = isOutOfRange(rear_twist.velocity, 
        rear_ax_state_[StateIndex::linear_velocity], params_.precision);
    if (is_speed_up) {
        acceleration = model_.baseSpeedUpLimits().clamp(acceleration);
    }
    else {
        acceleration = model_.baseBrakingLimits().clamp(acceleration);
    }

    control_.curvature = rear_twist.curvature;
    control_.velocity = rear_twist.velocity;
    control_.acceleration 
        = model_.baseToRearAcceleration(acceleration, curvature);
}

void SimulatorEngine::setBaseControl(double velocity, double curvature) {
    double acceleration;

    const bool is_speed_up = isOutOfRange(velocity,
        rear_ax_state_[StateIndex::linear_velocity], params_.precision);
    if (is_speed_up) {
        acceleration = model_.baseAccelerationLimits().max;
    }
    else {
        acceleration = model_.baseAccelerationLimits().min;
    }

    if (velocity < 0) {
        acceleration *= -1;
    }

    setBaseControl(velocity, acceleration, curvature);
}

SimulatorEngine::State SimulatorEngine::calculateStateDerivative(
    const SimulatorEngine::State &state, double acceleration) {
    
    SimulatorEngine::State deriv;
    deriv[StateIndex::x] = state[StateIndex::linear_velocity];
    deriv[StateIndex::linear_velocity] = acceleration;
    return deriv;
}

void SimulatorEngine::validateAcceleration(double& acceleration, double& target_velocity) {
    const double velocity_delta = acceleration * params_.integration_step;
    const double new_velocity 
        = rear_ax_state_[StateIndex::linear_velocity] + velocity_delta;
    const bool is_speed_up = isOutOfRange(control_.velocity, 
        rear_ax_state_[StateIndex::linear_velocity], params_.precision);
    const bool target_speed_achieved = is_speed_up
        ^ isOutOfRange(new_velocity, target_velocity, params_.precision);
    if (target_speed_achieved) {
        rear_ax_state_[StateIndex::linear_velocity] = target_velocity;
        if ((target_velocity - control_.velocity) > params_.precision) {
            target_velocity = control_.velocity;
            acceleration = model_.baseAccelerationLimits().max;
            if (control_.velocity < 0) {
                acceleration *= -1;
            }
        }
        else {
            acceleration = 0;
        }
    }
}

SimulatorEngine::State SimulatorEngine::calculateRK4(double acceleration) {
    const auto k1 = calculateStateDerivative(rear_ax_state_, acceleration);
    const auto k2 = calculateStateDerivative(
        rear_ax_state_ + k1 * cache_.integration_step_2, acceleration);
    const auto k3 = calculateStateDerivative(
        rear_ax_state_ + k2 * cache_.integration_step_2, acceleration);
    const auto k4 = calculateStateDerivative(
        rear_ax_state_ + k3 * params_.integration_step, acceleration);

    return (k1 + 2 * k2 + 2 * k3 + k4) * cache_.integration_step_6;
}

namespace {

rclcpp::Duration convertFromSecondsToDuration(double seconds) {
    auto int_seconds = int(seconds);
    auto nanoseconds = (seconds - int_seconds) * 1e9;
    return rclcpp::Duration(int_seconds, int(nanoseconds));
}

} // namespace

void SimulatorEngine::advance(double seconds) {
    time_ += convertFromSecondsToDuration(seconds);

    const int integration_steps = seconds * cache_.inverse_integration_step;
    
    double acceleration = control_.acceleration;
    double target_velocity
        = rear_ax_state_[StateIndex::linear_velocity] * control_.velocity < 0
            ? 0
            : control_.velocity;

    for (int i = 0; i < integration_steps; ++i) {
        validateAcceleration(acceleration, target_velocity);
        rear_ax_state_ += calculateRK4(acceleration);
    }
}

}  // namespace truck::simulator
