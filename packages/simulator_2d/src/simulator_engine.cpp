#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(std::unique_ptr<model::Model> model,
    double integration_step, double precision) {
        
    model_ = std::move(model);

    params_.integration_step = integration_step;
    params_.precision = precision;

    cache_.integration_step_2 = integration_step / 2;
    cache_.integration_step_6 = integration_step / 6;
    cache_.inverse_integration_step = 1 / integration_step;
    cache_.inverse_wheelbase_length = 1 / model_->wheelBase().length;

    reset_rear();
}

void SimulatorEngine::reset_rear(double x, double y, double yaw,
    double steering, double linear_velocity) {

    rear_ax_state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity)
        .finished();
}

void SimulatorEngine::reset_rear() {
    reset_rear(-model_->wheelBase().base_to_rear, 0, 0, 0, 0);
}

void SimulatorEngine::reset_base(const geom::Pose& pose,
    double middle_steering, double linear_velocity) {
    
    const auto [rear_x, rear_y] = geom::Vec2{pose.pos.x, pose.pos.y} 
        - model_->wheelBase().base_to_rear * pose.dir;

    const double base_curvature = std::tan(middle_steering) * cache_.inverse_wheelbase_length;
    const auto base_twist = model::Twist {base_curvature, linear_velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    reset_rear(rear_x, rear_y, yaw, middle_steering, rear_twist.velocity);
}

std::unique_ptr<TruckState> SimulatorEngine::getBaseTruckState() const {
    return TruckState::fromRearToBaseState(*model_.get(), rear_ax_state_[StateIndex::x],
        rear_ax_state_[StateIndex::y], rear_ax_state_[StateIndex::yaw], time_,
        rear_ax_state_[StateIndex::steering], rear_ax_state_[StateIndex::linear_velocity],
        control_.curvature);
}

namespace {

bool isOutOfRange(double value, double limit, double precision) {
    return (limit >= 0 && value + precision >= limit)
        || (limit < 0 && value - precision <= limit);
}

} // namespace

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    VERIFY(acceleration >= 0);

    curvature = model_->baseCurvatureLimits().clamp(curvature);
    velocity = model_->baseVelocityLimits().clamp(velocity);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    const bool is_speed_up = isOutOfRange(rear_twist.velocity, 
        rear_ax_state_[StateIndex::linear_velocity], params_.precision);
    if (is_speed_up) {
        acceleration = std::max(acceleration, model_->baseMaxAcceleration());
    }
    else {
        acceleration = std::max(acceleration, model_->baseMaxDeceleration());
    }

    control_.curvature = rear_twist.curvature;
    control_.velocity = rear_twist.velocity;
    control_.acceleration 
        = model_->baseToRearAcceleration(acceleration, curvature);
}

void SimulatorEngine::setBaseControl(double velocity, double curvature) {
    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);
    const bool is_speed_up = isOutOfRange(rear_twist.velocity,
        rear_ax_state_[StateIndex::linear_velocity], params_.precision);

    double acceleration = is_speed_up
        ? model_->baseMaxAcceleration()
        : model_->baseMaxDeceleration();

    setBaseControl(velocity, acceleration, curvature);
}

namespace {

int softSign(double number, double precision) {
    if (number > precision) {
        return 1;
    }
    
    if (number < -precision) {
        return -1;
    }

    return 0;
}

double getOptionalValue(const std::optional<double>& opt, double max) {
    if (!opt) {
        return max;
    }

    return std::min(*opt, max);
}

} // namespace

double SimulatorEngine::getCurrentAcceleration() {
    double target_velocity = control_.velocity;
    const bool need_stop =
        (softSign(control_.velocity, params_.precision) *
         softSign(rear_ax_state_[StateIndex::linear_velocity], params_.precision)) < 0;
    if (need_stop) {
        target_velocity = 0;
    }

    const auto action_sign = softSign(abs(target_velocity) 
        - abs(rear_ax_state_[StateIndex::linear_velocity]), params_.precision);
    const auto acceleration_sign = softSign(target_velocity 
        - rear_ax_state_[StateIndex::linear_velocity], params_.precision);

    double current_acceleration = 0;
    if (action_sign == 1) {
        // Acceleration.
        current_acceleration = acceleration_sign 
            * getOptionalValue(control_.acceleration, model_->baseMaxAcceleration());
    } else if (action_sign == -1) {
        // Deceleration.
        current_acceleration = acceleration_sign
            * getOptionalValue(control_.acceleration, model_->baseMaxDeceleration());
    }

    const double velocity_delta = current_acceleration * cache_.inverse_integration_step;
    const double new_velocity = rear_ax_state_[StateIndex::linear_velocity] + velocity_delta;
    const bool target_speed_achieved = control_.acceleration > 0
        ? new_velocity + params_.precision > target_velocity
        : new_velocity - params_.precision < target_velocity;
    if (target_speed_achieved) {
        current_acceleration = (target_velocity 
            - rear_ax_state_[StateIndex::linear_velocity]) * params_.integration_step;
    }

    return current_acceleration;
}

SimulatorEngine::State SimulatorEngine::calculateStateDerivative(
    const SimulatorEngine::State &state, double acceleration) {
    
    SimulatorEngine::State deriv;
    deriv.setZero();
    deriv[StateIndex::x] = state[StateIndex::linear_velocity];
    deriv[StateIndex::linear_velocity] = acceleration;
    return deriv;
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

    for (int i = 0; i < integration_steps; ++i) {
        const double current_acceleration = getCurrentAcceleration();



        rear_ax_state_ += calculateRK4(current_acceleration);
    }
}

}  // namespace truck::simulator
