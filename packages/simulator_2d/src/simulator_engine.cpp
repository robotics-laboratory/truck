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

    resetRear();
}

void SimulatorEngine::resetRear(double x, double y, double yaw,
    double steering, double linear_velocity) {

    rear_ax_state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity)
        .finished();
}

void SimulatorEngine::resetRear() {
    resetRear(-model_->wheelBase().base_to_rear, 0, 0, 0, 0);
}

void SimulatorEngine::resetBase(const geom::Pose& pose,
    double middle_steering, double linear_velocity) {
    
    const auto [rear_x, rear_y] = geom::Vec2{pose.pos.x, pose.pos.y} 
        - model_->wheelBase().base_to_rear * pose.dir;

    const double base_curvature = std::tan(middle_steering) * cache_.inverse_wheelbase_length;
    const auto base_twist = model::Twist {base_curvature, linear_velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    resetRear(rear_x, rear_y, yaw, middle_steering, rear_twist.velocity);
}

geom::Pose SimulatorEngine::getOdomBasePose() const {
    const double x = rear_ax_state_[StateIndex::x];
    const double y = rear_ax_state_[StateIndex::y];
    const double yaw = rear_ax_state_[StateIndex::yaw];

    geom::Pose pose;
    pose.dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    pose.pos = geom::Vec2{x, y} + model_->wheelBase().base_to_rear * pose.dir;
    return pose;
}

model::Steering SimulatorEngine::getCurrentSteering(double rear_curvature) const {
    return model_->rearCurvatureToSteering(rear_curvature);
}

model::Steering SimulatorEngine::getTargetSteering() const {
    return model_->rearCurvatureToSteering(control_.curvature);
}

model::Twist SimulatorEngine::rearToOdomBaseTwist(double rear_curvature) const {
    const double linear_velocity = rear_ax_state_[StateIndex::linear_velocity];
    const auto twist = model::Twist {
        rear_curvature,
        linear_velocity
    };

    return model_->rearToBaseTwist(twist);
}

geom::Vec2 SimulatorEngine::rearToOdomBaseLinearVelocity(
    truck::geom::AngleVec2 dir, double base_velocity) const {

    return dir * base_velocity;
}

double SimulatorEngine::rearToOdomBaseAngularVelocity(
    double base_velocity, double rear_curvature) const {

    return base_velocity * rear_curvature;
}

TruckState SimulatorEngine::getTruckState() const {
    const double steering = rear_ax_state_[StateIndex::steering];

    const auto pose = getOdomBasePose();
    const double rear_curvature = model_->middleSteeringToRearCurvature(steering);
    const auto current_steering = getCurrentSteering(rear_curvature);
    const auto target_steering = getTargetSteering();
    const auto twist = rearToOdomBaseTwist(rear_curvature);
    const auto linear_velocity = rearToOdomBaseLinearVelocity(pose.dir, twist.velocity);
    const auto angular_velocity = rearToOdomBaseAngularVelocity(twist.velocity, rear_curvature);
    
    return TruckState()
        .setTime(time_)
        .setBaseOdomPose(pose)
        .setCurrentSteering(current_steering)
        .setTargetSteering(target_steering)
        .setBaseOdomTwist(twist)
        .setBaseOdomLinearVelocity(linear_velocity)
        .setBaseOdomAngularVelocity(angular_velocity);
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

} // namespace

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    VERIFY(acceleration >= 0);

    curvature = model_->baseCurvatureLimits().clamp(curvature);
    velocity = model_->baseVelocityLimits().clamp(velocity);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    const int action_sign = softSign(abs(rear_twist.velocity) 
        - abs(rear_ax_state_[StateIndex::linear_velocity]), params_.precision);
    if (action_sign == 1) {
        acceleration = std::max(acceleration, model_->baseMaxAcceleration());
    }
    else if (action_sign == -1) {
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

    const int action_sign = softSign(abs(rear_twist.velocity) 
        - abs(rear_ax_state_[StateIndex::linear_velocity]), params_.precision);
    double acceleration = 0;
    if (action_sign == 1) {
        acceleration = model_->baseMaxAcceleration();
    } else if (action_sign == -1) {
        acceleration = model_->baseMaxDeceleration();
    }

    setBaseControl(velocity, acceleration, curvature);
}

namespace {

double getOptionalValue(const std::optional<double>& opt, double max) {
    if (!opt) {
        return max;
    }

    return std::min(*opt, max);
}

} // namespace

double SimulatorEngine::getCurrentAcceleration() const {
    const double velocity = rear_ax_state_[StateIndex::linear_velocity];

    double target_velocity = control_.velocity;
    const bool need_stop = (softSign(control_.velocity, params_.precision)
        * softSign(velocity, params_.precision)) < 0;
    if (need_stop) {
        target_velocity = 0;
    }

    const int action_sign = softSign(abs(target_velocity) 
        - abs(velocity), params_.precision);
    const int acceleration_sign = softSign(target_velocity 
        - velocity, params_.precision);

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

    const double velocity_delta = current_acceleration * params_.integration_step;
    const double new_velocity = velocity + velocity_delta;
    const bool target_velocity_achieved 
        = (acceleration_sign > 0 && (new_velocity + params_.precision > target_velocity))
        || (acceleration_sign < 0 && (new_velocity - params_.precision < target_velocity));

    if (target_velocity_achieved) {
        current_acceleration = (target_velocity 
            - velocity) * cache_.inverse_integration_step;
    }

    return current_acceleration;
}

double SimulatorEngine::getCurrentSteeringVelocity() const {
    const double steering = rear_ax_state_[StateIndex::steering];
    const double target_steering = getTargetSteering().middle.radians();
    const int velocity_sign = softSign(target_steering - steering, params_.precision);
    double current_velocity = velocity_sign * model_->steeringVelocity();

    const double steering_delta = current_velocity * params_.integration_step;
    const double new_steering = steering + steering_delta;
    const bool target_steering_achieved
        = (velocity_sign > 0 && (new_steering + params_.precision > target_steering))
        || (velocity_sign < 0 && (new_steering - params_.precision < target_steering));

    if (target_steering_achieved) {
        current_velocity = (target_steering 
            - steering) * cache_.inverse_integration_step;
    }

    return current_velocity;
}

SimulatorEngine::State SimulatorEngine::calculateStateDerivative(
    const SimulatorEngine::State &state, double acceleration, double steering_velocity) const {
    
    const double yaw = state[StateIndex::yaw];
    const double velocity = state[StateIndex::linear_velocity];
    const double steering = state[StateIndex::steering];

    SimulatorEngine::State deriv;
    deriv.setZero();
    deriv[StateIndex::x] = cos(yaw) * velocity;
    deriv[StateIndex::y] = sin(yaw) * velocity;
    deriv[StateIndex::yaw] = tan(steering) * velocity * cache_.inverse_wheelbase_length;
    deriv[StateIndex::linear_velocity] = acceleration;
    deriv[StateIndex::steering] = steering_velocity;

    return deriv;
}

SimulatorEngine::State SimulatorEngine::calculateRK4(
    double acceleration, double steering_velocity) const {

    const auto k1 = calculateStateDerivative(rear_ax_state_, acceleration, steering_velocity);
    const auto k2 = calculateStateDerivative(
        rear_ax_state_ + k1 * cache_.integration_step_2, acceleration, steering_velocity);
    const auto k3 = calculateStateDerivative(
        rear_ax_state_ + k2 * cache_.integration_step_2, acceleration, steering_velocity);
    const auto k4 = calculateStateDerivative(
        rear_ax_state_ + k3 * params_.integration_step, acceleration, steering_velocity);

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
        const double current_steering_velocity = getCurrentSteeringVelocity();
        rear_ax_state_ += calculateRK4(current_acceleration, current_steering_velocity);
    }
}

}  // namespace truck::simulator
