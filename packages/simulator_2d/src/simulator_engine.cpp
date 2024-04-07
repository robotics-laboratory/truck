#include "simulator_2d/simulator_engine.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace truck::simulator {

// m/s^2
const double FREE_FALL_ACCELERATION = 9.81;

SimulatorEngine::SimulatorEngine(std::unique_ptr<model::Model> model,
    double integration_step, double precision) {
        
    model_ = std::move(model);
    initializeParameters(integration_step, precision);
    initializeMathCache(integration_step);
    initializeImuCache();
    initializeMap();
    resetRear();
}

void SimulatorEngine::initializeParameters(double integration_step, double precision) {
    params_.integration_step = integration_step;
    params_.precision = precision;
}

void SimulatorEngine::initializeMathCache(double integration_step) {
    cache_.integration_step_2 = integration_step / 2;
    cache_.integration_step_6 = integration_step / 6;
    cache_.inverse_integration_step = 1 / integration_step;
    cache_.inverse_wheelbase_length = 1 / model_->wheelBase().length;
    cache_.shape_length = model_->shape().length;
    cache_.shape_width_half = model_->shape().width / 2;
}

void SimulatorEngine::initializeImuCache() {
    const auto imu_tf = model_->getLatestTranform("base", "camera_imu_optical_frame");

    cache_.rear_to_hyro_translation.x = imu_tf.getOrigin().x() - model_->wheelBase().base_to_rear;
    cache_.rear_to_hyro_translation.y = imu_tf.getOrigin().y();
    cache_.base_to_hyro_rotation.setRotation(imu_tf.getRotation());
}

void SimulatorEngine::initializeMap() {
    const auto lidar_translation = model_->getLatestTranform("base", "lidar_link").getOrigin();
    const geom::Vec2 base_to_lidar(lidar_translation.x(), lidar_translation.y());
    map_ = std::make_unique<SimulatorMap>(params_.precision, base_to_lidar, model_->lidar());
}

void SimulatorEngine::resetRear(double x, double y, double yaw,
    double steering, double linear_velocity) {

    rear_ax_state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity)
        .finished();
    checkForCollisions();
}

void SimulatorEngine::resetRear() {
    resetRear(-model_->wheelBase().base_to_rear, 0, 0, 0, 0);
}

void SimulatorEngine::resetBase(const geom::Pose& pose,
    double middle_steering, double linear_velocity) {
    
    const auto [rear_x, rear_y] = geom::Vec2{pose.pos.x, pose.pos.y} 
        - model_->wheelBase().base_to_rear * pose.dir;

    const double yaw = pose.dir.angle().radians();

    const double base_curvature = std::tan(middle_steering) * cache_.inverse_wheelbase_length;
    const auto base_twist = model::Twist {base_curvature, linear_velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    resetRear(rear_x, rear_y, yaw, middle_steering, rear_twist.velocity);
}

void SimulatorEngine::resetMap(const std::string& path) {
    map_->resetMap(path);
    checkForCollisions();
}

void SimulatorEngine::eraseMap() {
    map_->eraseMap();
}

void SimulatorEngine::checkForCollisions() {
    const double x = rear_ax_state_[StateIndex::kX];
    const double y = rear_ax_state_[StateIndex::kY];
    const double yaw = rear_ax_state_[StateIndex::kYaw];

    fail_ |= map_->checkForCollisions({x, y}, cache_.shape_length, cache_.shape_width_half, yaw);
}

geom::Pose SimulatorEngine::getOdomBasePose() const {
    const double x = rear_ax_state_[StateIndex::kX];
    const double y = rear_ax_state_[StateIndex::kY];
    const double yaw = rear_ax_state_[StateIndex::kYaw];

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

model::Twist SimulatorEngine::getRearTwist(double rear_curvature) const {
    const double linear_velocity = rear_ax_state_[StateIndex::kLinearVelocity];
    return model::Twist {
        rear_curvature,
        linear_velocity
    };
}

namespace {

geom::Vec3 applyRotation(const tf2::Transform& rotation, const geom::Vec3& vector) {
    tf2::Vector3 v(vector.x, vector.y, vector.z);
    v = rotation * v;
    return geom::Vec3(v.getX(), v.getY(), v.getZ());
}

} // namespace

geom::Vec3 SimulatorEngine::getImuAngularVelocity(double angular_velocity) const {
    const geom::Vec3 w(0, 0, angular_velocity);
    return applyRotation(cache_.base_to_hyro_rotation, w);
}

namespace {

geom::Vec3 getLinearAcceleration(const model::Twist& twist, double acceleration, double yaw) {
    const auto yaw_vec = geom::Vec2::fromAngle(geom::Angle::fromRadians(yaw));
    const auto normal_a = squared(twist.velocity) * twist.curvature;
    const auto tan_a = std::sqrt(squared(acceleration) - squared(normal_a));
    return geom::Vec3(
        tan_a * yaw_vec + yaw_vec.left() * normal_a,
        FREE_FALL_ACCELERATION
    );
}

} // namespace

geom::Vec3 SimulatorEngine::getImuLinearAcceleration(const model::Twist& rear_twist) const {
    const auto twist = model_->rearToArbitraryPointTwist(rear_twist, cache_.rear_to_hyro_translation);
    const auto acceleration = getCurrentAcceleration();
    const auto yaw = rear_ax_state_[StateIndex::kYaw];

    const auto la = getLinearAcceleration(twist, acceleration, yaw);

    return applyRotation(cache_.base_to_hyro_rotation, la);
}

namespace {

geom::Vec2 rearToOdomBaseLinearVelocity(truck::geom::AngleVec2 dir, double base_velocity) {
    return dir * base_velocity;
}

double getAngularVelocity(const model::Twist& base_twist) {
    return base_twist.velocity * base_twist.curvature;
}

} // namespace

TruckState SimulatorEngine::getTruckState() const {
    const double steering = rear_ax_state_[StateIndex::kSteering];

    const auto pose = getOdomBasePose();
    const auto rear_curvature = model_->middleSteeringToRearCurvature(steering);
    const auto current_steering = getCurrentSteering(rear_curvature);
    const auto target_steering = getTargetSteering();
    const auto rear_twist = getRearTwist(rear_curvature);
    const auto twist = model_->rearToBaseTwist(rear_twist);
    const auto linear_velocity = rearToOdomBaseLinearVelocity(pose.dir, twist.velocity);
    const auto angular_velocity = getAngularVelocity(twist);
    auto lidar_ranges = map_->getLidarRanges(pose);
    const auto current_rps = model_->linearVelocityToMotorRPS(twist.velocity);
    const auto target_rps = model_->linearVelocityToMotorRPS(control_.velocity);
    const auto gyro_angular_velocity = getImuAngularVelocity(angular_velocity);
    const auto accel_linear_acceleration = getImuLinearAcceleration(rear_twist);
    
    return TruckState()
        .fail(fail_)
        .time(time_)
        .odomBasePose(pose)
        .currentSteering(current_steering)
        .targetSteering(target_steering)
        .baseTwist(twist)
        .odomBaseLinearVelocity(linear_velocity)
        .baseAngularVelocity(angular_velocity)
        .lidarRanges(std::move(lidar_ranges))
        .currentMotorRps(current_rps)
        .targetMotorRps(target_rps)
        .gyroAngularVelocity(gyro_angular_velocity)
        .accelLinearAcceleration(accel_linear_acceleration);
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

/**
 * @param desired_velocity The velocity to strive for.
 * @param velocity Current (initial) velocity.
 * @param precision Precision of calculations.
 * 
 * @return A pair of values:
 * first - the maneuver sign of the model;
 * second - the target speed.
 * 
 * If the sign is positive, it is necessary to accelerate. 
 * If the sign is negative, it is necessary to decelerate.
 * 
 * If the current and desired velocities are of different signs,
 * the target speed will be 0 (the model must first stop,
 * and then start moving in the opposite direction).
 * Otherwise, the target and desired velocities are the same.
 */
std::pair<int, double> actionSign(double desired_velocity, double velocity, double precision) {
    const bool need_stop = (softSign(desired_velocity, precision)
        * softSign(velocity, precision)) < 0;
    const double target_velocity = need_stop ? 0 : desired_velocity;

    return {softSign(abs(target_velocity) - abs(velocity), precision), target_velocity};
}

} // namespace

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    VERIFY(acceleration >= 0);

    curvature = model_->baseCurvatureLimits().clamp(curvature);
    velocity = model_->baseVelocityLimits().clamp(velocity);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    const auto [action_sign, _] = actionSign(rear_twist.velocity,
        rear_ax_state_[StateIndex::kLinearVelocity], params_.precision);
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

    const int action_sign = actionSign(rear_twist.velocity,
        rear_ax_state_[StateIndex::kLinearVelocity], params_.precision).first;
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
    const double velocity = rear_ax_state_[StateIndex::kLinearVelocity];
    
    const auto [action_sign, target_velocity] 
        = actionSign(control_.velocity, velocity, params_.precision);

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
    const double steering = rear_ax_state_[StateIndex::kSteering];
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
    
    const double yaw = state[StateIndex::kYaw];
    const double velocity = state[StateIndex::kLinearVelocity];
    const double steering = state[StateIndex::kSteering];

    SimulatorEngine::State deriv;
    deriv.setZero();
    deriv[StateIndex::kX] = cos(yaw) * velocity;
    deriv[StateIndex::kY] = sin(yaw) * velocity;
    deriv[StateIndex::kYaw] = tan(steering) * velocity * cache_.inverse_wheelbase_length;
    deriv[StateIndex::kLinearVelocity] = acceleration;
    deriv[StateIndex::kSteering] = steering_velocity;

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
    if (fail_) {
        return;
    }

    time_ += convertFromSecondsToDuration(seconds);

    const int integration_steps = seconds * cache_.inverse_integration_step;

    for (int i = 0; i < integration_steps; ++i) {
        const double current_acceleration = getCurrentAcceleration();
        const double current_steering_velocity = getCurrentSteeringVelocity();
        rear_ax_state_ += calculateRK4(current_acceleration, current_steering_velocity);
    }

    checkForCollisions();
}

}  // namespace truck::simulator
