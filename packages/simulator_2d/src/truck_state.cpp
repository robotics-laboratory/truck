#include "simulator_2d/truck_state.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

TruckState::TruckState(const rclcpp::Time& time, const geom::Pose& pose,
    const model::Steering& current_steering, const model::Steering& target_steering,
    const model::Twist& twist, const geom::Vec2& linear_velocity,
    const geom::Vec2& angular_velocity) {
    
    cache_.time = time;
    cache_.pose = pose;
    cache_.current_steering = current_steering;
    cache_.target_steering = target_steering;
    cache_.twist = twist;
    cache_.linear_velocity = linear_velocity;
    cache_.angular_velocity = angular_velocity;
}

const rclcpp::Time& TruckState::getTime() const {
    return cache_.time;
}

geom::Pose TruckState::getPose() const {
    return cache_.pose;
}

model::Steering TruckState::getCurrentSteering() const {
    return cache_.current_steering;
}

model::Steering TruckState::getTargetSteering() const {
    return cache_.target_steering;
}

model::Twist TruckState::getTwist() const {
    return cache_.twist;
}

geom::Vec2 TruckState::getLinearVelocityVector() const {
    return cache_.linear_velocity;
}

geom::Vec2 TruckState::getAngularVelocityVector() const {
    return cache_.angular_velocity;
}

std::unique_ptr<TruckState> TruckState::fromRearToBaseState(const model::Model& model, double x,
    double y, double yaw, const rclcpp::Time& time, double steering, 
    double linear_velocity, double control_curvature) {

    const auto pose = getPose(model, x, y, yaw);
    const double rear_curvature = getCurrentRearCurvature(model, steering);
    const auto current_steering = getCurrentSteering(model, rear_curvature);
    const auto target_steering = getTargetSteering(model, control_curvature);
    const auto twist = getTwist(model, rear_curvature, linear_velocity);
    const auto linear_velocity_vector = getLinearVelocityVector(yaw, twist.velocity);
    const auto angular_velocity_vector = getAngularVelocityVector(yaw,
        twist.velocity, rear_curvature);

    return std::make_unique<TruckState>(TruckState(time, pose, current_steering,
        target_steering, twist, linear_velocity_vector, angular_velocity_vector));
}

geom::Pose TruckState::getPose(const model::Model& model, 
    double x, double y, double yaw) {

    geom::Pose pose;
    pose.dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    pose.pos.x = x + model.wheelBase().base_to_rear * pose.dir.x();
    pose.pos.y = y + model.wheelBase().base_to_rear * pose.dir.y();
    return pose;
}

double TruckState::getCurrentRearCurvature(
    const model::Model& model, double steering) {

    return tan(steering) / model.wheelBase().length;
}

model::Steering TruckState::getCurrentSteering(
    const model::Model& model, double rear_curvature) {

    return model.rearCurvatureToSteering(rear_curvature);
}

model::Steering TruckState::getTargetSteering(
    const model::Model& model, double control_curvature) {

    return model.rearCurvatureToSteering(control_curvature);
}

model::Twist TruckState::getTwist(const model::Model& model, 
    double rear_curvature, double linear_velocity) { 

    const auto twist = model::Twist {
        rear_curvature,
        linear_velocity
    };
    return model.rearToBaseTwist(twist);
}

geom::Vec2 TruckState::getLinearVelocityVector(double yaw, 
    double base_velocity) {

    const auto dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    return dir * base_velocity;
}

geom::Vec2 TruckState::getAngularVelocityVector(double yaw, 
    double base_velocity, double rear_curvature) {

    const auto dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    const double angular_velocity = base_velocity * rear_curvature;
    return dir * angular_velocity;
}

}  // namespace truck::simulator
