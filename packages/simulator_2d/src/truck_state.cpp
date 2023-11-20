#include "simulator_2d/truck_state.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

const rclcpp::Time& TruckState::getTime() const {
    return cache_.time;
}

geom::Pose TruckState::getBaseOdomPose() const {
    return cache_.base_odom_pose;
}

model::Steering TruckState::getCurrentSteering() const {
    return cache_.current_steering;
}

model::Steering TruckState::getTargetSteering() const {
    return cache_.target_steering;
}

model::Twist TruckState::getBaseOdomTwist() const {
    return cache_.base_odom_twist;
}

geom::Vec2 TruckState::getBaseOdomLinearVelocity() const {
    return cache_.base_odom_linear_velocity;
}

double TruckState::getBaseOdomAngularVelocity() const {
    return cache_.base_odom_angular_velocity;
}

TruckState& TruckState::setTime(const rclcpp::Time& time) {
    cache_.time = time;
}

TruckState& TruckState::setBaseOdomPose(const geom::Pose& pose) {
    cache_.base_odom_pose = pose;
}

TruckState& TruckState::setCurrentSteering(const model::Steering& current_steering) {
    cache_.current_steering = current_steering;
}

TruckState& TruckState::setTargetSteering(const model::Steering& target_steering) {
    cache_.target_steering = target_steering;
}

TruckState& TruckState::setBaseOdomTwist(const model::Twist& twist) {
    cache_.base_odom_twist = twist;
}

TruckState& TruckState::setBaseOdomLinearVelocity(const geom::Vec2& linear_velocity) {
    cache_.base_odom_linear_velocity = linear_velocity;
}

TruckState& TruckState::setBaseOdomAngularVelocity(double angular_velocity) {
    cache_.base_odom_angular_velocity = angular_velocity;
}

}  // namespace truck::simulator
