#include "simulator_2d/truck_state.h"

#include <algorithm>
#include <cmath>

namespace truck::simulator {

bool TruckState::fail() const {
    return cache_.fail;
}

rclcpp::Time TruckState::time() const {
    return cache_.time;
}

geom::Pose TruckState::odomBasePose() const {
    return cache_.base_odom_pose;
}

model::Steering TruckState::currentSteering() const {
    return cache_.current_steering;
}

model::Steering TruckState::targetSteering() const {
    return cache_.target_steering;
}

model::Twist TruckState::baseTwist() const {
    return cache_.base_odom_twist;
}

geom::Vec2 TruckState::odomBaseLinearVelocity() const {
    return cache_.base_odom_linear_velocity;
}

double TruckState::baseAngularVelocity() const {
    return cache_.base_odom_angular_velocity;
}

const std::vector<float>& TruckState::lidarRanges() const {
    return cache_.lidar_ranges;
}

double TruckState::currentMotorRps() const {
    return cache_.current_motor_rps;
}

double TruckState::targetMotorRps() const {
    return cache_.target_motor_rps;
}

geom::Vec3 TruckState::gyroAngularVelocity() const {
    return cache_.gyro_angular_velocity;
}

geom::Vec3 TruckState::accelLinearAcceleration() const {
    return cache_.accel_linear_acceleration;
}

TruckState& TruckState::fail(bool fail) {
    cache_.fail = fail;
    return *this;
}

TruckState& TruckState::time(const rclcpp::Time& time) {
    cache_.time = time;
    return *this;
}

TruckState& TruckState::odomBasePose(const geom::Pose& pose) {
    cache_.base_odom_pose = pose;
    return *this;
}

TruckState& TruckState::currentSteering(const model::Steering& current_steering) {
    cache_.current_steering = current_steering;
    return *this;
}

TruckState& TruckState::targetSteering(const model::Steering& target_steering) {
    cache_.target_steering = target_steering;
    return *this;
}

TruckState& TruckState::baseTwist(const model::Twist& twist) {
    cache_.base_odom_twist = twist;
    return *this;
}

TruckState& TruckState::odomBaseLinearVelocity(const geom::Vec2& linear_velocity) {
    cache_.base_odom_linear_velocity = linear_velocity;
    return *this;
}

TruckState& TruckState::baseAngularVelocity(double angular_velocity) {
    cache_.base_odom_angular_velocity = angular_velocity;
    return *this;
}

TruckState& TruckState::lidarRanges(std::vector<float> lidar_ranges) {
    cache_.lidar_ranges = std::move(lidar_ranges);
    return *this;
}

TruckState& TruckState::currentMotorRps(double current_rps) {
    cache_.current_motor_rps = current_rps;
    return *this;
}

TruckState& TruckState::targetMotorRps(double target_rps) {
    cache_.target_motor_rps = target_rps;
    return *this;
}

TruckState& TruckState::gyroAngularVelocity(geom::Vec3 angular_velocity) {
    cache_.gyro_angular_velocity = angular_velocity;
    return *this;
}

TruckState& TruckState::accelLinearAcceleration(geom::Vec3 linear_acceleration) {
    cache_.accel_linear_acceleration = linear_acceleration;
    return *this;
}

}  // namespace truck::simulator
