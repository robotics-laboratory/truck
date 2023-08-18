#include "simulator_2d/simulator_engine.h"

#include <chrono>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::~SimulatorEngine() {
    isRunning_ = false;
    running_thread_.join();
}

void SimulatorEngine::start(std::unique_ptr<model::Model> &model, 
    const double simulation_tick, const double precision) {
    
    params_.simulation_tick = simulation_tick;
    params_.precision = precision;
    model_ = std::unique_ptr<model::Model>(std::move(model));
    isRunning_ = true;
    running_thread_ = std::thread(&SimulatorEngine::processSimulation, this);
}

geom::Vec2 SimulatorEngine::getTruckSizes() const {
    return geom::Vec2(model_->shape().length, model_->shape().width);
}

geom::Pose SimulatorEngine::getPose() const {
    return state_.pose;
}

geom::Angle SimulatorEngine::getSteering() const {
    return state_.steering;
}

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return state_.linearVelocity;
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return state_.angularVelocity;
}

void SimulatorEngine::setControl(
    const double velocity, const double acceleration, const double curvature) {
    
    control_.velocity = velocity;
    control_.acceleration = acceleration;
    control_.curvature = curvature;
    /*
    RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), 
        std::to_string(velocity) + " " + std::to_string(acceleration) 
        + " " + std::to_string(curvature));
    //*/
}

void SimulatorEngine::setControl(
    const double velocity, const double curvature) {

    const double acceleration = abs(velocity - control_.velocity) < params_.precision
        ? 0 
        : velocity < control_.velocity - params_.precision 
            ? model_->baseAccelerationLimits().min
            : model_->baseAccelerationLimits().max;
    setControl(velocity, acceleration, curvature);
}

void SimulatorEngine::updateState() {
    state_.pose.pos.x += params_.simulation_tick * control_.velocity;
    state_.pose.pos.y += params_.simulation_tick * abs(control_.velocity) * control_.curvature / 3;
}

void SimulatorEngine::processSimulation() {
    auto wait_time = std::chrono::duration<double>(params_.simulation_tick);
    while (isRunning_) {
        updateState();
        std::this_thread::sleep_for(wait_time);
    }
}

} // namespace truck::simulator