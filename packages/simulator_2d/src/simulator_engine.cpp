#include "simulator_2d/simulator_engine.h"

#include <chrono>

// Delete this! And logger!
#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(std::unique_ptr<model::Model> &model) {
    model_ = std::unique_ptr<model::Model>(std::move(model));
    
    running_thread_ = std::thread(&SimulatorEngine::processSimulation, this);
    isRunning_ = true;
    
    /*
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.simulation_tick), 
        std::bind(&SimulatorNode::updateState, this));
    //*/
}

SimulatorEngine::~SimulatorEngine() {
    isRunning_ = false;
    running_thread_.join();
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

void SimulatorEngine::setControl(
    const double velocity, const double acceleration, const double curvature) {
    control_.velocity = velocity;
    control_.acceleration = acceleration;
    control_.curvature = curvature;
}

void SimulatorEngine::updateState() {
    state_.pose.pos.x += params_.simulation_tick * control_.velocity;
    //RCLCPP_INFO(rclcpp::get_logger("simulator_logger"), std::format("{} * {} = {}", params_.simulation_tick, control_.velocity, state_.pose.pos.x));
    if (control_.velocity > 0.01) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_logger"), 
            std::to_string(control_.velocity) + " " + std::to_string(state_.pose.pos.x));
    }
}

void SimulatorEngine::processSimulation() {
    auto wait_time = std::chrono::duration<double>(params_.simulation_tick);
    while (isRunning_) {
        updateState();
        std::this_thread::sleep_for(wait_time);
    }
}

} // namespace truck::simulator