#include "simulator_2d/simulator_engine.h"

#include <chrono>

SimulatorEngine::SimulatorEngine() {
    model_ = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.simulation_tick), 
        std::bind(&SimulatorNode::updateState, this));
}

geom::Vec2 SimulatorEngine::getTruckSizes() {
    return new geom::Vec2(model_->shape().length, model_->shape().width);
}

geom::Pose SimulatorEngine::getPose() {
    return state_.pose;
}

geom::Angle SimulatorEngine::getSteering() {
    return state_.steering;
}

void SimulatorEngine::setControl(double velocity, double acceleration, double curvature) {
    control_.velocity = velocity;
    control_.acceleration = acceleration;
    control_.curvature = curvature;
}

void SimulatorEngine::updateState() {
    state_.pose.pos.x = params_.simulation_tick * control_.velocity;
}