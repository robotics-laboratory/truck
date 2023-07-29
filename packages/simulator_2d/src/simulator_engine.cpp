#include "simulator_2d/simulator_engine.h"

#include <chrono>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(std::unique_ptr<model::Model> &model, double simulation_tick) {
    params_.simulation_tick = simulation_tick;
    model_ = std::unique_ptr<model::Model>(std::move(model));
    running_thread_ = std::thread(&SimulatorEngine::processSimulation, this);
    isRunning_ = true;
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
}

void SimulatorEngine::processSimulation() {
    auto wait_time = std::chrono::duration<double>(params_.simulation_tick);
    while (isRunning_) {
        updateState();
        std::this_thread::sleep_for(wait_time);
    }
}

} // namespace truck::simulator