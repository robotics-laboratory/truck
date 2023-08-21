#include "simulator_2d/simulator_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace truck::simulator {

SimulatorEngine::~SimulatorEngine() {
    isRunning_ = false;
    running_thread_.join();
}

void SimulatorEngine::start(std::unique_ptr<model::Model> &model, const double simulation_tick, 
    const int integration_steps, const double precision) {
    
    model_ = std::unique_ptr<model::Model>(std::move(model));
    params_.simulation_tick = simulation_tick;
    params_.integration_steps = integration_steps;
    params_.integration_step = simulation_tick / integration_steps;
    params_.precision = precision;
    params_.turning_speed = model_->wheelTurningSpeed();
    params_.wheelbase = model_->wheelBase().length;
    params_.steering_limit = model_->leftSteeringLimits().max.radians();
    state_.x = -params_.wheelbase / 2;
    isRunning_ = true;
    running_thread_ = std::thread(&SimulatorEngine::processSimulation, this);
}

geom::Pose SimulatorEngine::getPose() const {
    geom::Pose pose;
    const double l_b = params_.wheelbase / 2;
    pose.pos.x = state_.x + l_b * cos(state_.rotation);
    pose.pos.y = state_.y + l_b * sin(state_.rotation);
    pose.dir = geom::Vec2(cos(state_.rotation), sin(state_.rotation));
    return pose;
}

geom::Angle SimulatorEngine::getSteering() const {
    return geom::Angle(state_.steering);
}

double SimulatorEngine::getSpeed() const { return state_[StateIndex::linear_velocity]; }

geom::Vec2 SimulatorEngine::getLinearVelocity() const {
    return geom::Vec2(cos(state_.linear_velocity), sin(state_.linear_velocity));
}

geom::Vec2 SimulatorEngine::getAngularVelocity() const {
    return geom::Vec2(cos(state_.angular_velocity), sin(state_.angular_velocity));
}

void SimulatorEngine::setControl(
    const double velocity, const double acceleration, const double curvature) {

    control_.velocity = model_->baseVelocityLimits().clamp(velocity);
    control_.acceleration = model_->baseAccelerationLimits().clamp(acceleration);
    const auto curvature_limit = model_->baseMaxAbsCurvature();
    control_.curvature 
        = std::clamp(curvature, -curvature_limit, curvature_limit);
    /*
    RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), 
        "v = " + std::to_string(control_.velocity) 
        + " a = " + std::to_string(control_.acceleration) 
        + " c = " + std::to_string(control_.curvature));
    //*/
}

void SimulatorEngine::setControl(
    const double velocity, const double curvature) {
    
    const auto limits = model_->baseAccelerationLimits();
    const double acceleration = abs(velocity - state_.linear_velocity) < params_.precision
        ? 0 
        : state_.linear_velocity < velocity
            ? limits.max
            : limits.min;
    setControl(velocity, acceleration, curvature);
}

void SimulatorEngine::calculate_state_delta(const SimulationState &state,
    const double acceleration, const double &steering_delta, SimulationState &delta) {
    
    delta.x = cos(state.rotation) * state.linear_velocity;
    delta.y = sin(state.rotation) * state.linear_velocity;
    delta.rotation = tan(state.steering) * state.linear_velocity / params_.wheelbase;
    delta.steering = steering_delta;
    delta.linear_velocity = acceleration;
}

void SimulatorEngine::updateState() {
    const double old_rotation = state_.rotation;

    const double steering_final = abs(control_.curvature) < params_.precision 
        ? 0 
        : std::clamp(control_.curvature > 0
            ? atan2(params_.wheelbase, control_.curvature)
            : -atan2(params_.wheelbase, -control_.curvature), 
        -params_.steering_limit, +params_.steering_limit);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "000000000000000000000000");
    double steering_delta = abs(steering_final - state_.steering) < params_.precision 
        ? 0 
        : state_.steering < steering_final
            ? params_.turning_speed
            : -params_.turning_speed;
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "11111111111111111111111");
    double acceleration = abs(control_.velocity - state_.linear_velocity) < params_.precision 
        ? 0 
        : control_.acceleration;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "2222222222222222222222222");
    SimulationState k[4];
    for (auto i = 0; i < params_.integration_steps; ++i) {
        if (abs(steering_final - state_.steering) - params_.precision 
            < abs(params_.simulation_tick * steering_delta)) {
            steering_delta = steering_final - state_.steering;
        }
//RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "3333333333333333333333333");
        if (abs(control_.velocity - state_.linear_velocity) - params_.precision
            < abs(params_.simulation_tick * acceleration)) {
            acceleration = control_.velocity - state_.linear_velocity;
        }
//RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "44444444444444444444444444");
        calculate_state_delta(state_, acceleration, steering_delta, k[0]);
        calculate_state_delta(state_ + k[0] * (params_.integration_step / 2),
            acceleration, steering_delta, k[1]);
        calculate_state_delta(state_ + k[1] * (params_.integration_step / 2),
            acceleration, steering_delta, k[2]);
        calculate_state_delta(state_ + k[2] * params_.integration_step,
            acceleration, steering_delta, k[3]);
        k[1] *= 2;
        k[2] *= 2;
        SimulationState::addSum(state_, k, 4, params_.integration_step / 6);
        
RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "5555555555555555555555555555");
        state_.rotation = fmod(state_.rotation, 2 * M_PI);
        if (state_.rotation < 0) {
            state_.rotation += 2 * M_PI;
        }
//RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "6666666666666666666666666666");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), "9999999999999999999999999999");
    state_.angular_velocity = (state_.rotation - old_rotation) / params_.simulation_tick;
}

void SimulatorEngine::advance(const double time) {
    const int integration_steps = time / params_.integration_step;

    const double old_rotation = state_[StateIndex::rotation];

    const double steering_final = abs(control_.curvature) < params_.precision
                                      ? 0
                                      : std::clamp(
                                            atan(params_.wheelbase * control_.curvature),
                                            -params_.steering_limit,
                                            +params_.steering_limit);

    double steering_rest = abs(steering_final - state_[StateIndex::steering]);
    double steering_velocity = steering_rest < params_.precision ? 0
                               : state_[StateIndex::steering] < steering_final
                                   ? params_.max_steering_velocity
                                   : -params_.max_steering_velocity;

    double velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]);
    double acceleration = velocity_rest < params_.precision ? 0 : control_.acceleration;

    for (auto i = 0; i < integration_steps; ++i) {
        steering_rest = abs(steering_final - state_[StateIndex::steering]) -
                        abs(steering_velocity) * params_.integration_step / 6;
        if (steering_rest < params_.precision) {
            state_[StateIndex::steering] = steering_final;
            steering_velocity = 0;
        }

        velocity_rest = abs(control_.velocity - state_[StateIndex::linear_velocity]) -
                        abs(acceleration) * params_.integration_step / 6;
        if (velocity_rest < params_.precision) {
            state_[StateIndex::linear_velocity] = control_.velocity;
            acceleration = 0;
        }

        auto k1 = calculateStateDelta(state_, acceleration, steering_velocity);
        auto k2 = calculateStateDelta(
            state_ + k1 * (params_.integration_step / 2), acceleration, steering_velocity);
        auto k3 = calculateStateDelta(
            state_ + k2 * (params_.integration_step / 2), acceleration, steering_velocity);
        auto k4 = calculateStateDelta(
            state_ + k3 * params_.integration_step, acceleration, steering_velocity);

        state_ += (k1 + 2 * k2 + 2 * k3 + k4) * (params_.integration_step / 6);
    }

    state_[StateIndex::angular_velocity] 
        = (state_[StateIndex::rotation] - old_rotation) / time;
}

}  // namespace truck::simulator