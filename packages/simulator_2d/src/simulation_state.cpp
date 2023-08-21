#include "simulator_2d/simulation_state.h"

// REMOVE THIS
#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

SimulationState::SimulationState() {
    x = 0.0;
    y = 0.0;
    rotation = 0.0;
    steering = 0.0;
    linear_velocity = 0.0;
    angular_velocity = 0.0;
}

SimulationState::SimulationState(const double x, const double y, const double rotation, 
    const double steering, const double linear_velocity) {
    
    this->x = x;
    this->y = y;
    this->rotation = rotation;
    this->steering = steering;
    this->linear_velocity = linear_velocity;
}

 SimulationState& SimulationState::operator =(const SimulationState& other) {
    if (this == &other) {
        return *this;
    }

    x = other.x; 
    y = other.y; 
    rotation = other.rotation;
    steering = other.steering; 
    linear_velocity = other.linear_velocity;
    angular_velocity = other.angular_velocity;
    return *this;
}

SimulationState& SimulationState::operator +(const SimulationState& other) const {
    return *(new SimulationState(x + other.x, y + other.y, rotation + other.rotation,
        steering + other.steering, linear_velocity + other.linear_velocity));
}

SimulationState& SimulationState::operator +=(const SimulationState& other) {
    x += other.x; 
    y += other.y; 
    rotation += other.rotation;
    steering += other.steering; 
    linear_velocity += other.linear_velocity;
    return *this;
}

SimulationState& SimulationState::operator *(const double number) const {
    return *(new SimulationState(number * x, number * y, number * rotation,
        number * steering, number * linear_velocity));
}

SimulationState& SimulationState::operator *=(const double number) {
    x *= number;
    y *= number; 
    rotation *= number;
    steering *= number; 
    linear_velocity *= number;
    return *this;
}

void SimulationState::addSum(SimulationState &state, const SimulationState *sum, 
    const int summands_number, const double ratio) {

    for (auto i = 0; i < summands_number; ++i) {
        state.x += sum[i].x * ratio;
        state.y += sum[i].y * ratio;
        state.rotation += sum[i].rotation * ratio;
        state.steering += sum[i].steering * ratio;
        state.linear_velocity += sum[i].linear_velocity * ratio;
    }
}

} // namespace truck::simulator