namespace truck::simulator {

struct SimulationState {
    double x;
    double y;
    double rotation;
    double steering;
    double linear_velocity;
    double angular_velocity;

    SimulationState();
    SimulationState(const double x, const double y, const double rotation, 
        const double steering, const double linear_velocity);
    SimulationState& operator=(const SimulationState& other);
    SimulationState& operator +(const SimulationState& other) const;
    SimulationState& operator +=(const SimulationState& other);
    SimulationState& operator *(const double number) const;
    SimulationState& operator *=(const double number);
    static void addSum(SimulationState &state, const SimulationState *sum);
};

} // namespace truck::simulator