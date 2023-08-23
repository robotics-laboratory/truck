#include "simulator_2d/simulation_state.h"

#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

class SimulatorEngine {
    public:
        ~SimulatorEngine();
        void start(std::unique_ptr<model::Model> &model, const double simulation_tick = 0.01, 
            const int integration_steps = 1000, const double precision = 1e-8);
        geom::Pose getPose() const;
        geom::Angle getSteering() const;
        geom::Vec2 getLinearVelocity() const;
        geom::Vec2 getAngularVelocity() const;
        void setControl(const double velocity, const double acceleration, const double curvature);
        void setControl(const double velocity, const double curvature);
        void suspend();
        void resume();

    private:
        void calculate_state_delta(const SimulationState &state,
            const double acceleration, const double &steering_delta, SimulationState &delta);
        void updateState();
        void processSimulation();

        bool isRunning_ = false;
        bool isResumed_ = false;

        std::thread running_thread_;

        std::unique_ptr<model::Model> model_ = nullptr;

        struct Parameters {
            double simulation_tick;
            int integration_steps;
            double integration_step;
            double precision;
            double turning_speed;
            double wheelbase;
            double steering_limit;
        } params_;

        SimulationState state_;

        struct Control {
            double velocity = 0.0;
            double acceleration = 0.0;
            double curvature = 0.0;
        } control_;
};

}  // namespace truck::simulator