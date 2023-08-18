#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

class SimulatorEngine {
    public:
        ~SimulatorEngine();
        void start(std::unique_ptr<model::Model> &model, 
            const double simulation_tick, const double precision = 1e-8);
        geom::Vec2 getTruckSizes() const;
        geom::Pose getPose() const;
        geom::Angle getSteering() const;
        geom::Vec2 getLinearVelocity() const;
        geom::Vec2 getAngularVelocity() const;
        void setControl(const double velocity, const double acceleration, const double curvature);
        void setControl(const double velocity, const double curvature);

    private:
        void updateState();
        void processSimulation();
        void calculateAcceleration(const double velocity, const double curvature);

        bool isRunning_ = false;

        std::thread running_thread_;

        std::unique_ptr<model::Model> model_ = nullptr;

        struct Parameters {
            double simulation_tick;
            double precision;
        } params_;

        struct State {
            // The coordinates and the yaw.
            geom::Pose pose;
            // The position of the virtual corner in the middle (bicycle model).
            geom::Angle steering;
            geom::Vec2 linearVelocity;
            geom::Vec2 angularVelocity;
        } state_;

        struct Control {
            double velocity = 0.0;
            double acceleration = 0.0;
            double curvature = 0.0;
        } control_;
};

}  // namespace truck::simulator