#include "model/model.h"

#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::simulator {

class SimulatorEngine {
    public:
        SimulatorEngine();
        geom::Vec2 getTruckSizes();
        geom::Pose getPose();
        geom::Angle getSteering();
        void setControl(double velocity, double acceleration, double curvature);

    private:
        void updateState();

        std::unique_ptr<model::Model> model_ = nullptr;

        rclcpp::TimerBase::SharedPtr timer_ = nullptr;

        struct Parameters {
            double simulation_tick = 0.01;
        } params_;

        struct State {
            // The coordinates and the yaw.
            geom::Pose pose;
            // The position of the virtual corner in the middle (bicycle model).
            geom::Angle steering;
            //double velocity = 0.0;
        } state_;

        struct Control {
            double velocity = 0.0;
            double acceleration = 0.0;
            double curvature = 0.0;
        } control_;
};

} // namespace truck::simulator