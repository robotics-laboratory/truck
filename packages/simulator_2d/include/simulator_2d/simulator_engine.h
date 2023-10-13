#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

class SimulatorEngine {
  public:
    SimulatorEngine(const std::string& model_config_path, 
        double integration_step = 0.001, double precision = 1e-8);
    void reset();
    rclcpp::Time getTime() const;
    geom::Pose getPose() const;
    geom::Angle getMiddleSteering() const;
    geom::Angle getLeftSteering() const;
    geom::Angle getRightSteering() const;
    geom::Angle getTargetLeftSteering() const;
    geom::Angle getTargetRightSteering() const;
    double getSpeed() const;
    geom::Vec2 getLinearVelocity() const;
    geom::Vec2 getAngularVelocity() const;
    void setControl(double velocity, double acceleration, double curvature);
    void setControl(double velocity, double curvature);
    /**
     * @param time in seconds.
     */
    void advance(const double time = 1.0);

  private:
    rclcpp::Time time_;

    typedef Eigen::Matrix<double, 6, 1> State;

    State calculateStateDelta(
        const State &state, const double acceleration,
        const double steering_velocity);

    struct Parameters {
        double integration_step;
        double integration_step_2;
        double integration_step_6;
        double inverse_integration_step;
        double precision;
        double inverse_wheelbase_length;
        double wheelbase_width_2;
    } params_;

    struct Control {
        double velocity = 0.0;
        double acceleration = 0.0;
        double curvature = 0.0;
    } control_;

    enum StateIndex {
        x = 0,
        y = 1,
        yaw = 2,
        steering = 3,
        linear_velocity = 4,
        angular_velocity = 5
    };

    // For the rear axle.
    State state_;

    model::Model model_;
};

}  // namespace truck::simulator
