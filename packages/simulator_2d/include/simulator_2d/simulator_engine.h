#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

typedef Eigen::Matrix<double, 6, 1> State;

enum StateIndex { 
    x = 0, 
    y = 1, 
    yaw = 2, 
    steering = 3, 
    linear_velocity = 4, 
    angular_velocity = 5 
};

class SimulatorEngine {
  public:
    SimulatorEngine(const std::string& model_config_path, 
        double integration_step = 0.001, double precision = 1e-8);
    void reset(const State &state);
    void reset();
    rclcpp::Time getTime() const;
    geom::Pose getPose() const;
    double getCurrentCurvature() const;
    double getMiddleSteering() const;
    model::Steering getCurrentSteering() const;
    model::Steering getTargetSteering() const;
    model::Twist getTwist() const;
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

    // For the rear axle.
    State state_;

    model::Model model_;
};

}  // namespace truck::simulator
