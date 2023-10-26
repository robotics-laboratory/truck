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
    void reset(double x, double y, double yaw, double steering, 
        double linear_velocity, double angular_velocity);
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

    typedef Eigen::Matrix<double, 6, 1> State;

    State calculateStateDelta(
        const State &state, const double acceleration);

    struct Parameters {
        double integration_step;
        double precision;    
    } params_;

    struct Cache {
        double integration_step_2;
        double integration_step_6;
        double inverse_integration_step;
        double inverse_wheelbase_length;
        double wheelbase_width_2;
    } cache_;

    struct Control {
        double velocity = 0.0;
        double acceleration = 0.0;
        double curvature = 0.0;
    } control_;

    // For the rear axle.
    State state_;

    enum StateIndex { 
        x = 0, 
        y = 1, 
        yaw = 2, 
        steering = 3, 
        linear_velocity = 4, 
        angular_velocity = 5 
    };

    model::Model model_;
};

}  // namespace truck::simulator
