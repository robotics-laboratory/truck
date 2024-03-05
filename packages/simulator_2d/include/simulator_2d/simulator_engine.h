#include "simulator_2d/truck_state.h"

#include "model/model.h"
#include "geom/angle.h"
#include "geom/angle_vector.h"
#include "geom/pose.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <eigen3/Eigen/Dense>

#include <optional>
#include <string>
#include <vector>

namespace truck::simulator {

class SimulatorEngine {
  public:
    SimulatorEngine(std::unique_ptr<model::Model> model,
        double integration_step = 1e-3, double precision = 1e-8);

    void resetBase(const geom::Pose& pose, double middle_steering, double linear_velocity);
    void resetMap(const std::string& path);
    void eraseMap();

    TruckState getTruckState() const;

    void setBaseControl(double velocity, double acceleration, double curvature);
    void setBaseControl(double velocity, double curvature);
    void advance(double seconds = 1.0);

  private:
    enum StateIndex { 
        kX = 0, 
        kY = 1, 
        kYaw = 2, 
        kSteering = 3, 
        kLinearVelocity = 4
    };

    using State = Eigen::Matrix<double, 5, 1>;

    void resetRear(double x, double y, double yaw,
        double steering, double linear_velocity);
    void resetRear();

    geom::Pose getOdomBasePose() const;
    model::Steering getCurrentSteering(double rear_curvature) const;
    model::Steering getTargetSteering() const;
    model::Twist rearToOdomBaseTwist(double rear_curvature) const;
    geom::Vec2 rearToOdomBaseLinearVelocity(
        truck::geom::AngleVec2 dir,double base_velocity) const;
    double rearToBaseAngularVelocity(
        double base_velocity, double rear_curvature) const;
    std::vector<float> getLidarRanges(const geom::Pose& odom_base_pose) const;

    double getCurrentAcceleration() const;
    double getCurrentSteeringVelocity() const;
    State calculateStateDerivative(const State &state,
        double acceleration, double steering_velocity) const;
    State calculateRK4(double acceleration, double steering_velocity) const;

    struct Parameters {
        double integration_step;
        double precision;
    } params_;

    struct Cache {
        double integration_step_2;
        double integration_step_6;
        double inverse_integration_step;
        double inverse_wheelbase_length;
        geom::Vec2 base_to_lidar;
        int lidar_rays_number;
        geom::AngleVec2 lidar_angle_min;
        geom::Angle lidar_angle_increment;
    } cache_;

    // The value is set in setControl and should not change in other methods.
    struct Control {
        double velocity = 0.0;
        std::optional<double> acceleration = 0.0;
        double curvature = 0.0;
    } control_;

    rclcpp::Time time_;

    State rear_ax_state_;

    std::unique_ptr<model::Model> model_ = nullptr;

    geom::Segments obstacles_;
};

}  // namespace truck::simulator
