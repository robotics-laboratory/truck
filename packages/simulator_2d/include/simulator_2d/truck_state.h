#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::simulator {

class TruckState {
  public:
    TruckState(const rclcpp::Time& time, const geom::Pose& pose,
        const model::Steering& current_steering, const model::Steering& target_steering,
        const model::Twist& twist, const geom::Vec2& linear_velocity,
        const geom::Vec2& angular_velocity);

    static std::unique_ptr<TruckState> fromRearToBaseState(const model::Model& model,
        double x, double y, double yaw, const rclcpp::Time& time, double steering, 
        double linear_velocity, double control_curvature);

    const rclcpp::Time& getTime() const;
    geom::Pose getPose() const;
    model::Steering getCurrentSteering() const;
    model::Steering getTargetSteering() const;
    model::Twist getTwist() const;
    geom::Vec2 getLinearVelocityVector() const;
    geom::Vec2 getAngularVelocityVector() const;

  private:

    static geom::Pose getPose(const model::Model& model, 
        double x, double y, double yaw);
    static model::Steering getCurrentSteering(
        const model::Model& model, double rear_curvature);
    static model::Steering getTargetSteering(
        const model::Model& model, double control_curvature);
    static model::Twist getTwist(const model::Model& model, 
        double rear_curvature, double linear_velocity);
    static geom::Vec2 getLinearVelocityVector(double yaw, 
        double base_velocity);
    static geom::Vec2 getAngularVelocityVector(double yaw, 
        double base_velocity, double rear_curvature);
    
    struct Cache {
        rclcpp::Time time;
        geom::Pose pose;
        model::Steering current_steering;
        model::Steering target_steering;
        model::Twist twist;
        geom::Vec2 linear_velocity;
        geom::Vec2 angular_velocity;
    } cache_;
};

}  // namespace truck::simulator
