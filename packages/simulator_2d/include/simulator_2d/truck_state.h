#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::simulator {

class TruckState {
  public:
    rclcpp::Time time() const;
    geom::Pose odomBasePose() const;
    model::Steering currentSteering() const;
    model::Steering targetSteering() const;
    model::Twist baseTwist() const;
    geom::Vec2 odomBaseLinearVelocity() const;
    double baseAngularVelocity() const;

    TruckState& time(const rclcpp::Time& time);
    TruckState& odomBasePose(const geom::Pose& pose);
    TruckState& currentSteering(const model::Steering& current_steering);
    TruckState& targetSteering(const model::Steering& target_steering);
    TruckState& baseTwist(const model::Twist& twist);
    TruckState& odomBaseLinearVelocity(const geom::Vec2& linear_velocity);
    TruckState& baseAngularVelocity(double angular_velocity);

  private:
    struct Cache {
        rclcpp::Time time;
        geom::Pose base_odom_pose;
        model::Steering current_steering;
        model::Steering target_steering;
        model::Twist base_odom_twist;
        geom::Vec2 base_odom_linear_velocity;
        double base_odom_angular_velocity;
    } cache_;
};

}  // namespace truck::simulator
