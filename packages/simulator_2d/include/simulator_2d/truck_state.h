#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::simulator {

class TruckState {
  public:
    const rclcpp::Time& getTime() const;
    geom::Pose getBaseOdomPose() const;
    model::Steering getCurrentSteering() const;
    model::Steering getTargetSteering() const;
    model::Twist getBaseOdomTwist() const;
    geom::Vec2 getBaseOdomLinearVelocity() const;
    double getBaseOdomAngularVelocity() const;

    TruckState& setTime(const rclcpp::Time& time);
    TruckState& setBaseOdomPose(const geom::Pose& pose);
    TruckState& setCurrentSteering(const model::Steering& current_steering);
    TruckState& setTargetSteering(const model::Steering& target_steering);
    TruckState& setBaseOdomTwist(const model::Twist& twist);
    TruckState& setBaseOdomLinearVelocity(const geom::Vec2& linear_velocity);
    TruckState& setBaseOdomAngularVelocity(double angular_velocity);

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
