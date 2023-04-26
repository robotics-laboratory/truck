#include "truck_gazebo_plugins/ackermann_model.h"
#include "truck_gazebo_plugins/common.h"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <exception>
#include <sstream>
#include <string>

namespace gazebo {

void AckermannModelPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    node_ = gazebo_ros::Node::Get(sdf);

    model_ = truck::model::makeUniquePtr(
        node_->get_logger(),
        GetParam<std::string>(sdf, "config_path"));

    {  // steering
        const auto steering = GetElement(sdf, "steering");

        steering_error_ = GetParam<float>(steering, "error");

        steering_velocity_ =
            truck::geom::Angle::fromDegrees(GetParam<double>(steering, "velocity"));

        steering_torque_ = GetParam<float>(steering, "torque");

        steering_left_joint_ = GetJoint(model, GetParam<std::string>(steering, "left_joint"));
        steering_right_joint_ = GetJoint(model, GetParam<std::string>(steering, "right_joint"));
    }

    {  // rear
        auto rear = GetElement(sdf, "rear");

        rear_left_joint_ = GetJoint(model, GetParam<std::string>(rear, "left_joint"));
        rear_right_joint_ = GetJoint(model, GetParam<std::string>(rear, "right_joint"));

        const auto pd = GetParam<ignition::math::Vector2d>(rear, "pd");
        const auto motor_torque = GetParam<float>(rear, "torque");
        const float wheel_torque = motor_torque / 2 * model_->gearRatio();

        velocity_left_pd_.SetPGain(pd.X());
        velocity_left_pd_.SetDGain(pd.Y());
        velocity_left_pd_.SetCmdMin(-wheel_torque);
        velocity_left_pd_.SetCmdMax(+wheel_torque);

        velocity_right_pd_.SetPGain(pd.X());
        velocity_right_pd_.SetDGain(pd.Y());
        velocity_right_pd_.SetCmdMin(-wheel_torque);
        velocity_right_pd_.SetCmdMax(+wheel_torque);

        gzwarn << "velocity:pd: p=" << pd.X() << " d=" << pd.Y() << " t=" << motor_torque
              << std::endl;
    }

    command_slot_ = node_->create_subscription<truck_msgs::msg::Control>(
        "/control/command", 1, [this](truck_msgs::msg::Control::ConstSharedPtr command) {
            std::lock_guard lock(mutex_);
            command_ = std::move(command);
        });

    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AckermannModelPlugin::OnUpdate, this, std::placeholders::_1));
}

void AckermannModelPlugin::OnUpdate(const common::UpdateInfo& info) {
    std::lock_guard lock(mutex_);

    auto set_target = [this, &info](double velocity, double curvature) {
        const truck::model::Twist base_twist{
            truck::clamp(curvature, model_->baseMaxAbsCurvature()),
            model_->baseVelocityLimits().clamp(velocity)};

        const double delta = (info.simTime - last_update_time_).Double();

        const auto rear_twist = model_->baseToRearTwist(base_twist);
        const auto steering = model_->rearTwistToSteering(rear_twist);

        auto diff_to_velocity = [this](double steering_diff) -> double {
            const double steering_velocity = steering_velocity_.radians();

            if (steering_diff > +steering_error_) {
                return +steering_velocity;
            }

            if (steering_diff < -steering_error_) {
                return -steering_velocity;
            }

            return 0.0;
        };

        const auto steering_left = steering_left_joint_->Position(0);
        const double steering_left_diff = steering.left.radians() - steering_left;

        steering_left_joint_->SetParam("fmax", 0, steering_torque_);
        steering_left_joint_->SetParam("vel", 0, diff_to_velocity(steering_left_diff));

        const auto steering_right = steering_right_joint_->Position(0);
        const double steering_right_diff = steering.right.radians() - steering_right;

        steering_right_joint_->SetParam("fmax", 0, steering_torque_);
        steering_right_joint_->SetParam("vel", 0, diff_to_velocity(steering_right_diff));

        const auto wheel_velocity = model_->rearTwistToWheelVelocity(rear_twist);

        const double velocity_left = rear_left_joint_->GetVelocity(0);
        const double velocity_right = rear_right_joint_->GetVelocity(0);

        if (std::isnan(velocity_left) || std::isnan(velocity_right)) {
            gzerr << "velocity[l=" << velocity_left << ", r=" << velocity_right << "]" << std::endl;
            throw std::runtime_error("Bad velocity!");
        }

        const double velocity_left_force =
            velocity_left_pd_.Update(velocity_left - wheel_velocity.left.radians(), delta);
        rear_left_joint_->SetForce(0, velocity_left_force);

        const double velocity_right_force =
            velocity_right_pd_.Update(velocity_right - wheel_velocity.right.radians(), delta);
        rear_right_joint_->SetForce(0, velocity_right_force);
    };

    if (not command_) {
        if (not emergency_stop_) {
            RCLCPP_WARN(node_->get_logger(), "Wait for command...");
            emergency_stop_ = true;
        }

        set_target(0, 0);
    } else {
        const auto now = gazebo_ros::Convert<rclcpp::Time>(info.realTime);
        const auto latency = now - command_->header.stamp;

        if (false && latency > timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Lose control for %fs!", latency.seconds());
            set_target(0, 0);
            emergency_stop_ = true;
        } else {
            set_target(command_->velocity, command_->curvature);
            emergency_stop_ = false;
        }
    }

    last_update_time_ = info.simTime;
}

void AckermannModelPlugin::Reset() { command_ = nullptr; }

GZ_REGISTER_MODEL_PLUGIN(AckermannModelPlugin)

}  // namespace gazebo