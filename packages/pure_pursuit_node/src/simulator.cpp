#include "simulator.hpp"

#include "controller.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <algorithm>

inline auto odometry_to_pose_stamped(const nav_msgs::msg::Odometry &odm) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odm.header;
    pose.pose = odm.pose.pose;
    return pose;
}

std::vector<nav_msgs::msg::Odometry> pure_pursuit::simulate(nav_msgs::msg::Odometry start,
                                              nav_msgs::msg::Odometry finish, double sim_dt, int controller_freq,
                                              const model::Model &params)
{
    const double eps = 1e-6, PI2_INV = 1 / (M_PI * 2);
    const tf2::Vector3 z_axis(0, 0, 1);
    Controller controller(params);
    auto current_odometry = start;
    std::vector trajectory = {odometry_to_pose_stamped(start), odometry_to_pose_stamped(finish)};
    while (1) {
        auto cmd = controller.get_motion(current_odometry, trajectory, nullptr);
        tf2::Vector3 current_direction;
        tf2::convert(current_odometry.twist.twist.linear, current_direction);
        double current_velocity = sqrt(tf2::tf2Dot(current_direction, current_direction));
        current_direction /= current_velocity;

        tf2::Quaternion current_orientation;
        tf2::convert(current_odometry.pose.pose.orientation, current_orientation);
        double current_yaw = current_orientation.getAngle();

        if (cmd.velocity < eps && current_velocity < eps)
            break;

        for (int i = 0; i < controller_freq; ++i) {
            double angular_velocity = current_velocity * cmd.curvature * PI2_INV;
            double angular_delta = angular_velocity * sim_dt;
            current_yaw += angular_delta;
            tf2::Matrix3x3 rm;
            rm.setRPY(0, 0, angular_delta);
            current_direction = rm * current_direction;
            current_velocity += std::min(cmd.acceleration * sim_dt, cmd.velocity - current_velocity);
        }
    }
    return {};
}