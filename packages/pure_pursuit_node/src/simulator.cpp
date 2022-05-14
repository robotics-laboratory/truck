#include "simulator.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>

#include "controller.hpp"

inline auto odometry_to_pose_stamped(const nav_msgs::msg::Odometry& odm) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odm.header;
    pose.pose = odm.pose.pose;
    return pose;
}

template <class F, class T>
inline void convert_vector(const F& from, T& to) {
    to.setValue(from.x, from.y, from.z);
}

namespace pure_pursuit {

SimulationResult simulate(const nav_msgs::msg::Odometry& start, const nav_msgs::msg::Odometry& finish,
                          uint64_t sim_timeout_ns, uint64_t sim_dt_ns, uint64_t controller_period,
                          const model::Model& params) {
    const double eps = 1e-3, PI2_INV = 1 / (M_PI * 2);
    const tf2::Vector3 z_axis(0, 0, 1);
    const double dt = sim_dt_ns * 1e-9;
    Controller controller(params);
    auto current_odometry = start;
    std::vector trajectory = {odometry_to_pose_stamped(start), odometry_to_pose_stamped(finish)};
    std::vector<nav_msgs::msg::Odometry> ans;
    uint64_t current_time = 0;
    while (current_time < sim_timeout_ns) {
        ans.emplace_back(current_odometry);
        auto controller_result = controller.getMotion(current_odometry, trajectory);
        if (!controller_result)
            return SimulationError{static_cast<int>(SimulationError::CONTROLLER_FAILED) | static_cast<int>(controller_result.error()) << 1};
        tf2::Quaternion current_orientation;
        tf2::convert(current_odometry.pose.pose.orientation, current_orientation);
        double current_yaw = current_orientation.getAngle();

        tf2::Vector3 current_position;
        convert_vector(current_odometry.pose.pose.position, current_position);

        tf2::Vector3 current_direction;
        // tf2::convert(current_odometry.twist.twist.linear, current_direction);
        convert_vector(current_odometry.twist.twist.linear, current_direction);

        double current_velocity = sqrt(tf2::tf2Dot(current_direction, current_direction));
        if (current_velocity > eps) {
            current_direction /= current_velocity;
        } else {
            current_direction = tf2::Matrix3x3(current_orientation) * tf2::Vector3(1, 0, 0);
        }

        for (uint64_t i = 0; i < controller_period && current_time < sim_timeout_ns; ++i) {
            double angular_velocity = current_velocity * controller_result->cmd.curvature * PI2_INV;
            double angular_delta = angular_velocity * dt;
            current_yaw += angular_delta;
            tf2::Matrix3x3 rm;
            rm.setRPY(0, 0, angular_delta);
            auto new_velocity = current_velocity +
                                std::min(controller_result->cmd.acceleration * dt, controller_result->cmd.velocity - current_velocity,
                                         [](auto a, auto b) { return std::abs(a) < std::abs(b); });
            current_position += current_direction * (current_velocity + new_velocity) / 2 * dt;
            current_direction = rm * current_direction;
            current_velocity = new_velocity;
            if (current_velocity < 0) current_velocity = 0;
            current_time += sim_dt_ns;
        }
        if (controller_result->cmd.velocity < eps && current_velocity < eps) return ans;

        current_direction *= current_velocity;
        current_odometry.twist.twist.linear.x = current_direction.x();
        current_odometry.twist.twist.linear.y = current_direction.y();
        current_odometry.pose.pose.position.x = current_position.x();
        current_odometry.pose.pose.position.y = current_position.y();
        current_orientation.setRPY(0, 0, current_yaw);
        tf2::convert(current_orientation, current_odometry.pose.pose.orientation);
        current_odometry.header.stamp =
            rclcpp::Time(current_odometry.header.stamp.nanosec + sim_dt_ns * controller_period);
    }
    return SimulationError::FINISH_POINT_IS_NOT_ARRIVED;
}

};  // namespace pure_pursuit
