#include "controller.hpp"

#include <algorithm>
#include <cmath>

using pure_pursuit_msgs::msg::Command;
using namespace geometry_msgs::msg;

namespace {

double quaternoin_to_flat_angle(const Quaternion &q) {
    return std::copysign(2 * std::acos(q.w), q.z);
}

template<class P1, class P2>
double distance(const P1 &a, const P2 &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

struct Vector {
    double x, y;
    Vector &operator+=(const Vector &other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vector &operator-=(const Vector &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    Vector &operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }
    Vector &operator/=(double s) {
        x /= s;
        y /= s;
        return *this;
    }
    friend Vector operator+(Vector a, const Vector &b) {
        a += b;
        return a;
    }
    friend Vector operator-(Vector a, const Vector &b) {
        a -= b;
        return a;
    }
    friend Vector operator*(Vector v, double s) {
        v *= s;
        return v;
    }
    friend Vector operator/(Vector v, double s) {
        v /= s;
        return v;
    }
    friend double dot(const Vector &a, const Vector &b) {
        return a.x * b.x + a.y * b.y;
    }
    friend double cross(const Vector &a, const Vector &b) {
        return a.x * b.y - a.y * b.x;
    }
    Vector rotate(double angle) const {
        double sn = std::sin(angle);
        double cs = std::cos(angle);
        return {x * cs - y * sn, x * sn + y * cs};
    }
    double angle() const {
        return std::atan2(y, x);
    }
    double len() const {
        return std::sqrt(x * x + y * y);
    }
};

inline double ros_time_to_seconds(const rclcpp::Time &t) {
    return t.seconds() + t.nanoseconds() * 1e-9;
}

};

namespace pure_pursuit {

std::optional<Command> Controller::get_motion(
      const nav_msgs::msg::Odometry &odometry
    , const std::vector<PoseStamped> &path
) {
    auto &position = odometry.pose.pose.position;
    auto it = std::find_if(path.rbegin(), path.rend(), [&position, this](const PoseStamped &p) {
        return distance(p.pose.position, position) <= params.lookahead_distance;
    });
    if (it == path.rend())
        return std::nullopt;
    Vector p0{position.x, position.y};
    Vector p{it->pose.position.x, it->pose.position.y};
    p -= p0;
    p = p.rotate(-quaternoin_to_flat_angle(odometry.pose.pose.orientation));
    bool sign = std::signbit(p.y);
    p.y = std::abs(p.y);
    double r = dot(p, p) / (2 * p.y);
    Vector center{0, r};
    auto target_angle = (p - center).angle();
    double dist;
    if (r < 1e4) {
        dist = r * target_angle;
    } else {
        if (p.x < 0) {
            return std::nullopt;
        }
        dist = p.x;
    }

    Vector velocity_vector{odometry.twist.twist.linear.x, odometry.twist.twist.linear.y};
    double current_velocity = velocity_vector.len();
    auto path_time_by_accel = [&current_velocity, &dist](double final_velocity, double accel) {
        double accel_time = (final_velocity - current_velocity) / accel;
        double accel_path = accel * std::pow(accel_time, 2) / 2 + current_velocity * accel_time;
        if (accel_path > dist) {
            double d = std::pow(current_velocity, 2) + 2 * accel * dist;
            return (copysign(sqrt(d), -accel) - current_velocity) / accel;
        } else {
            return accel_time + (dist - accel_path) / final_velocity;
        }
    };

    double accel, target_velocity;

    double required_time = ros_time_to_seconds(it->header.stamp) - ros_time_to_seconds(odometry.header.stamp);
    double min_posible_time = path_time_by_accel(params.max_velocity, params.max_accel);
    if (required_time < min_posible_time) {
        target_velocity = params.max_velocity;
        accel = params.max_accel;
    } else {
        double required_velocity = 0;
        if (it != path.rbegin()) {
            required_velocity = distance(it->pose.position, prev(it)->pose.position) /
                (ros_time_to_seconds(prev(it)->header.stamp) - ros_time_to_seconds(it->header.stamp));
        }
        required_velocity = std::min(required_velocity, params.max_velocity);
        double accel_constraint = copysign(params.max_accel, required_velocity - current_velocity);
        if (path_time_by_accel(required_velocity, accel_constraint) < required_time) {
            target_velocity = required_velocity;
            double l = 0, r = accel_constraint;
            if (accel_constraint < 0) {
                std::swap(l, r);
            }
            for (int i = 0; i < 20; ++i) { // More robust than epsilon-based binary search
                double m = (l + r) / 2;
                if (path_time_by_accel(required_velocity, m) > required_time)
                    l = m;
                else
                    r = m;
            }
            accel = r;
        } else {
            accel = accel_constraint;
            target_velocity = accel < 0 ? 0 : params.max_accel;
        }
    }

    double time = dist / target_velocity;

    Command result;

    result.acceleration = accel;

    auto direction = velocity_vector * target_velocity / velocity_vector.len();

    result.velocity.linear.x = direction.x;
    result.velocity.linear.y = direction.y;
    result.velocity.linear.z = 0;

    result.velocity.angular.x = 0;
    result.velocity.angular.y = 0;
    result.velocity.angular.z = -target_angle / time;

    if (sign)
        result.velocity.angular.z *= -1;

    return result;
}

};
