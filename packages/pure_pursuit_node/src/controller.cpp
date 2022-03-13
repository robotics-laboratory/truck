#include "controller.hpp"

#include <algorithm>
#include <cmath>

using pure_pursuit_msgs::msg::Command;
using namespace geometry_msgs::msg;

namespace {

double quaternoin_to_flat_angle(const Quaternion &q) {
    return std::copysign(2 * std::acos(q.w), q.z);
}

};

namespace pure_pursuit {

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
    friend Vector operator+(Vector a, const Vector &b) {
        a += b;
        return a;
    }
    friend Vector operator-(Vector a, const Vector &b) {
        a -= b;
        return a;
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
    p = p.rotate(-quaternoin_to_flat_angle(odometry.pose.pose.orientation) + M_PI / 2);
    bool sign = 0;
    if (p.x < 0) {
        p.x = -p.x;
        sign = 1;
    }
    double r = dot(p, p) / (2 * p.x);
    double dist;
    if (r < 1e4)
        dist = r * (p - Vector{r, 0}).angle();
    else
        dist = p.y;
    auto new_velocity = params.max_velocity;
    double time = dist / new_velocity;
    // auto new_velocity = dist / time; // TODO: acceleration
    // if (new_velocity > params.max_velocity)
    //     return std::nullopt;
    auto target_angle = (p - Vector{r, 0}).angle() - M_PI / 2;
    auto angular_delta = target_angle - M_PI / 2;

    Command result;

    result.velocity = odometry.twist.twist;
    double norm = Vector{odometry.twist.twist.linear.x, odometry.twist.twist.linear.y}.len();
    result.velocity.linear.x *= new_velocity / norm;
    result.velocity.linear.y *= new_velocity / norm;
    result.velocity.angular.z = angular_delta * time;
    if (sign)
        result.velocity.angular.z *= -1;

    return result;
}

};
