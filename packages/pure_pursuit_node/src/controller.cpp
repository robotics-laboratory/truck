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
    auto new_velocity = params.max_velocity;
    double time = dist / new_velocity;

    Command result;

    Vector direction{odometry.twist.twist.linear.x, odometry.twist.twist.linear.y};
    direction *= new_velocity / direction.len();

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
