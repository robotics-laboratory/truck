#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace pure_pursuit {

using planning_interfaces::msg::Point;
using pure_pursuit_msgs::msg::Command;

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
    friend double operator*(const Vector &a, const Vector &b) {
        return a.x * b.x + a.y * b.y;
    }
    friend double operator^(const Vector &a, const Vector &b) {
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
      const pure_pursuit_msgs::msg::State &state
    , const std::vector<Point> &path
) {
    auto &position = state.position;
    auto it = std::find_if(path.rbegin(), path.rend(), [&position, this](const Point &p) {
        return distance(p, position) <= params.lookahead_distance;
    });
    if (it == path.rend())
        return std::nullopt;
    Vector p0{position.x, position.y};
    Vector p{it->x, it->y};
    // Vector intersection;
    // if () {

    // }
    p -= p0;
    p = p.rotate(-state.yaw + M_PI / 2);
    bool sign = 0;
    if (p.x < 0) {
        p.x = -p.x;
        sign = 1;
    }
    double r = p * p / (2 * p.x);
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

    result.velocity = state.velocity;
    double norm = Vector{state.velocity.linear.x, state.velocity.linear.y}.len();
    result.velocity.linear.x *= new_velocity / norm;
    result.velocity.linear.y *= new_velocity / norm;
    result.velocity.angular.z = angular_delta * time;
    if (sign)
        result.velocity.angular.z *= -1;

    return result;
}

};
