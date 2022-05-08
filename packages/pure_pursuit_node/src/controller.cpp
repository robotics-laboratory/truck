#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using pure_pursuit_msgs::msg::Command;
using namespace geometry_msgs::msg;
using visualization_msgs::msg::Marker;

namespace {

double quaternoin_to_flat_angle(const Quaternion& q) {
    return std::copysign(2 * std::acos(q.w), q.z);
}

template <class P1, class P2>
double distance(const P1& a, const P2& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

struct Vector {
    double x, y;
    Vector& operator+=(const Vector& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vector& operator-=(const Vector& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    Vector& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }
    Vector& operator/=(double s) {
        x /= s;
        y /= s;
        return *this;
    }
    friend Vector operator+(Vector a, const Vector& b) {
        a += b;
        return a;
    }
    friend Vector operator-(Vector a, const Vector& b) {
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
    friend double dot(const Vector& a, const Vector& b) { return a.x * b.x + a.y * b.y; }
    friend double cross(const Vector& a, const Vector& b) { return a.x * b.y - a.y * b.x; }
    Vector rotate(double angle) const {
        double sn = std::sin(angle);
        double cs = std::cos(angle);
        return {x * cs - y * sn, x * sn + y * cs};
    }
    double angle() const { return std::atan2(y, x); }
    double len() const { return std::sqrt(x * x + y * y); }
};

inline double ros_time_to_seconds(const rclcpp::Time& t) { return t.seconds(); }

inline double get_arc_length(double r, double angle) { return r * angle; }

};  // namespace

namespace pure_pursuit {

ControllerResult Controller::get_motion(const nav_msgs::msg::Odometry& odometry,
                                        const std::vector<PoseStamped>& path,
                                        VisualInfo* visual_info) {
    if (visual_info) {
        Marker start;
        start.type = Marker::SPHERE;
        start.pose = odometry.pose.pose;
        start.color.a = 1;
        start.color.r = 1;
        start.scale.x = 0.1;
        start.scale.y = 0.1;
        start.scale.z = 0.1;
        start.id = visual_info->arc.markers.size();
        visual_info->arc.markers.emplace_back(start);
    }
    auto& position = odometry.pose.pose.position;
    auto it = std::find_if(path.rbegin(), path.rend(), [&position, this](const PoseStamped& p) {
        return distance(p.pose.position, position) <= model.lookahead_distance;
    });
    if (it == path.rend()) return ControllerError::UNREACHEABLE_TRAJECTORY;
    if (visual_info) {
        for (auto& x : path) {
            Marker target;
            target.type = Marker::SPHERE;
            target.pose = x.pose;
            target.color.a = 1;
            target.color.b = 1;
            target.scale.x = 0.1;
            target.scale.y = 0.1;
            target.scale.z = 0.1;
            target.id = visual_info->arc.markers.size();
            visual_info->arc.markers.emplace_back(target);
        }
    }
    Vector p0{position.x, position.y};
    Vector p{it->pose.position.x, it->pose.position.y};
    p -= p0;
    p = p.rotate(-quaternoin_to_flat_angle(odometry.pose.pose.orientation));
    bool sign = std::signbit(p.y);
    p.y = std::abs(p.y);

    double y_tolerance = std::max(abs(p.x), 1.0) * 0.01;
    bool stright_trajectory = (p.y < y_tolerance);

    Vector center;
    double target_angle;
    auto& r = center.y;
    if (stright_trajectory) {
        r = std::numeric_limits<double>::infinity();
        target_angle = 0;
    } else {
        r = dot(p, p) / (2 * p.y);
        target_angle = M_PI / 2 + (p - center).angle();
        if (target_angle < 0) target_angle += M_PI * 2;
    }

    if (visual_info) {
        constexpr int points = 50;
        double original_yaw = quaternoin_to_flat_angle(odometry.pose.pose.orientation);
        for (int i = 1; i < points; ++i) {
            Vector pos;
            if (stright_trajectory) {
                pos = p * i / points;
            } else {
                double angle = -M_PI / 2 + target_angle / points * i;
                pos = center + Vector{cos(angle) * r, sin(angle) * r};
            }
            if (sign) pos.y = -pos.y;
            pos = pos.rotate(original_yaw);
            pos += p0;
            Marker mark;
            mark.type = Marker::SPHERE;
            mark.pose.position.x = pos.x;
            mark.pose.position.y = pos.y;
            mark.color.a = 1;
            mark.color.g = 1;
            mark.scale.x = 0.05;
            mark.scale.y = 0.05;
            mark.scale.z = 0.05;
            mark.id = visual_info->arc.markers.size();
            visual_info->arc.markers.emplace_back(mark);
        }
    }

    double dist;
    if (stright_trajectory) {
        if (p.x < 0) {
            return ControllerError::IMPOSSIBLE_ARC;
        }
        dist = p.x;
    } else {
        dist = get_arc_length(r, target_angle);
    }

    Vector velocity_vector{odometry.twist.twist.linear.x, odometry.twist.twist.linear.y};
    double current_velocity = velocity_vector.len();
    auto path_time_by_accel = [&current_velocity, &dist](double final_velocity, double accel) {
        double accel_time = std::max((final_velocity - current_velocity) / accel, 0.0);
        double accel_path = accel * std::pow(accel_time, 2) / 2 + current_velocity * accel_time;
        if (accel_path > dist) {
            double d = std::pow(current_velocity, 2) + 2 * accel * dist;
            return (copysign(sqrt(d), accel) - current_velocity) / accel;
        } else {
            return accel_time + (dist - accel_path) / final_velocity;
        }
    };

    double accel, target_velocity;

    double required_time =
        ros_time_to_seconds(it->header.stamp) - ros_time_to_seconds(odometry.header.stamp);
    if (required_time < 0) required_time = 0;
    double required_velocity = 0;
    if (it != path.rbegin()) {
        required_velocity =
            distance(it->pose.position, prev(it)->pose.position) /
            (ros_time_to_seconds(prev(it)->header.stamp) - ros_time_to_seconds(it->header.stamp));
    }
    required_velocity = std::min(required_velocity, model.max_velocity);

    if (it != path.rbegin()) {  // Optimize time firstly
        double min_posible_time = path_time_by_accel(model.max_velocity, model.max_acceleration);
        if (required_time < min_posible_time) {
            target_velocity = model.max_velocity;
            accel = model.max_acceleration;
        } else {
            double accel_constraint = required_velocity >= current_velocity
                                          ? model.max_acceleration
                                          : -model.max_decceleration;
            if (path_time_by_accel(required_velocity, accel_constraint) < required_time) {
                target_velocity = required_velocity;
                double l = 0, r = accel_constraint;
                if (accel_constraint < 0) {
                    std::swap(l, r);
                }
                for (int i = 0; i < 20; ++i) {  // More robust than epsilon-based binary search
                    double m = (l + r) / 2;
                    if (path_time_by_accel(required_velocity, m) > required_time)
                        l = m;
                    else
                        r = m;
                }
                accel = r;
            } else {
                accel = accel_constraint;
                target_velocity = accel < 0 ? 0 : model.max_velocity;
            }
        }
    } else {  // Optimize velocity firstly
        double velocity_delta = (required_velocity - current_velocity);
        double min_time = velocity_delta / (velocity_delta < 0 ? -model.max_decceleration
                                                               : model.max_acceleration);
        double min_dist = (required_velocity + current_velocity) * min_time / 2;

        if (min_dist > dist) {  // Could not arrive rquired velocity
            target_velocity = required_velocity;
            accel = (velocity_delta < 0 ? -model.max_decceleration : model.max_acceleration);
        } else {
            /*
                Current velocity = V1.
                Will move with constant acceleration a until velocity = Vm, then with constant
               decceleration d until velocity = V2.

                S = (Vm + V1) * (Vm - V1) / (2 * a) + (Vm + V2) * (Vm - V2) / (2 * d)
                S - (Vm + V1) * (Vm - V1) / (2 * a) = (Vm + V2) * (Vm - V2) / (2 * d)
                (2 * S * a - (Vm + V1) * (Vm - V1)) / (2 * a) = (Vm + V2) * (Vm - V2) / (2 * d)
                2d = (Vm + V2) * (Vm - V2) * (2 * a) / (2 * S * a - (Vm + V1) * (Vm - V1))
                d = (Vm + V2) * (Vm - V2) * (2 * a) / 2(2 * S * a - (Vm + V1) * (Vm - V1))

                t = (Vm - V1) / a + (Vm - V2) / d
                t = (Vm - V1) / a + 2 * (2 * S * a - (Vm + V1) * (Vm - V1)) * (Vm - V2) / ((Vm + V2)
                    * (Vm - V2) * (2 * a))

                t = (Vm - V1) / a + 2 * (2 * S * a - (Vm + V1) * (Vm - V1)) / ((Vm + V2) * (2 * a))

                t = (Vm - V1) / a + 2 * (2 * S * a) / ((Vm + V2) * (2 * a)) - 2 * ((Vm + V1) * (Vm -
               V1)) / ((Vm + V2) * (2 * a))

                t = (Vm - V1) / a + 2 * S / (Vm + V2) - ((Vm + V1) * (Vm - V1)) / ((Vm + V2) * a)

                t - 2S / (Vm + V2) = (Vm - V1) / a - ((Vm + V1) * (Vm - V1)) / ((Vm + V2) * a)

                t - 2S / (Vm + V2) = ((Vm - V1) - (Vm + V1) * (Vm - V1) / (Vm + V2)) / a

                a = ((Vm - V1) - (Vm + V1) * (Vm - V1) / (Vm + V2)) / (t - 2S / (Vm + V2))
            */
            auto d_by_a = [dist](double V1, double V2, double Vm, double a) {
                return (Vm + V2) * (Vm - V2) * (2 * a) /
                       (2 * (2 * dist * a - (Vm + V1) * (Vm - V1)));
            };
            auto a_by_t = [dist](double V1, double V2, double Vm, double t) {
                return ((Vm - V1) - (Vm + V1) * (Vm - V1) / (Vm + V2)) / (t - 2 * dist / (Vm + V2));
            };
            double Vl = 0;
            double Vr = model.max_velocity;
            double Vm, t1, t2;
            for (int i = 0; i < 20; ++i) {  // More robust than epsilon-based binary search
                Vm = (Vl + Vr) / 2;
                double max_abs_a =
                    (Vm < current_velocity ? -model.max_decceleration : model.max_acceleration);
                double max_abs_d =
                    (Vm < required_velocity ? -model.max_acceleration : model.max_decceleration);
                double d_for_max_a = d_by_a(current_velocity, required_velocity, Vm, max_abs_a);
                double a_for_max_d = d_by_a(required_velocity, current_velocity, Vm, max_abs_d);
                if (!std::isfinite(a_for_max_d) || a_for_max_d < -model.max_decceleration ||
                    a_for_max_d > model.max_acceleration || !std::isfinite(d_for_max_a) ||
                    d_for_max_a < -model.max_decceleration ||
                    d_for_max_a > model.max_acceleration) {
                    if (Vm < current_velocity)
                        Vl = Vm;
                    else
                        Vr = Vm;
                    continue;
                }
                t1 = (Vm - current_velocity) / max_abs_a + (Vm - required_velocity) / d_for_max_a;
                t2 = (Vm - current_velocity) / a_for_max_d + (Vm - required_velocity) / max_abs_d;
                if (required_time < (t1 + t2) / 2)
                    Vl = Vm;
                else
                    Vr = Vm;
            }
            target_velocity = Vm;
            if (std::abs(current_velocity - required_velocity) < 1e-3) {
                accel = model.max_acceleration;
            } else {
                accel = a_by_t(current_velocity, required_velocity, target_velocity, std::max(required_time, std::min(t1, t2)));
                if (!std::isfinite(accel) || accel < -model.max_decceleration - 1e-3 ||
                    accel > model.max_acceleration + 1e-3 ||
                    (accel < 0 && target_velocity > current_velocity) ||
                    (accel > 0 && target_velocity < current_velocity)) {
                    return ControllerError::BROKEN_FORMULA_FOR_ACCELERATION;
                }
            }
            if (std::abs(target_velocity - current_velocity) < 1e-3)
                accel = -d_by_a(current_velocity, required_velocity, current_velocity, accel);
        }
    }

    Command result;

    result.acceleration = accel;
    result.velocity = target_velocity;
    result.curvature = 1 / r;

    if (sign) result.curvature *= -1;

    return result;
}

};  // namespace pure_pursuit
