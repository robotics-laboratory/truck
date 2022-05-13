#include <cmath>

#include "speed_planner.hpp"

inline double pathTimeByAccel(double required_dist, double current_velocity, double final_velocity, double accel) noexcept {
    double accel_time = std::max((final_velocity - current_velocity) / accel, 0.0);
    double accel_path = accel * std::pow(accel_time, 2) / 2 + current_velocity * accel_time;
    if (accel_path > required_dist) {
        double d = std::pow(current_velocity, 2) + 2 * accel * required_dist;
        return (copysign(sqrt(d), accel) - current_velocity) / accel;
    } else {
        return accel_time + (required_dist - accel_path) / final_velocity;
    }
};

namespace pure_pursuit {

MovingPlan getPlanWithTimePrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model) {
    MovingPlan result;
    double min_posible_time = pathTimeByAccel(required_dist, current_velocity, model.max_velocity, model.max_acceleration);
    if (required_time < min_posible_time) {
        result.velocity = model.max_velocity;
        result.acceleration = model.max_acceleration;
    } else {
        double accel_constraint = required_velocity >= current_velocity
                                      ? model.max_acceleration
                                      : -model.max_decceleration;
        if (pathTimeByAccel(required_dist, current_velocity, required_velocity, accel_constraint) < required_time) {
            result.velocity = required_velocity;
            double l = 0, r = accel_constraint;
            if (accel_constraint < 0) {
                std::swap(l, r);
            }
            for (int i = 0; i < 20; ++i) {  // More robust than epsilon-based binary search
                double m = (l + r) / 2;
                if (pathTimeByAccel(required_dist, current_velocity, required_velocity, m) > required_time)
                    l = m;
                else
                    r = m;
            }
            result.acceleration = r;
        } else {
            result.acceleration = accel_constraint;
            result.velocity = result.acceleration < 0 ? 0 : model.max_velocity;
        }
    }
    return result;
}

MovingPlan getPlanWithVelocityPrior(double required_dist, double required_time, double required_velocity, double current_velocity, const model::Model& model) {
    MovingPlan result;
    double velocity_delta = (required_velocity - current_velocity);
    double min_time = velocity_delta / (velocity_delta < 0 ? -model.max_decceleration
                                                           : model.max_acceleration);
    double min_dist = (required_velocity + current_velocity) * min_time / 2;

    if (min_dist > required_dist) {  // Could not arrive required velocity
        result.velocity = required_velocity;
        result.acceleration = (velocity_delta < 0 ? -model.max_decceleration : model.max_acceleration);
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
        auto d_by_a = [required_dist](double V1, double V2, double Vm, double a) {
            return (Vm + V2) * (Vm - V2) * (2 * a) /
                   (2 * (2 * required_dist * a - (Vm + V1) * (Vm - V1)));
        };
        auto a_by_t = [required_dist](double V1, double V2, double Vm, double t) {
            return ((Vm - V1) - (Vm + V1) * (Vm - V1) / (Vm + V2)) / (t - 2 * required_dist / (Vm + V2));
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
        result.velocity = Vm;
        if (std::abs(current_velocity - required_velocity) < 1e-3) {
            result.acceleration = model.max_acceleration;
        } else {
            result.acceleration = a_by_t(current_velocity, required_velocity, result.velocity, std::max(required_time, std::min(t1, t2)));
        }
        if (std::abs(result.velocity - current_velocity) < 1e-3)
            result.acceleration = -d_by_a(current_velocity, required_velocity, current_velocity, result.acceleration);
    }
    return result;
}

}
