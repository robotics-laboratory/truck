#pragma once

#include <cmath>
#include <optional>

#include <geom/arc.hpp>
#include <model/model.hpp>
#include "pure_pursuit/integrator.hpp"

namespace pure_pursuit {

namespace detail {
    template<size_t taylor_powers>
    constexpr std::array<double, taylor_powers> precalcCoefs(bool parity) {
        std::array<double, taylor_powers> coefs{};
        for (size_t i = 0; i < taylor_powers; ++i) {
            if (i % 2 == parity) {
                double coef = 0;
                if (i / 2 % 2 == 1) {
                    coef = -1;
                } else {
                    coef = 1;
                }
                coefs[i] = coef;
            }
        }
        return coefs;
    }
};

class CurvaturePlanner {
private:
    constexpr static size_t taylor_powers = 40;
    constexpr static std::array<double, taylor_powers> sin_coefs = detail::precalcCoefs<taylor_powers>(1);
    constexpr static std::array<double, taylor_powers> cos_coefs = detail::precalcCoefs<taylor_powers>(0);
    detail::TaylorSeriesIntegrator<taylor_powers> integrator;
    double length, max_wheels_angle;

private:
    static double theta(double x) {
        return -std::log(std::cos(x));
    }

public:
    CurvaturePlanner(const model::Model& model, size_t integrator_steps)
        : integrator(
              &theta, -model.max_wheels_angle,
              model.max_wheels_angle, integrator_steps)
        , length(model.truck_length), max_wheels_angle(model.max_wheels_angle) {}

    struct Plan {
        geom::Arc final_arc;
        double sigma_delta;
    };
    std::optional<Plan> calculatePlan(
        geom::Vec2d start, geom::Vec2d target, double start_yaw, double start_sigma,
        double linear_velocity, double angular_velocity);
};

}  // namespace pure_pursuit
