#pragma once
#include "nlohmann/json.hpp"

#include <cmath>
#include <utility>


using nlohmann::json;

/* `angled_move` produces (x, y) coordinates after the move from point (0, 0, theta)
 * forward by `dx` and left by `dy`
 */
inline std::pair<double, double> angled_move(double dx, double dy, double theta) {
    return {
        dx * std::cos(theta) - dy * std::sin(theta),
        dx * std::sin(theta) + dy * std::cos(theta),
    };
}

struct Circle {
    double x;
    double y;
    double r;

    inline static Circle from_json(json circle) {
        return Circle{
            circle["center"]["x"].get<double>(),
            circle["center"]["y"].get<double>(),
            circle["radius"].get<double>()
        };
    }
};

