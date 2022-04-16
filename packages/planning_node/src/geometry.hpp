#pragma once
#include <cmath>
#include <utility>

inline double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

/* `angled_move` produces (x, y) coordinates after the move from point (0, 0, theta)
 * forward by `dx` and left by `dy`
 */
inline std::pair<double, double> angled_move(double dx, double dy, double theta) {
    return {
        dx * std::cos(deg_to_rad(theta)) - dy * std::sin(deg_to_rad(theta)),
        dx * std::sin(deg_to_rad(theta)) + dy * std::cos(deg_to_rad(theta)),
    };
}

struct Circle {
    double x;
    double y;
    double r;
};

