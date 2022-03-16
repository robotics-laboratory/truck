#include "controller.hpp"

#include <vector>
#include <iostream>
#include <cmath>

int main() {
    Controller ctrl(Parameters{100, 0, 1, 6});
    std::vector<Point> path = {
        {0, 0, 0, 0, 0},
        {0, 5, 5, 0, 0},
        {0, 10, 10, 0, 0},
        {5, 10, 15, 0, 0},
        {10, 10, 20, 0, 0},
        {10, 5, 25, 0, 0},
        {10, 0, 30, 0, 0}
    };
    double T = 40;
    double dt = 0.1;
    Point pos = {0, 0, 0, M_PI / 2, 0};
    double cur_v = 0;
    for (double t = 0; t < T; t += dt) {
        pos.time = t;
        auto opt = ctrl.get_motion(pos, cur_v, path);
        if (!opt) {
            std::cout << "No way(\n";
            break;
        }
        auto [sigma, v] = *opt;
        std::cout << pos.time << " " << pos.x << " " << pos.y << " " << pos.venicle_angle << " " << v << "\n";
        pos.x += std::cos(pos.venicle_angle) * v;
        pos.y += std::sin(pos.venicle_angle) * v;
        pos.wheels_angle = sigma;
        pos.venicle_angle += std::tan(pos.wheels_angle) * v;
        cur_v = v;
    }
}
