#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "model/model.h"

using namespace std;
using namespace truck;

class StaticCollisionChecker {
    private:
        model::Model model_;
        int margin_size_meters = 10;
        int margin_size_cells = 200;
        float cell_size_meters = 0.05f;

    public:
        StaticCollisionChecker();

        float distTransformOnePoint(
            float x,
            float y,
            const std_msgs::msg::Float32MultiArray& dt_grid
        );

        float distTransform(
            float x_origin,
            float y_origin,
            float yaw,
            const std_msgs::msg::Float32MultiArray& dt_grid
        );
};