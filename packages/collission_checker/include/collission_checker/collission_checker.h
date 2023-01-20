#include <cmath>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "global_variables/global_variables.h"

using namespace std;

#pragma once

class StaticCollisionChecker {
    private:
        float distTransformOnePoint(
            float x,
            float y,
            const std_msgs::msg::Float32MultiArray& dt_grid
        );

    public:
        StaticCollisionChecker();

        float distTransform(
            float x_origin,
            float y_origin,
            float yaw,
            const std_msgs::msg::Float32MultiArray& dt_grid
        );

        float distTransform(
            float x_origin,
            float y_origin,
            float yaw,
            const std_msgs::msg::Float32MultiArray& dt_grid,
            visualization_msgs::msg::Marker& circles
        );
};