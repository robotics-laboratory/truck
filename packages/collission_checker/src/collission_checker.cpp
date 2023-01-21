#include "collission_checker/collission_checker.h"

StaticCollisionChecker::StaticCollisionChecker()
    : model_("/truck/packages/model/config/model.yaml") {}

float StaticCollisionChecker::distTransformOnePoint(
    float x,
    float y,
    const std_msgs::msg::Float32MultiArray& dt_grid
    ) {

    int x_cell_index = x / (float(margin_size_meters) / margin_size_cells);
    int y_cell_index = y / (float(margin_size_meters) / margin_size_cells);

    if ((x_cell_index >= margin_size_cells) ||
        (y_cell_index >= margin_size_cells) ||
        (x_cell_index < 0) ||
        (y_cell_index < 0)) {

        return 0;
    }

    float dist = dt_grid.data[y_cell_index * margin_size_cells + x_cell_index];
    dist = (dist * cell_size_meters) - (model_.truckShape().width / 2);

    return dist > 0 ? dist : 0;
}

float StaticCollisionChecker::distTransform(
    float x_origin,
    float y_origin,
    float yaw,
    const std_msgs::msg::Float32MultiArray& dt_grid) {
    
    float min_distance = numeric_limits<float>::max();
    vector<geometry_msgs::msg::Point> points = model_.getCircles(x_origin, y_origin, yaw);

    for (const auto& p : points) {
        float tmp = distTransformOnePoint(p.x, p.y, dt_grid);
        min_distance = tmp < min_distance ? tmp : min_distance;
    }

    return min_distance;
}