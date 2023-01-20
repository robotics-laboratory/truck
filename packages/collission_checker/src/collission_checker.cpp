#include "collission_checker/collission_checker.h"

StaticCollisionChecker::StaticCollisionChecker() {}

float StaticCollisionChecker::distTransformOnePoint(
    float x,
    float y,
    const std_msgs::msg::Float32MultiArray& dt_grid
    ) {
    
    int x_cell_index = convert_coord_to_cell_index(x);
    int y_cell_index = convert_coord_to_cell_index(y);

    if ((x_cell_index >= grid().margin_size_cells) ||
        (y_cell_index >= grid().margin_size_cells) ||
        (x_cell_index < 0) ||
        (y_cell_index < 0)) {

        return 0;
    }

    float dist = dt_grid.data[y_cell_index * grid().margin_size_cells + x_cell_index];
    dist = (dist * grid().cell_size_meters()) - (truck().width / 2);

    return dist > 0 ? dist : 0;
}

float StaticCollisionChecker::distTransform(
    float x_origin,
    float y_origin,
    float yaw,
    const std_msgs::msg::Float32MultiArray& dt_grid) {

    vector<float> distances;

    if (truck().circles_count == 1) {
        distances.push_back(distTransformOnePoint(x_origin, y_origin, dt_grid));
    } else {
        for (int i = 0; i < truck().circles_count; i++) {
            float offset =
                -0.5 * (truck().length - truck().width) +
                i * ((truck().length - truck().width) / (truck().circles_count - 1));

            float x = x_origin + offset * cos(yaw);
            float y = y_origin + offset * sin(yaw);

            distances.push_back(distTransformOnePoint(x, y, dt_grid));
        }
    }

    return *min_element(distances.begin(), distances.end());;
}

float StaticCollisionChecker::distTransform(
    float x_origin,
    float y_origin,
    float yaw,
    const std_msgs::msg::Float32MultiArray& dt_grid,
    visualization_msgs::msg::Marker& circles) {

    vector<float> distances;
    geometry_msgs::msg::Point point;

    if (truck().circles_count == 1) {
        point.x = x_origin;
        point.y = y_origin;

        circles.points.push_back(point);
        distances.push_back(distTransformOnePoint(point.x, point.y, dt_grid));
    } else {
        for (int i = 0; i < truck().circles_count; i++) {
            float offset =
                -0.5 * (truck().length - truck().width) +
                i * ((truck().length - truck().width) / (truck().circles_count - 1));

            point.x = x_origin + offset * cos(yaw);
            point.y = y_origin + offset * sin(yaw);

            circles.points.push_back(point);
            distances.push_back(distTransformOnePoint(point.x, point.y, dt_grid));
        }
    }

    return *min_element(distances.begin(), distances.end());;
}