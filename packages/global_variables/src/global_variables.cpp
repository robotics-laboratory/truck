#include "global_variables/global_variables.h"

truck::truck() {
    length = 0.505f;
    width = 0.313f;
    height = 0.2f;

    max_angle_to_turn = 30.0f;
    max_distance_to_obstacle_meters = 0.35f;
    circles_count = 3;
}

grid::grid() {
    margin_size_cells = 200;
    margin_size_meters = 10;

    free_cell_color = 0;
    obstacle_cell_color = 99;
    darkest_free_cell_color = 90.0f;

    max_dist_of_interest_meters = 2.0f;
}

float grid::cell_size_meters() {
    return float(margin_size_meters) / margin_size_cells;
}

float grid::max_dist_of_interest_cells() {
    return max_dist_of_interest_meters / cell_size_meters();
}

int convert_coord_to_cell_index(float coord) {
    return coord / grid().cell_size_meters();
}