#pragma once

class truck {
    public:
        float length;
        float width;
        float height;

        float max_angle_to_turn;
        float max_distance_to_obstacle_meters;
        int circles_count;

        truck();
};

class grid {
    public:
        int margin_size_cells;
        int margin_size_meters;

        int free_cell_color;
        int obstacle_cell_color;
        float darkest_free_cell_color;

        float max_dist_of_interest_meters;

        float cell_size_meters();
        float max_dist_of_interest_cells();

        grid();
};

int convert_coord_to_cell_index(float coord);