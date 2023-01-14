#include <cmath>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "include/a_star.h"
#include "include/global_variables.h"

using namespace std;

Vec::Vec() {}

Vec::Vec(float start_x, float start_y, float end_x, float end_y) {
    coord_x = end_x - start_x;
    coord_y = end_y - start_y;
}

float Vec::len() {
    return sqrt(pow(coord_x, 2) + pow(coord_y, 2));
}

float dot_product(const Vec& v1, const Vec& v2) {
    return v1.coord_x * v2.coord_x + v1.coord_y * v2.coord_y;
}

float from_radians_to_degrees(float res) {
    return res * (180 / M_PI);
}

float find_angle(Vec& v1, Vec& v2) {
    return from_radians_to_degrees(acos((dot_product(v1, v2)) / (v1.len() * v2.len())));
}

Vertex::Vertex() {}

Vertex::Vertex (int x_index_, int y_index_) {
    x_index = x_index_;
    y_index = y_index_;
}

void Vertex::add_neighbors(
    vector<vector<Vertex>>& vertices
    ) {
    int n = grid().margin_size_cells;

    // neighbor type: yellow
    if (x_index > 0)
        neighbors.push_back(&vertices[x_index - 1][y_index]);

    if (x_index < n - 1)
        neighbors.push_back(&vertices[x_index + 1][y_index]);

    if (y_index > 0)
        neighbors.push_back(&vertices[x_index][y_index - 1]);

    if (y_index < n - 1)
        neighbors.push_back(&vertices[x_index][y_index + 1]);

    // neighbor type: pink
    if ((x_index > 0) && (y_index > 0))
        neighbors.push_back(&vertices[x_index - 1][y_index - 1]);

    if ((x_index < n - 1) && (y_index < n - 1))
        neighbors.push_back(&vertices[x_index + 1][y_index + 1]);

    if ((x_index > 0) && (y_index < n - 1))
        neighbors.push_back(&vertices[x_index - 1][y_index + 1]);

    if ((x_index < n - 1) && (y_index > 0))
        neighbors.push_back(&vertices[x_index + 1][y_index - 1]);

    // neighbor type: green
    if ((x_index > 1) && (y_index > 0))
        neighbors.push_back(&vertices[x_index - 2][y_index - 1]);

    if ((x_index > 0) && (y_index > 1))
        neighbors.push_back(&vertices[x_index - 1][y_index - 2]);

    if ((x_index < n - 2) && (y_index < n - 1))
        neighbors.push_back(&vertices[x_index + 2][y_index + 1]);

    if ((x_index < n - 1) && (y_index < n - 2))
        neighbors.push_back(&vertices[x_index + 1][y_index + 2]);

    if ((x_index > 0) && (y_index < n - 2))
        neighbors.push_back(&vertices[x_index - 1][y_index + 2]);

    if ((x_index > 1) && (y_index < n - 1))
        neighbors.push_back(&vertices[x_index - 2][y_index + 1]);

    if ((x_index < n - 1) && (y_index > 1))
        neighbors.push_back(&vertices[x_index + 1][y_index - 2]);

    if ((x_index < n - 2) && (y_index > 0))
        neighbors.push_back(&vertices[x_index + 2][y_index - 1]);
}

float neighbors_dist(
    const Vertex* vertex_1,
    const Vertex* vertex_2) {
    if ((vertex_1->x_index == vertex_2->x_index) ||
        (vertex_1->y_index == vertex_2->y_index)) {
        // neighbor type: yellow
        return 1.0;
    } else if ((abs(vertex_1->x_index - vertex_2->x_index) == 1) &&
               (abs(vertex_1->y_index - vertex_2->y_index) == 1)) {
        // neighbor type: pink
        return sqrt(2);
    } else {
        // neighbor type: green
        return sqrt(5);
    }
}

float heuristic(
    const Vertex* vertex_1,
    const Vertex* vertex_2) {
    return sqrt(
        pow(vertex_1->x_index - vertex_2->x_index, 2) +
        pow(vertex_1->y_index - vertex_2->y_index, 2)
    );
}

void initialize_graph(
    vector<vector<Vertex>>& vertices,
    const nav_msgs::msg::OccupancyGrid& oc_grid,
    rclcpp::Node& node
    ) {
    for (int i = 0; i < grid().margin_size_cells; i++) {
        for (int j = 0; j < grid().margin_size_cells; j++) {
            vertices[i][j].x_index = i;
            vertices[i][j].y_index = j;
            vertices[i][j].add_neighbors(vertices);

            try {
                if (oc_grid.data.at(j * grid().margin_size_cells + i) == grid().obstacle_cell_color) {
                    vertices[i][j].obstacle = 1;
                } else {
                    vertices[i][j].obstacle = 0;
                }
            } catch (const std::out_of_range& e) {
                RCLCPP_INFO(node.get_logger(), "ERROR: occupancy grid is not found");
            }
        }
    }
}

void prepare_point(geometry_msgs::msg::PoseStamped& p, int x, int y, rclcpp::Node& node) {
    p.header.frame_id = "odom_ekf";
    p.header.stamp = node.now();
    p.pose.position.x = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * x);
    p.pose.position.y = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * y);
    p.pose.position.z = 0.005;
}

void generate_optimal_path(
    Vertex* current_vertex,
    vector<geometry_msgs::msg::PoseStamped>& optimal_points,
    rclcpp::Node& node
    ) {
    geometry_msgs::msg::PoseStamped p;
    prepare_point(p, current_vertex->x_index, current_vertex->y_index, node);

    optimal_points.push_back(p);

    // Go to previous vertex
    while (current_vertex->prev_vertex != nullptr) {
        current_vertex = current_vertex->prev_vertex;

        geometry_msgs::msg::PoseStamped p;
        prepare_point(p, current_vertex->x_index, current_vertex->y_index, node);

        optimal_points.push_back(p);
    }
}

float get_distance_transform(
    int x,
    int y,
    const std_msgs::msg::Float32MultiArray& dist_trans_arr
    ) {
    return dist_trans_arr.data[y * grid().margin_size_cells + x] * grid().cell_size_meters();
}

bool far_from_sides(const Vertex* v, const std_msgs::msg::Float32MultiArray& dist_trans_arr) {
    return get_distance_transform(v->x_index, v->y_index, dist_trans_arr) >= truck().max_distance_to_obstacle_meters;
}

bool kinematically_feasible(const Vertex* neighbor, const Vertex* current_vertex, Vec& cur_yaw) {
    Vec future_yaw(
        current_vertex->x_index,
        current_vertex->y_index,
        neighbor->x_index,
        neighbor->y_index
    );

    return find_angle(cur_yaw, future_yaw) <= truck().max_angle_to_turn;
}

int a_star(
    vector<geometry_msgs::msg::PoseStamped>& optimal_points,
    const int start_x_index, const int start_y_index,
    const int end_x_index, const int end_y_index,
    rclcpp::Node& node,
    const nav_msgs::msg::OccupancyGrid& oc_grid,
    const std_msgs::msg::Float32MultiArray& dist_trans_arr,
    const Vec& start_yaw
    ) {
    Vec current_yaw;

    bool first_time = true;

    vector<vector<Vertex>> vertices =
        vector<vector<Vertex>>(
            grid().margin_size_cells,
            vector<Vertex>(grid().margin_size_cells)
        );
    
    initialize_graph(vertices, oc_grid, node);

    Vertex* start_vertex = &vertices[start_x_index][start_y_index];
    Vertex* end_vertex = &vertices[end_x_index][end_y_index];

    if ((start_vertex->obstacle == 1) || (end_vertex->obstacle == 1)) {
        return 2;
    }

    set<Vertex*> open_set, closed_set;
    open_set.insert(start_vertex);

    Vertex* current_vertex;

    while (open_set.size() > 0) {
        vector<Vertex*> open_set_vector;
        copy(open_set.begin(), open_set.end(), back_inserter(open_set_vector));
        
        auto iter_min_f = open_set_vector.begin();

        for (auto it = open_set_vector.begin(); it != open_set_vector.end(); it++) {
            if ((*it)->f < (*iter_min_f)->f) {
                iter_min_f = it;
            }
        }

        current_vertex = *iter_min_f;

        if (first_time == true) {
            first_time = false;
            current_yaw = start_yaw;
        } else {
            current_yaw = Vec(
                current_vertex->prev_vertex->x_index,
                current_vertex->prev_vertex->y_index,
                current_vertex->x_index,
                current_vertex->y_index
            );
        }

        // Path found
        if ((current_vertex->x_index == end_x_index) &&
            (current_vertex->y_index == end_y_index)) {
            generate_optimal_path(current_vertex, optimal_points, node);
            return 1;
        }

        closed_set.insert(current_vertex);
        open_set.erase(current_vertex);

        for (auto neighbor : current_vertex->neighbors) {
            bool neighbor_changed = false;

            // Checking neighbors to satisfy several conditions
            if ((closed_set.find(neighbor) == closed_set.end()) &&                  // Not in closed set
                (neighbor->obstacle == 0) &&                                        // Not obstacle
                (kinematically_feasible(neighbor, current_vertex, current_yaw)) &&  // Smooth turns
                (far_from_sides(neighbor, dist_trans_arr))                          // Far from obstacles
                ) {
                                        
                float temp_g =
                    current_vertex->g +
                    neighbors_dist(current_vertex, neighbor);

                if (open_set.find(neighbor) != open_set.end()) {
                    if (temp_g < neighbor->g) {
                        neighbor_changed = true;
                        neighbor->g = temp_g;
                    }
                } else {
                    neighbor_changed = true;
                    neighbor->g = temp_g;

                    open_set.insert(neighbor);
                }
            }

            if (neighbor_changed == true) {
                neighbor->h = heuristic(neighbor, end_vertex);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->prev_vertex = current_vertex;
            }
        }
    }

    return 3;
}