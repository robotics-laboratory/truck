#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

using namespace std;

class Vec {
    public:
        float coord_x;
        float coord_y;

        Vec();
        Vec(float start_x, float start_y, float end_x, float end_y);

        float len();
};

class Vertex {
    public:
        int x_index;
        int y_index;
        int obstacle = 0;
        float g = 0.0f;
        float h = 0.0f;
        float f = 0.0f;
        vector<Vertex*> neighbors;
        Vertex* prev_vertex = nullptr;

        Vertex();
        Vertex(int x_index_, int y_index_);

        void add_neighbors(vector<vector<Vertex>>& vertices);      
};

int a_star(
    vector<geometry_msgs::msg::PoseStamped>& optimal_points,
    const int start_x_index, const int start_y_index,
    const int end_x_index, const int end_y_index,
    rclcpp::Node& node,
    const nav_msgs::msg::OccupancyGrid& oc_grid,
    const std_msgs::msg::Float32MultiArray& dt_grid_float_values,
    const Vec& start_yaw 
);