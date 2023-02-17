#pragma once

#include "model/params.h"
#include "collision_checker/collision_checker.h"

#include <tf2/impl/utils.h>

#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

namespace truck::astar {

/**
 * Defining path elementary parts called primitives
 * @details each primitive is described by a set of subpoints (x,y) and directions in them (yaw)  
 */
struct Primitive {
    std::vector<geom::Vec2> sub_points;
    std::vector<double> yaws;
};

/**
 * Defining 2D indices of graph's vertices
 * @details each vertex has 2 indices (i, j) which are limited
 * @details by the physical dimensions of the graph and its resolution 
 */
struct Index2D {
    int i, j;
};

/**
 * Defining graph vertices
 */
struct Vertex {
    // position info
    Index2D index_;
    geom::Vec2 coord_;
    double incoming_yaw_;

    // scores info
    double g_, h_, f_;

    // neighbors info
    std::vector<Vertex*> neighbors_;
    std::vector<int> neighbors_primitives_indices_;

    // previous vertex pointer
    Vertex* prev_vertex_{nullptr};

    // index of primitive connecting previous and current vertices
    int prev_vertex_primitive_index_{NULL};

    Vertex();
    Vertex(const geom::Vec2& coord, const Index2D& index);
};

/**
 * A* planner
 * @details finding optimal path inside given graph
 * @details i -> rows -> height -> y
 * @details j -> columns -> width -> x
 */
class AStar {
    public:
        // read info about pre-calculated primitives from file
        void readData();

        // initialize graph's vertices
        void initializeGraph(const geom::Pose& start_pose, const geom::Pose& end_pose, const tf2::Transform& tr_from_ekf_to_base);

        // find optimal path and get its points
        std::vector<geom::Vec2> getPathPoints();

        // get vertices points of built graph
        std::vector<geom::Vec2> getGraphVerticesPoints();

        AStar(const collision_checker::StaticCollisionChecker& collision_checker);
    private:
        // graph properties
        int width_;
        int heigth_;
        double resolution_;

        // planner properties
        double max_turn_angle_;
        double min_obstacle_dist_;

        // stored info about pre-calculated primitives from file
        int primitives_count_;
        std::vector<Primitive> primitives_;
        std::vector<Index2D> neighbors_local_ind_;
        std::vector<double> yaws_;

        // graph vertices
        std::vector<std::vector<Vertex>> vertices_;

        // optimal path points
        std::vector<geom::Vec2> path_points_;

        // start and end poses
        geom::Pose start_pose_;
        geom::Pose end_pose_;

        // start and end vertices pointers
        Vertex* start_vertex_;
        Vertex* end_vertex_;

        // instance of StaticCollisionChecker library
        collision_checker::StaticCollisionChecker collision_checker_;

        // transform from ekf to base
        tf2::Transform tr_from_ekf_to_base_;

        // path construction constraints 
        bool collisionFree(const Primitive& primitive);
        bool acceptableAngle(double yaw_1, double yaw_2);
        
        // clip given yaw to the nearest pre-calculated one
        double clipYaw(double yaw);

        // construct optimal path  
        void calculatePathPoints(Vertex* vertex);

        // calculate approximate distance between given points
        double heuristic(const geom::Vec2& point_1, const geom::Vec2& point_2);
};
    
} // namespace truck::astar