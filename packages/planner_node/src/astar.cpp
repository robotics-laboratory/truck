#include "astar.h"

namespace truck::astar {

Vertex::Vertex() {}

Vertex::Vertex(const geom::Vec2& coord, const Index2D& index)
    : coord_(coord)
    , index_(index)
    , g_(0.0)
    , h_(0.0)
    , f_(0.0)
    , incoming_yaw_(0.0) {}

AStar::AStar(const collision_checker::StaticCollisionChecker& collision_checker)
    : width_(20)
    , heigth_(20)
    , resolution_(0.2)
    , max_turn_angle_(0.0)
    , min_obstacle_dist_(0.3)
    , collision_checker_(collision_checker) {
    // pre-initialize graph's vertices
    vertices_ =
        std::vector<std::vector<Vertex>>(
            int(heigth_ / resolution_),
            std::vector<Vertex>(int(width_ / resolution_))
        );
}

/**
 * Read info about pre-calculated primitives from file
 * @details for now the this function is implemented poorly,
 * @details I don't see the point in macking it better,
 * @details cause you still wanted me to change .txt to .json
 */
void AStar::readData() {
    std::fstream file("/truck/packages/planner/data/primitives.txt");
    std::string line, log;

    // read yaws
    getline(file, line);
    for (int i = 0; i < 12; i++) {
        double yaw;
        file >> yaw;
        yaws_.push_back(yaw);
    }

    getline(file, line);
    getline(file, line);
    getline(file, line);
    
    // read neighbors indices
    for (int i = 0; i < 36; i++) {
        getline(file, line);
        std::istringstream is(line);
        int ind_x, ind_y;
        is >> ind_x >> ind_y;
        neighbors_local_ind_.push_back(Index2D{ind_y, ind_x});
    }

    // read primitives
    primitives_count_ = 36;

    bool reading = false;
    Primitive primitive;

    while (getline(file, line)) {
        if (reading == true) {
            if (line != "") {
                std::istringstream is(line);
                double x, y, yaw;
                while (is >> x >> y >> yaw) {
                    geom::Vec2 p(x, y);
                    primitive.sub_points.push_back(p);
                    primitive.yaws.push_back(yaw);
                }
            } else {
                // save data about current primitive
                reading = false;
                primitives_.push_back(primitive);
                primitive.sub_points.clear();
                primitive.yaws.clear();
            }
        }

        if (line == "Edge") {
            reading = true;
        }
    }

    if (reading == true) {
        reading = false;
        primitives_.push_back(primitive);
        primitive.sub_points.clear();
        primitive.yaws.clear();
    }
}

std::vector<geom::Vec2> AStar::getGraphVerticesPoints() {
    std::vector<geom::Vec2> vertices_points;

    for (int i = 0; i < int(heigth_ / resolution_); i++) {
        for (int j = 0; j < int(width_ / resolution_); j++) {
            vertices_points.push_back(vertices_[i][j].coord_);
        }
    }

    return vertices_points;
}

void AStar::initializeGraph(const geom::Pose& start_pose, const geom::Pose& end_pose, const tf2::Transform& tr_from_ekf_to_base) {
    start_pose_ = start_pose;
    end_pose_ = end_pose;
    tr_from_ekf_to_base_ = tr_from_ekf_to_base;

    geom::Vec2 origin_vertex(
        -(width_ / 2) + start_pose.pos.x,
        -(heigth_ / 2) + start_pose.pos.y
    );

    Index2D start_pose_index{
        std::round((start_pose.pos.y - origin_vertex.y) / resolution_),
        std::round((start_pose.pos.x - origin_vertex.x) / resolution_)
    };

    Index2D end_pose_index{
        std::round((end_pose.pos.y - origin_vertex.y) / resolution_),
        std::round((end_pose.pos.x - origin_vertex.x) / resolution_)
    };
    
    // set coordiantes and indices for each vertex
    for (int i = 0; i < int(heigth_ / resolution_); i++) {
        for (int j = 0; j < int(width_ / resolution_); j++) {
            // initialize vertex
            vertices_[i][j].coord_ = origin_vertex + (geom::Vec2(j, i) * resolution_);
            vertices_[i][j].index_ = Index2D{i, j};

            // save pointer to start vertex
            if ((i == start_pose_index.i) && (j == start_pose_index.j)) {
                start_vertex_ = &vertices_[i][j];
            }
            
            // save pointer to end vertex
            if ((i == end_pose_index.i) && (j == end_pose_index.j)) {
                end_vertex_ = &vertices_[i][j];
            }
        }
    }

    // set neighbors for each vertex
    for (int i = 0; i < int(heigth_ / resolution_); i++) {
        for (int j = 0; j < int(width_ / resolution_); j++) {

            // find potential neighbors
            for (int k = 0; k < primitives_count_; k++) {
                Index2D neighbor_global_ind{
                    neighbors_local_ind_[k].i + i,
                    neighbors_local_ind_[k].j + j
                };
                
                // check indices borders
                if ((neighbor_global_ind.i > 0 && neighbor_global_ind.i < int(heigth_ / resolution_)) &&
                    (neighbor_global_ind.j > 0 && neighbor_global_ind.j < int(width_ / resolution_))) {
                        // store neighbor
                        vertices_[i][j].neighbors_.push_back(
                            &vertices_[neighbor_global_ind.i][neighbor_global_ind.j]
                        );

                        // store index of primitive from (i,j) vertex to its 'k'-th neighbor
                        vertices_[i][j].neighbors_primitives_indices_.push_back(k);
                }
            }
        }
    }
}

/**
 * Check if angle between two yaws is not greater than permissible
 * @param yaw_1 current yaw
 * @param yaw_2 initial yaw of potential primitive
 * @return bool
 */
bool AStar::acceptableAngle(double yaw_1, double yaw_2) {
    /** @todo */
}

/**
 * Check if it's possible to drive through given primitive without collisions
 * @details each primitive's subpoint is checked for a collision by StaticCollisionChecker library
 * @details primitive subpoints and yaws should be converted from 'odom_ekf' frame to 'base' frame
 * @param primitive
 * @return bool
 */
bool AStar::collisionFree(const Primitive& primitive) {
    for (const geom::Vec2& point : primitive.sub_points) {
        // convert point from 'ekf' to 'base' frame
        tf2::Vector3 base_pos_tf2 =
            tr_from_ekf_to_base_(tf2::Vector3(point.x, point.y, 0.0));

        geom::Vec2 base_pos(base_pos_tf2.x(), base_pos_tf2.y());

        /** @todo */
        // convert yaw from 'ekf' to 'base' frame
        geom::Vec2 base_dir;

        geom::Pose base_pose;
        base_pose.pos = base_pos;
        base_pose.dir = base_dir;

        if (collision_checker_(base_pose) < min_obstacle_dist_) {
            return false;
        }
    }

    return true;
}

double AStar::heuristic(const geom::Vec2& point_1, const geom::Vec2& point_2) {
    return (point_1 - point_2).len();
}

void AStar::calculatePathPoints(Vertex* vertex) {
    while (vertex->prev_vertex_ != nullptr) {
        Primitive& primitive = primitives_[vertex->prev_vertex_primitive_index_];

        for (const geom::Vec2& point : primitive.sub_points) {
            path_points_.push_back(point + vertex->prev_vertex_->coord_);
        }

        vertex = vertex->prev_vertex_;
    }
}

double AStar::clipYaw(double yaw) {
    double clipped_yaw = yaws_[0];
    double min_dist_clipped_yaw = abs(yaw - clipped_yaw);

    for (int i = 0; i < yaws_.size(); i++) {
        double cur_dist_clipped_yaw = abs(yaw - yaws_[i]);

        if (cur_dist_clipped_yaw < min_dist_clipped_yaw) {
            clipped_yaw = yaws_[i];
            min_dist_clipped_yaw = cur_dist_clipped_yaw;
        }
    }

    return clipped_yaw;
}

std::vector<geom::Vec2> AStar::getPathPoints() {
    bool first_time = true;

    Vertex* cur_vertex;
    std::set<Vertex*> open_set, closed_set;

    open_set.insert(start_vertex_);

    while (open_set.size() > 0) {
        std::vector<Vertex*> open_set_vector;
        std::copy(open_set.begin(), open_set.end(), std::back_inserter(open_set_vector));

        auto it_min_f = open_set_vector.begin();
        for (auto it = open_set_vector.begin(); it != open_set_vector.end(); it++) {
            if ((*it)->f_ < (*it_min_f)->f_) {
                it_min_f = it;
            }
        }

        cur_vertex = *it_min_f;

        if (first_time == true) {
            cur_vertex->incoming_yaw_ = clipYaw(start_pose_.dir.angle()._mPI_PI().radians());
            first_time = false;
        } else {
            cur_vertex->incoming_yaw_ = primitives_[cur_vertex->prev_vertex_primitive_index_].yaws[-1];
        }

        /** @todo */
        // consider yaw of end point when checking for found path
        if ((cur_vertex->coord_.x == end_vertex_->coord_.x) &&
            (cur_vertex->coord_.y == end_vertex_->coord_.y)) {
            // path found
            // store all primitives' subpoints
            calculatePathPoints(cur_vertex);
            return path_points_;
        }

        closed_set.insert(cur_vertex);
        open_set.erase(cur_vertex);

        // look at neighbors
        for (int i = 0; i < cur_vertex->neighbors_.size(); i++) {
            bool neighbor_changed = false;
            int primitive_index_to_neighbor = cur_vertex->neighbors_primitives_indices_[i];

            double neighbor_start_yaw = primitives_[primitive_index_to_neighbor].yaws[0];
            Vertex* neighbor = cur_vertex->neighbors_[i];

            // check constraints
            if ((closed_set.find(neighbor) == closed_set.end())) {
                double tmp_g =
                    cur_vertex->g_ + \
                    heuristic(cur_vertex->coord_, neighbor->coord_);

                if (open_set.find(neighbor) != open_set.end()) {
                    if (tmp_g < neighbor->g_) {
                        neighbor_changed = true;
                        neighbor->g_ = tmp_g;
                    }
                } else {
                    neighbor_changed = true;
                    neighbor->g_ = tmp_g;
                    open_set.insert(neighbor);
                }
            }

            if (neighbor_changed == true) {
                neighbor->h_ = heuristic(neighbor->coord_, end_vertex_->coord_);
                neighbor->f_ = neighbor->g_ + neighbor->h_;
                neighbor->prev_vertex_ = cur_vertex;
                neighbor->prev_vertex_primitive_index_ = primitive_index_to_neighbor;
            }
        }
    }
    
    // path not found
    return path_points_;
}

} // namespace truck::astar