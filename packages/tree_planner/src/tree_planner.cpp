#include "tree_planner/tree_planner.h"

#include <cmath>
#include <optional>
#include <random>
#include <string_view>

#include "geom/angle_vector.h"
#include "geom/transform.h"

namespace truck::planner::tree_planner {

Edge::Edge(Node* destination, const geom::Poses& poses)
    : destination(destination)
    , poses(poses)
    , length((poses.back().pos - poses.front().pos).len()) {}

Edge::Edge(Node* destination, geom::Poses&& poses)
    : destination(destination)
    , poses(std::move(poses))
    , length((poses.back().pos - poses.front().pos).len()) {}

Node::Node(
    const geom::Vec2& pose, double cost_to_come, double locally_minimum_cost_to_come,
    double heuristics)
    : pose(pose)
    , cost_to_come(cost_to_come)
    , locally_minimum_cost_to_come(locally_minimum_cost_to_come)
    , heuristics(heuristics) {}

Node::Node(const geom::Vec2& pose, Edge* parent_edge, double heuristics)
    : pose(pose)
    , cost_to_come(unreachable)
    , locally_minimum_cost_to_come(parent_edge->destination->cost_to_come + parent_edge->length)
    , heuristics(heuristics) {}

inline bool Node::PtrCompare::operator()(const Node* node_1, const Node* node_2) const noexcept {
    if (node_2 == nullptr) {
        return true;
    }
    if (node_1 == nullptr) {
        return false;
    }
    int node_1_g_value = std::min(node_1->cost_to_come, node_1->locally_minimum_cost_to_come);
    int node_2_g_value = std::min(node_2->cost_to_come, node_2->locally_minimum_cost_to_come);
    return std::make_pair(node_1_g_value + node_1->heuristics, node_1_g_value) <
           std::make_pair(node_2_g_value + node_2->heuristics, node_2_g_value);
}

const std::vector<Node>& TreePlanner::GetNodes() const noexcept { return nodes_; }

TreePlanner::TreePlanner()
    : rand_gen_(
          std::bind(std::uniform_real_distribution<>(0.0, 1.0), std::default_random_engine())) {}

void TreePlanner::SetParams(const TreePlannerParams& params) {
    params_ = params;
    nodes_.reserve(params_.max_sampling_num);
    edges_.reserve(params_.max_sampling_num - 1);
}

void TreePlanner::Build() noexcept {
    while (nodes_.size() < static_cast<size_t>(params_.max_sampling_num)) {
        Extend(planning_space_);
        ReduceInconsistency();
    }
}

geom::Poses TreePlanner::GetPath() const noexcept {
    if (optimal_goal_node_ == nullptr) {
        return {};
    }

    geom::Poses path;
    Node* cur_node = optimal_goal_node_;
    while (cur_node->parent_edge != nullptr) {
        auto it = cur_node->parent_edge->poses.begin();
        while (it + 1 != cur_node->parent_edge->poses.end()) {
            path.push_back(*it);
        }
        cur_node = cur_node->parent_edge->destination;
    }
    path.push_back(geom::Pose(
        cur_node->pose,
        geom::AngleVec2::fromVector(
            path.empty() ? geom::Vec2(0, 0) : (path.back().pos - cur_node->pose).unit())));
    return path;
}

const Node* TreePlanner::GetGoalNode() const noexcept { return optimal_goal_node_; }

bool TreePlanner::FoundPath() const noexcept { return optimal_goal_node_ != nullptr; }

TreePlanner& TreePlanner::Reset(
    const geom::BBox& planning_space, const geom::Vec2& start_pose, const geom::Circle& goal_region,
    const model::Shape& shape,
    std::shared_ptr<const collision::StaticCollisionChecker> checker) noexcept {
    nodes_.clear();
    edges_.clear();
    heap_.clear();
    r_star_tree_.clear();
    planning_space_ = planning_space;
    start_pose_ = start_pose;
    goal_region_ = goal_region;
    shape_ = shape;
    checker_ = std::move(checker);
    AddNode(start_pose_, 0, 0, (start_pose_ - goal_region_.center).len()).is_start_node = true;
    return *this;
}

geom::Vec2 TreePlanner::Steer(
    const geom::Vec2& nearest_point, const geom::Vec2& point) const noexcept {
    const geom::Vec2 diff_vec = point - nearest_point;
    if (diff_vec.len() > params_.steering_dist) {
        return nearest_point + diff_vec.unit() * params_.steering_dist;
    }
    return point;
}

double TreePlanner::GetBallRadius() const noexcept {
    return gamma_ * std::log(nodes_.size()) / nodes_.size();
}

std::optional<geom::Poses> TreePlanner::TryConnect(
    const geom::Vec2& from, const geom::Vec2& to) const noexcept {
    bool is_obstacle_free = true;
    const auto dir_vector = to - from;
    geom::Poses path;
    path.reserve(dir_vector.len() / shape_.length + 1);
    for (int i = 0; i < dir_vector.len() / shape_.length; ++i) {
        path.push_back(geom::Pose(
            from + dir_vector.unit() * shape_.length * i,
            geom::AngleVec2::fromVector(dir_vector.unit())));
        is_obstacle_free &= checker_->distance(path.back()) > shape_.width / 2;
    }
    path.push_back(geom::Pose(to, geom::AngleVec2::fromVector(dir_vector.unit())));
    is_obstacle_free &= checker_->distance(path.back()) > shape_.width / 2;

    if (!is_obstacle_free) {
        return std::nullopt;
    }

    return path;
}

void TreePlanner::UpdateQueue(Node* node) noexcept {
    if (node->cost_to_come != node->locally_minimum_cost_to_come) {
        if (node->heap_handle.node_) {
            heap_.update(node->heap_handle);
        } else {
            node->heap_handle = heap_.push(node);
        }
    } else {
        heap_.erase(node->heap_handle);
    }
}

void TreePlanner::Extend(const geom::BBox& bbox) noexcept {
    geom::Vec2 sampled_point = geom::Vec2(
        bbox.left_lower.x + (bbox.right_upper.x - bbox.left_lower.x) * rand_gen_(),
        bbox.left_lower.y + (bbox.right_upper.y - bbox.left_lower.y) * rand_gen_());

    std::vector<RStarTreeValue> nearest_nodes;
    r_star_tree_.query(bg::index::nearest(sampled_point, 1), std::back_inserter(nearest_nodes));
    const auto [nearest_point, nearest_node] = nearest_nodes.front();
    const geom::Vec2 steered_point = Steer(nearest_point, sampled_point);

    const auto path = TryConnect(nearest_point, steered_point);
    if (!path) {
        return;
    }
    geom::Poses reversed_path = *path;
    std::reverse(reversed_path.begin(), reversed_path.end());

    Node* new_node = &AddNode(
        steered_point,
        &AddEdge(nearest_node, std::move(reversed_path)),
        (start_pose_ - goal_region_.center).len());

    new_node->successor_edges.push_back(&AddEdge(new_node, std::move(*path)));

    nearest_nodes.clear();
    const double query_radius = GetBallRadius();
    RStarTreeBox query_box(
        geom::Vec2(new_node->pose.x - query_radius, new_node->pose.y - query_radius),
        geom::Vec2(new_node->pose.x + query_radius, new_node->pose.y + query_radius));
    r_star_tree_.query(bg::index::intersects(query_box), std::back_inserter(nearest_nodes));
    for (auto [point, node] : nearest_nodes) {
        if ((point - new_node->pose).len() < eps) {
            continue;
        }

        const auto path = TryConnect(point, new_node->pose);
        if (!path) {
            continue;
        }

        double cur_locally_minimal_cost_to_come =
            node->cost_to_come + (path->back().pos - path->front().pos).len();
        if (new_node->locally_minimum_cost_to_come > cur_locally_minimal_cost_to_come) {
            geom::Poses reversed_path = *path;
            std::reverse(reversed_path.begin(), reversed_path.end());
            new_node->locally_minimum_cost_to_come = cur_locally_minimal_cost_to_come;
            new_node->parent_edge->destination = node;
            new_node->parent_edge->length =
                (reversed_path.back().pos - reversed_path.front().pos).len();
            new_node->parent_edge->poses = std::move(reversed_path);
        }
        node->successor_edges.push_back(&AddEdge(new_node, std::move(*path)));
    }

    UpdateQueue(new_node);

    if (std::pow(goal_region_.center.x - new_node->pose.x, 2) +
            std::pow(goal_region_.center.y - new_node->pose.y, 2) <=
        std::pow(goal_region_.radius, 2)) {
        new_node->is_goal_node = true;
        if (Node::PtrCompare()(new_node, optimal_goal_node_)) {
            optimal_goal_node_->is_optimal_goal_node = false;
            optimal_goal_node_ = new_node;
            optimal_goal_node_->is_optimal_goal_node = true;
        }
    }
}

void TreePlanner::ReduceInconsistency() noexcept {
    while (!heap_.empty() &&
           (optimal_goal_node_ == nullptr || Node::PtrCompare()(heap_.top(), optimal_goal_node_))) {
        Node* top_node = heap_.top();
        top_node->cost_to_come = top_node->locally_minimum_cost_to_come;
        heap_.erase(top_node->heap_handle);

        for (auto* succ_edge : top_node->successor_edges) {
            if (succ_edge->destination->locally_minimum_cost_to_come >
                top_node->cost_to_come + succ_edge->length) {
                succ_edge->destination->locally_minimum_cost_to_come =
                    top_node->cost_to_come + succ_edge->length;
                geom::Poses reversed_path = succ_edge->poses;
                std::reverse(reversed_path.begin(), reversed_path.end());
                succ_edge->destination->parent_edge->destination = top_node;
                succ_edge->destination->parent_edge->poses = std::move(reversed_path);
                succ_edge->destination->parent_edge->length = succ_edge->length;

                UpdateQueue(succ_edge->destination);
            }
        }
    }
}

}  // namespace truck::planner::tree_planner
