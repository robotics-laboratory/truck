#pragma once

#include "collision/collision_checker.h"

#include "fastgrid/distance_transform.h"
#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "fastgrid/interpolation.h"
#include "fastgrid/manhattan_distance.h"

#include "geom/bounding_box.h"
#include "geom/circle.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include "model/params.h"

// #include <boost/container/set.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/heap/fibonacci_heap.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

BOOST_GEOMETRY_REGISTER_POINT_2D(truck::geom::Vec2, double, cs::cartesian, x, y)

namespace truck::planner::tree_planner {

namespace bc = boost::container;
namespace bg = boost::geometry;
namespace bh = boost::heap;

const double unreachable = std::numeric_limits<double>::max();
const double eps = 1e-7;

struct Node;

struct Edge {
    Edge() = default;
    Edge(Node* destination, const geom::Poses& poses);
    Edge(Node* destination, geom::Poses&& poses);

    Node* destination;

    geom::Poses path;

    double length;
};

struct Node {
    Node() = default;
    Node(
        const geom::Vec2& pose, double cost_to_come, double locally_minimum_cost_to_come,
        double heuristics);
    Node(const geom::Vec2& pose, std::shared_ptr<Edge> parent, double heuristics);

    geom::Vec2 pose;

    std::shared_ptr<Edge> parent_edge = nullptr;
    std::vector<std::shared_ptr<Edge>> successor_edges;

    double cost_to_come = unreachable;
    double locally_minimum_cost_to_come = unreachable;

    double heuristics = unreachable;

    bool is_start_node = false;
    bool is_goal_node = false;
    bool is_optimal_goal_node = false;

    struct PtrCompare {
        inline bool operator()(const Node* node_1, const Node* node_2) const noexcept;
    };

    bh::fibonacci_heap<Node*, bh::compare<Node::PtrCompare>>::handle_type heap_handle;
};

struct TreePlannerParams {
    int max_sampling_num;
    double steering_dist;
    double sampling_range;
    double goal_region_radius;
};

class TreePlanner {
  public:
    TreePlanner();

    TreePlanner& SetParams(const TreePlannerParams& params);

    TreePlanner& SetShape(const model::Shape& shape);

    TreePlanner& Build() noexcept;

    geom::Poses GetPath() const noexcept;

    const Node* GetStartNode() const noexcept;

    const Node* GetGoalNode() const noexcept;

    bool FoundPath() const noexcept;

    TreePlanner& Reset(
        const geom::BBox& planning_space, const geom::Vec2& start_pose,
        const geom::Circle& goal_region,
        std::shared_ptr<const collision::StaticCollisionChecker> checker) noexcept;

    const std::vector<Node>& GetNodes() const noexcept;

  private:
    using FibonacciHeap = bh::fibonacci_heap<Node*, bh::compare<Node::PtrCompare>>;

    // using SplayTreeType = bc::tree_assoc_options<bc::tree_type<bc::splay_tree>>::type;
    // using SplayTree = bc::set<Node*, Node::PtrCompare, bc::new_allocator<Node*>, SplayTreeType>;

    using RStarTreeValue = std::pair<geom::Vec2, Node*>;
    using RStarTreeBox = bg::model::box<geom::Vec2>;
    using RStarTree = bg::index::rtree<RStarTreeValue, bg::index::rstar<16>>;

    static constexpr int dim_ = 2;
    static constexpr double multiplier_ = 1.1;
    const double gamma_ = std::pow(2, dim_) * (1.0 + 1.0 / static_cast<double>(dim_));

    template<typename... Args>
    Node& AddNode(Args&&... args) noexcept {
        nodes_.emplace_back(std::forward<Args>(args)...);
        nodes_.back().heap_handle = heap_.push(&nodes_.back());
        r_star_tree_.insert(std::make_pair(nodes_.back().pose, &nodes_.back()));
        return nodes_.back();
    }

    template<typename... Args>
    std::shared_ptr<Edge> AddEdge(Args&&... args) noexcept {
        return std::make_shared<Edge>(std::forward<Args>(args)...);
    }

    geom::Vec2 Steer(const geom::Vec2& nearest_point, const geom::Vec2& point) const noexcept;

    double GetBallRadius() const noexcept;

    std::optional<geom::Poses> TryConnect(
        const geom::Vec2& from, const geom::Vec2& to) const noexcept;

    void UpdateQueue(Node* node) noexcept;

    void UpdateOptimalGoalNode(Node* node) noexcept;

    void Extend(const geom::BBox& bbox) noexcept;

    void ReduceInconsistency() noexcept;

    const std::function<double()> rand_gen_;

    TreePlannerParams params_;

    geom::BBox planning_space_;
    geom::Vec2 start_pose_;
    geom::Circle goal_region_;
    model::Shape shape_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;

    Node* optimal_goal_node_ = nullptr;

    std::vector<Node> nodes_;

    FibonacciHeap heap_;
    // SplayTree splay_;
    RStarTree r_star_tree_;

    int sampled_points_ = 0;
};

}  // namespace truck::planner::tree_planner
