#pragma once

#include "trajectory_planner/tree.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/function_output_iterator.hpp>

#include "common/math.h"

#include "geom/pose.h"
#include "geom/vector.h"

#include <vector>

BOOST_GEOMETRY_REGISTER_POINT_2D(truck::geom::Vec2, double, cs::cartesian, x, y)

namespace bc = boost::container;
namespace bg = boost::geometry;

namespace truck::trajectory_planner {

class SpatioTemporalRTree {
  public:
    using RTreeBox = typename bg::model::box<geom::Vec2>;
    using RTreeValue = typename std::pair<geom::Vec2, Node*>;
    using RTree = typename bg::index::rtree<RTreeValue, bg::index::rstar<16>>;

    using SearchResult = typename std::pair<double, Node*>;

    SpatioTemporalRTree() = default;

    SpatioTemporalRTree(const Discretization<double>& velocity_discretization);

    SpatioTemporalRTree(
        const Discretization<double>& velocity_discretization, const Edge::Estimator& estimator);

    SpatioTemporalRTree& Add(Node& node) noexcept;

    SpatioTemporalRTree& Remove(Node& node) noexcept;

    const std::vector<SearchResult>& RangeSearch(const Node& node, double radius) noexcept;

    SpatioTemporalRTree& Clear() noexcept;

    SpatioTemporalRTree& UpdateEstimator(const Edge::Estimator& estimator) noexcept;

  private:
    Discretization<double> velocity_discretization_;
    Edge::Estimator estimator_;

    std::vector<RTree> layers_;
    std::vector<SearchResult> result_buffer_;
};

}  // namespace truck::trajectory_planner