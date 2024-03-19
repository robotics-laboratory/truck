#pragma once

#include "trajectory_planner/state.h"

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
    using TreeBox = typename bg::model::box<geom::Vec2>;
    using TreeValue = typename std::pair<geom::Vec2, const State*>;
    using Tree = typename bg::index::rtree<TreeValue, bg::index::rstar<16>>;

    using SearchResult = typename std::pair<double, const State*>;

    SpatioTemporalRTree(const Discretization<double>& velocity_discretization);

    SpatioTemporalRTree& Add(const State& state) noexcept;

    void RangeSearch(
        const State& state, double radius, std::vector<SearchResult>& result_buffer) const noexcept;

  private:
    Discretization<double> velocity_discretization_;

    std::vector<Tree> layers_;
};

}  // namespace truck::trajectory_planner