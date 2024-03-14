#pragma once

#include "geom/pose.h"

#include <boost/geometry.hpp>

namespace truck::geom {

namespace bg = boost::geometry;

/** Wrapper for the R-Tree structure in the Boost C++ Library
 *
 * Information:
 * 1. geom::RTree object consists of pairs (2D point, index).
 * 2. implemented types convertions
 * 3. implemented spatial queries:
 *  3.1. search k-nearest points
 *  3.2. search points inside box
 *  3.3. search points inside circle
 */

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

Vec2 toVec2(const RTreePoint& rtree_point);

std::vector<Vec2> toVec2Array(const RTreeIndexedPoints& rtree_indexed_points);

RTreePoint toRTreePoint(const Vec2& point);

RTreeIndexedPoint toRTreeIndexedPoint(const Vec2& point, size_t index);

RTree toRTree(const std::vector<Vec2>& points);

RTreeIndexedPoints RTreeSearchKNN(const RTree& rtree, const Vec2& point, size_t k);

RTreeIndexedPoints RTreeSearchInsideBox(const RTree& rtree, const Vec2& center, double radius);

RTreeIndexedPoints RTreeSearchInsideCircle(const RTree& rtree, const Vec2& center, double radius);

}  // namespace truck::geom