#include "navigation/graph_builder.h"

#include "common/math.h"
#include "common/exception.h"

#include <boost/geometry.hpp>

namespace truck::navigation::graph {

namespace bg = boost::geometry;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

namespace {

geom::Vec2 toVec2(const RTreePoint& rtree_point) {
    return geom::Vec2(rtree_point.get<0>(), rtree_point.get<1>());
}

RTreePoint toRTreePoint(const geom::Vec2& point) {
    return RTreePoint(point.x, point.y);
}

RTreeIndexedPoint toRTreeIndexedPoint(const geom::Vec2& point, size_t index) {
    return RTreeIndexedPoint(toRTreePoint(point), index);
}

RTree toRTree(const std::vector<geom::Vec2>& points) {
    RTree rtree;

    for (size_t i = 0; i < points.size(); i++) {
        rtree.insert(toRTreeIndexedPoint(points[i], i));
    }

    return rtree;
}

RTreeIndexedPoints getNeighborsKNearest(const RTree& rtree, const geom::Vec2& point, size_t k_nearest) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), k_nearest + 1),
        std::back_inserter(rtree_indexed_points)
    );

    return rtree_indexed_points;
}

RTreeIndexedPoints getNeighborsInSearchRadius(const RTree& rtree, const geom::Vec2& point, double search_radius) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::satisfies([&](RTreeIndexedPoint const& rtree_indexed_point) {
            return (point - toVec2(rtree_indexed_point.first)).lenSq() < squared(search_radius);
        }),
        std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints getNeighbors(const RTree& rtree, const geom::Vec2& point, const GraphParams& params) {
    switch (params.mode) {
        case GraphParams::Mode::kNearest:
            return getNeighborsKNearest(rtree, point, params.k_nearest);
        case GraphParams::Mode::searchRadius:
            return getNeighborsInSearchRadius(rtree, point, params.search_radius);
        default:
            VERIFY(false);
            break;
    }
}

}

GraphBuilder::GraphBuilder(const GraphParams& params) : params_(params) {}

GraphBuilder& GraphBuilder::setNodes(const std::vector<geom::Vec2>& nodes) {
    nodes_ = nodes;
    return *this;
}

GraphBuilder& GraphBuilder::setComplexPolygons(const geom::ComplexPolygons& polygons) {
    polygons_ = polygons;
    return *this;
}

GraphBuilder& GraphBuilder::build() {
    RTree rtree_nodes = toRTree(nodes_);
    
    weights_ = std::vector<std::vector<std::optional<double>>>(
        nodes_.size(), std::vector<std::optional<double>>(nodes_.size(), std::nullopt));

    // iterate through every node
    for (const auto& cur_it : rtree_nodes) {
        geom::Vec2 cur_point = toVec2(cur_it.first);
        size_t cur_point_index = cur_it.second;

        // iterate through every neighbor of a current node
        for (const auto& neighbor_it : getNeighbors(rtree_nodes, cur_point, params_)) {
            geom::Vec2 neighbor_point = toVec2(neighbor_it.first);
            size_t neighbor_point_index = neighbor_it.second;

            if (cur_point_index != neighbor_point_index) {
                geom::Segment edge(cur_point, neighbor_point);

                if (collisionFreeEdge(edge)) {
                    weights_[cur_point_index][neighbor_point_index] = edge.len();
                    weights_[neighbor_point_index][cur_point_index] = edge.len();
                }
            }
        }
    }

    // get unique edges from weight matrix
    for (size_t i = 0; i < weights_.size(); i++) {
        for (size_t j = i + 1; j < weights_.size(); j++) {
            if (weights_[i][j].has_value()) {
                edges_.emplace_back(geom::Segment(nodes_[i], nodes_[j]));
            }
        }
    }

    return *this;
}

const std::vector<geom::Vec2>& GraphBuilder::getNodes() const { return nodes_; }

const geom::Segments& GraphBuilder::getEdges() const { return edges_; }

const std::vector<std::vector<std::optional<double>>>& GraphBuilder::getWeights() const { return weights_; }

bool GraphBuilder::collisionFreeEdge(const geom::Segment& edge) const {
    for (const geom::ComplexPolygon &polygon : polygons_) {
        if (polygon.outer.isIntersectSegment(edge)) {
            return false;
        }

        for (const geom::Polygon& inner_poly : polygon.inners) {
            if (inner_poly.isIntersectSegment(edge)) {
                return false;
            }
        }
    }

    return true;
}

}  // namespace truck::navigation::graph