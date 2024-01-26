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

RTreePoint toRTreePoint(const geom::Vec2& point) {
    return RTreePoint(point.x, point.y);
}

geom::Vec2 toVec2(const RTreePoint& rtree_point) {
    return geom::Vec2(rtree_point.get<0>(), rtree_point.get<1>());
}

RTree toRTree(const std::vector<geom::Vec2>& points) {
    RTree rtree;

    for (size_t i = 0; i < points.size(); i++) {
        rtree.insert(RTreeIndexedPoint(toRTreePoint(points[i]), i));
    }

    return rtree;
}

RTreeIndexedPoints getNeighborsKNearest(const RTree& rtree, const geom::Vec2& point, size_t k_nearest) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), k_nearest),
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

RTreeIndexedPoints getNeighbors(const RTree& rtree, const geom::Vec2& point, const GraphParams::Neighbor& neighbor_params) {
    switch (neighbor_params.mode) {
        case GraphParams::Neighbor::Mode::kNearest:
            return getNeighborsKNearest(rtree, point, neighbor_params.k_nearest);
        case GraphParams::Neighbor::Mode::searchRadius:
            return getNeighborsInSearchRadius(rtree, point, neighbor_params.search_radius);
        default:
            VERIFY(false);
            break;
    }
}

}

GraphBuilder::GraphBuilder(const GraphParams& params) : params_(params) {}

GraphBuild GraphBuilder::build(const std::vector<geom::Vec2>& mesh) {
    GraphBuild graph_build;

    buildEdges(graph_build, mesh);
    // buildRoute();

    return graph_build;
}

void GraphBuilder::buildEdges(GraphBuild& graph_build, const std::vector<geom::Vec2>& mesh) {
    weights_ = std::vector<std::vector<std::optional<double>>>(
        mesh.size(), std::vector<std::optional<double>>(mesh.size(), std::nullopt));

    RTree rtree_mesh = toRTree(mesh);

    // iterate through every mesh point
    for (const auto& cur_it : rtree_mesh) {
        geom::Vec2 cur_point = toVec2(cur_it.first);
        size_t cur_point_index = cur_it.second;

        // iterate through every neighbor of current mesh point
        for (const auto& neighbor_it : getNeighbors(rtree_mesh, cur_point, params_.neighbor)) {
            geom::Vec2 neighbor_point = toVec2(neighbor_it.first);
            size_t neighbor_point_index = neighbor_it.second;

            geom::Segment edge(cur_point, neighbor_point);

            graph_build.edges.emplace_back(edge);

            weights_[cur_point_index][neighbor_point_index] = edge.len();
        }
    }
}

void GraphBuilder::buildRoute() {
    /** @todo */
}

}  // namespace truck::navigation::graph