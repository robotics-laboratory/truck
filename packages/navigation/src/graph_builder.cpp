#include "navigation/graph_builder.h"

#include "common/exception.h"
#include "common/math.h"
#include "geom/distance.h"
#include "geom/intersection.h"
#include "geom/boost/point.h"
#include "geom/boost/box.h"

#include <boost/geometry.hpp>

#include <unordered_map>

namespace truck::navigation::graph {

namespace bg = boost::geometry;

using RTreeIndexedPoint = std::pair<geom::Vec2, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTreeBox = geom::BoundingBox;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

using EdgesMap = std::unordered_map<NodeId, std::unordered_map<NodeId, EdgeId>>;

namespace {

RTree toRTree(const Nodes& nodes) {
    RTree rtree;

    for (const Node& node : nodes) {
        rtree.insert(RTreeIndexedPoint(node.point, node.id));
    }

    return rtree;
}

RTreeIndexedPoints getNodeNeighborsKNN(
    const geom::Vec2& point, const RTree& rtree, size_t k_nearest) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(bg::index::nearest(point, k_nearest + 1), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints getNodeNeighborsSearchRadius(
    const geom::Vec2& point, const RTree& rtree, double search_radius) {
    RTreeIndexedPoints rtree_indexed_points;

    const RTreeBox rtree_box(
        geom::Vec2(point.x - search_radius, point.y - search_radius),
        geom::Vec2(point.x + search_radius, point.y + search_radius));

    rtree.query(
        bg::index::intersects(rtree_box)
            && bg::index::satisfies([&](const RTreeIndexedPoint& rtree_indexed_point) {
                   const geom::Vec2 neighbor_point = rtree_indexed_point.first;
                   return (point - neighbor_point).lenSq() < squared(search_radius);
               }),
        std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

bool isCollisionFree(const geom::Segment& edge, const geom::ComplexPolygons& polygons) {
    for (const geom::ComplexPolygon& polygon : polygons) {
        if (geom::intersect(polygon.outer, edge)) {
            return false;
        }

        for (const geom::Polygon& polygon_inner : polygon.inners) {
            if (geom::intersect(polygon_inner, edge)) {
                return false;
            }
        }
    }

    return true;
}

std::vector<NodeId> getNodeNeighbors(
    const Node& node, const RTree& rtree, const geom::ComplexPolygons& polygons,
    const GraphParams& params) {
    RTreeIndexedPoints rtree_indexed_points;

    switch (params.mode) {
        case GraphParams::Mode::kNearest:
            rtree_indexed_points = getNodeNeighborsKNN(node.point, rtree, params.k_nearest);
            break;
        case GraphParams::Mode::searchRadius:
            rtree_indexed_points =
                getNodeNeighborsSearchRadius(node.point, rtree, params.search_radius);
            break;
        default:
            VERIFY(false);
            break;
    }

    std::vector<NodeId> neighbor_nodes_ids;

    for (const RTreeIndexedPoint& rtree_indexed_point : rtree_indexed_points) {
        const NodeId neighbor_node_id = rtree_indexed_point.second;
        const geom::Vec2 neighbor_node_point = rtree_indexed_point.first;

        const geom::Segment edge(node.point, neighbor_node_point);

        if (node.id != neighbor_node_id && isCollisionFree(edge, polygons)) {
            neighbor_nodes_ids.emplace_back(neighbor_node_id);
        }
    }

    return neighbor_nodes_ids;
}

std::optional<EdgeId> tryGetEdge(EdgesMap& nodes_to_edges, NodeId from, NodeId to) {
    const auto from_node_it = nodes_to_edges.find(from);

    if (from_node_it == nodes_to_edges.end()) {
        return std::nullopt;
    }

    const auto to_node_it = from_node_it->second.find(to);

    if (to_node_it == from_node_it->second.end()) {
        return std::nullopt;
    }

    return to_node_it->second;
}

void updateEdgesInfo(Graph& graph, EdgesMap& nodes_to_edges, NodeId from, NodeId to) {
    std::optional<EdgeId> edge_id = tryGetEdge(nodes_to_edges, from, to);

    if (!edge_id.has_value()) {
        // edge doesn't exist, need to create one
        const Edge edge{
            .from = from,
            .to = to,
            .weight = geom::distance(graph.nodes[from].point, graph.nodes[to].point)};

        edge_id = graph.edges.size();
        nodes_to_edges[from][to] = *edge_id;

        graph.edges.emplace_back(edge);
    }

    graph.nodes[from].edges.emplace_back(*edge_id);
}

}  // namespace

GraphBuilder::GraphBuilder(const GraphParams& params) : params_(params) {}

Graph GraphBuilder::build(
    const std::vector<geom::Vec2>& mesh, const geom::ComplexPolygons& polygons) const {
    Graph graph;

    // initialize nodes
    for (size_t i = 0; i < mesh.size(); i++) {
        graph.nodes.emplace_back(Node{.id = i, .point = mesh[i], .edges = {}});
    }

    const RTree rtree_nodes = toRTree(graph.nodes);

    EdgesMap nodes_to_edges;

    for (const Node& cur_node : graph.nodes) {
        for (const NodeId neighbor_node_id :
             getNodeNeighbors(cur_node, rtree_nodes, polygons, params_)) {
            updateEdgesInfo(graph, nodes_to_edges, cur_node.id, neighbor_node_id);
            updateEdgesInfo(graph, nodes_to_edges, neighbor_node_id, cur_node.id);
        }
    }

    return graph;
}

}  // namespace truck::navigation::graph
