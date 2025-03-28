#include "motion_planner/graph_builder.h"
#include "common/exception.h"
#include "common/math.h"
#include "geom/complex_polygon.h"
#include "geom/boost.h"
#include "boost/geometry/geometry.hpp"

#include <iostream>

namespace truck::motion_planner {

namespace bg = boost::geometry;

using IndexPoint = std::pair<geom::Vec2, std::size_t>;
using IndexPoints = std::vector<IndexPoint>;
using RTree = bg::index::rtree<IndexPoint, bg::index::rstar<16>>;

namespace {

bool areWithinDistance(
    const auto& geometry, const geom::ComplexPolygons& polygons, double distance) {
    for (const geom::ComplexPolygon& polygon : polygons) {
        for (const geom::Segment& segment : polygon.segments()) {
            if (bg::distance(segment, geometry) < distance) {
                return true;
            }
        }

        for (const geom::Polygon& polygon_inner : polygon.inners) {
            if (bg::distance(polygon_inner, geometry) < distance) {
                return true;
            }
        }
    }

    return false;
}

RTree toRTree(const Reference& reference) {
    RTree rtree;
    for (std::size_t i = 0; i < reference.Points().size(); ++i) {
        rtree.insert(IndexPoint(reference.Points().at(i).pos, i));
    }

    return rtree;
}

IndexPoints pointsInSearchRadius(
    const geom::Vec2& point, const RTree& rtree, double search_radius) {
    IndexPoints rtree_indexed_points;

    const geom::BoundingBox rtree_box(
        geom::Vec2(point.x - search_radius, point.y - search_radius),
        geom::Vec2(point.x + search_radius, point.y + search_radius));

    rtree.query(
        bg::index::intersects(rtree_box)
            && bg::index::satisfies([&](const IndexPoint& rtree_indexed_point) {
                   const geom::Vec2 neighbor_point = rtree_indexed_point.first;
                   return (point - neighbor_point).lenSq() < squared(search_radius);
               }),
        std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

}  // namespace

hull::Milestone::Milestone(
    MilestoneId id, const geom::Pose& pose, double left_ledge, double right_ledge) noexcept :
    id{id}, pose{pose}, left_ledge{left_ledge}, right_ledge{right_ledge} {}

hull::Guide hull::Milestone::guide(double offset) const {
    VERIFY(Limits(-left_ledge, right_ledge).isMet(offset));
    return hull::Guide{.pos = (pose.dir.right().vec() * offset) + pose.pos, .dir = pose.dir};
}

hull::IndexGuides hull::Milestone::getSpacedOutGuides(double spacing) const {
    VERIFY(spacing > 0);

    IndexGuides guides;

    const int k_left = left_ledge / spacing;
    const int k_right = right_ledge / spacing;

    for (int i = -k_left; i <= k_right; ++i) {
        guides.emplace_back(i, guide(i * spacing));
    }

    return guides;
}

std::pair<hull::Guide, hull::Guide> hull::Milestone::getEndpoints() const {
    return std::make_pair(guide(-left_ledge), guide(right_ledge));
}

geom::Segment hull::Milestone::toSegment() const {
    return geom::Segment(guide(-left_ledge).pos, guide(right_ledge).pos);
}

GraphBuilder::GraphBuilder(const hull::GraphParams& params) : params_{params} {}

std::pair<hull::Graph, hull::GraphContext> GraphBuilder::buildGraph(
    const Reference& reference, const geom::ComplexPolygon& map) const {
    hull::GraphBuild build = hull::GraphBuild{.map = map, .reference = reference};
    makeMilestones(build);
    makeNodes(build);
    makeEdges(build);

    return {
        hull::Graph{
            .nodes = std::move(build.nodes),
            .edges = std::move(build.edges),
        },
        hull::GraphContext{
            .milestones = std::move(build.milestones),
            .milestone_nodes = std::move(build.milestone_nodes),
        },
    };
}

hull::Milestone makeMilestone(
    std::size_t milestone_id, const geom::Pose& milestone_pose, const hull::GraphParams& params,
    const hull::GraphBuild& build, const geom::ComplexPolygon& map) {
    RTree reference_points = toRTree(build.reference);

    auto static_collision_check = [&](const geom::Vec2& p,
                                      const geom::ComplexPolygon& complex_polygon) -> bool {
        if (areWithinDistance(p, {complex_polygon}, params.safezone_radius)
            || !bg::within(p, complex_polygon.outer)) {
            return true;
        }

        return false;
    };

    auto milestone_intersection_check =
        [&](const geom::Vec2& p, const geom::Vec2& ray_start, double ray_length) {
            auto pts = pointsInSearchRadius(p, reference_points, ray_length);
            for (const auto& [q, i] : pts) {
                if (geom::distance(ray_start, q)
                    >= .5 * params.milestone_spacing /* TODO come up with good constant */) {
                    return true;
                }
            }

            return false;
        };

    auto ray_cast = [&](const geom::Pose& start) -> double {
        std::size_t n_iterations = 0;

        for (geom::Vec2 curr = start.pos;
             !static_collision_check(curr, map)
             && !milestone_intersection_check(
                 curr, start.pos, n_iterations * params.raycast_increment)
             && n_iterations * params.raycast_increment < params.hull_radius
             /* TODO: online collision checks*/;
             ++n_iterations) {
            curr += start.dir * params.raycast_increment;
        }

        return (n_iterations - 1) * params.raycast_increment;
    };

    double left_ledge = ray_cast({milestone_pose.pos, milestone_pose.dir.left()});
    double right_ledge = ray_cast({milestone_pose.pos, milestone_pose.dir.right()});

    return hull::Milestone(milestone_id, milestone_pose, left_ledge, right_ledge);
}

void GraphBuilder::makeMilestones(hull::GraphBuild& build) const {
    std::size_t milestone_id = 0;
    for (auto it = build.reference.AdvanceFromBegin(); !it.reached_end;
         it = build.reference.AdvanceFrom(it.poly_idx, params_.milestone_spacing)) {
        build.milestones.push_back(
            std::move(makeMilestone(milestone_id++, it.point, params_, build, build.map)));
    }
}

void GraphBuilder::makeNodes(hull::GraphBuild& build) const {
    for (const auto& milestone : build.milestones) {
        build.milestone_nodes.emplace_back();
        const auto guides = milestone.getSpacedOutGuides(params_.node_spacing);
        for (const auto& [offset, guide] : guides) {
            if (areWithinDistance(guide.pos, {build.map}, params_.safezone_radius)) {
                continue;
            }

            hull::Node node = {
                .id = build.nodes.size(),
                .pose = guide,
                .out = {},
                .in = {},
                .milestone_id = milestone.id,
                .milestone_offset = offset,
            };

            build.milestone_nodes.at(milestone.id).push_back(node.id);
            build.nodes.push_back(std::move(node));
        }
    }
}

void GraphBuilder::makeEdges(hull::GraphBuild& build) const {
    std::size_t watch_dog = 0;
    for (hull::Node& from : build.nodes) {
        if (build.milestone_nodes.size() <= from.milestone_id + 1) {
            continue;
        }

        for (const NodeId to_id : build.milestone_nodes.at(from.milestone_id + 1)) {
            hull::Node& to = build.nodes.at(to_id);
            hull::Edge edge = {
                .id = build.edges.size(),
                .from = from.id,
                .to = to.id,
                .weight = geom::distance(from.pose, to.pose)};  // TODO: add heuristic

            if (std::abs(from.milestone_offset - to.milestone_offset) * params_.node_spacing
                    < params_.milestone_spacing * params_.max_edge_slope
                && !areWithinDistance(
                    geom::Segment{from.pose.pos, to.pose.pos}, {build.map}, params_.safezone_radius)
                && (!from.in.empty() || from.milestone_id == 0)) {
                from.out.push_back(edge.id);
                to.in.push_back(edge.id);  // if graph is not oriented

                // std::cerr << "edge.id: " << edge.id << "; from = " << from.id << " on "
                //           << from.milestone_id << "; to = " << to.id << " on " << to.milestone_id
                //           << ".\n";

                build.edges.push_back(std::move(edge));

                if (watch_dog++ > 1e6) {
                    std::cerr << "watchdog!\n";
                    return;
                }
            }
        }
    }

    std::cerr << "#edges = " << build.edges.size() << "\n";
}

}  // namespace truck::motion_planner
