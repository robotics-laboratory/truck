#include "motion_planner/motion_planner.h"
#include "common/exception.h"
#include "common/math.h"

#include <iostream>

namespace truck::motion_planner {

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

hull::GraphBuild GraphBuilder::buildGraph(const Reference& reference) const {
    hull::GraphBuild build = hull::GraphBuild{.reference = reference};
    makeMilestones(build);
    makeNodes(build);
    makeEdges(build);

    // return hull::Graph{.nodes = std::move(build.nodes), .edges = std::move(build.edges)};
    return build;
}

void GraphBuilder::makeMilestones(hull::GraphBuild& build) const {
    size_t milestone_id = 0;
    for (auto it = build.reference.AdvanceFromBegin(); !it.reached_end;
         it = build.reference.AdvanceFrom(it.poly_idx, params_.milestone_spacing)) {
        build.milestones.emplace_back(
            milestone_id++, it.point.pose(), params_.hull_radius, params_.hull_radius);
    }
}

void GraphBuilder::makeNodes(hull::GraphBuild& build) const {
    for (const auto& milestone : build.milestones) {
        build.milestone_nodes.emplace_back();
        const auto guides = milestone.getSpacedOutGuides(params_.node_spacing);
        for (const auto& [offset, guide] : guides) {
            hull::Node node = {
                .id = build.nodes.size(),
                .pose = guide,
                .edges = {},
                .milestone_id = milestone.id,
                .milestone_offset = offset};

            build.milestone_nodes[milestone.id].push_back(node.id);
            build.nodes.push_back(std::move(node));
        }
    }
}

void GraphBuilder::makeEdges(hull::GraphBuild& build) const {
    size_t watch_dog = 0;
    for (hull::Node& from : build.nodes) {
        if (build.milestone_nodes.size() >= from.id) {
            continue;
        }

        for (const NodeId to_id : build.milestone_nodes[from.id + 1]) {
            hull::Node& to = build.nodes[to_id];
            hull::Edge edge = {
                .id = build.edges.size(),
                .from = from.id,
                .to = to.id,
                .weight = geom::distance(from.pose, to.pose)};

            from.edges.push_back(edge.id);
            // to.edges.push_back(edge.id); // if graph is not oriented

            std::cerr << "edge.id: " << edge.id << "; from = " << from.id << "; to = " << to.id
                      << ".\n";
            build.edges.push_back(std::move(edge));
        }
    }
}

}  // namespace truck::motion_planner
