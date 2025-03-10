#include "motion_planner/motion_planner.h"
#include "common/exception.h"
#include "common/math.h"

namespace truck::motion_planner {

hull::Milestone::Milestone(
    size_t id, const geom::Pose& pose, double left_ledge, double right_ledge) noexcept :
    id{id}, pose{pose}, left_ledge{left_ledge}, right_ledge{right_ledge} {}

geom::Pose hull::Milestone::guide(double offset) const {
    VERIFY(Limits(-left_ledge, right_ledge).isMet(offset));
    return geom::Pose{.pos = (pose.dir.right().vec() * offset) + pose.pos, .dir = pose.dir};
}

geom::Poses hull::Milestone::getSpacedOutGuides(double spacing) const {
    VERIFY(spacing > 0);

    geom::Poses guides;

    const int k_left = left_ledge / spacing;
    const int k_right = right_ledge / spacing;

    for (int i = -k_left; i <= k_right; ++i) {
        guides.push_back(guide(i * spacing));
    }

    return guides;
}

std::pair<geom::Pose, geom::Pose> hull::Milestone::getEndpoints() const {
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
        const auto guides = milestone.getSpacedOutGuides(params_.node_spacing);
        for (const auto& guide : guides) {
            build.nodes.push_back(hull::Node{.id = build.nodes.size(), .pose = guide, .edges = {}});
            build.node_milestone_map.push_back(milestone.id);
        }
    }
}

void GraphBuilder::makeEdges(hull::GraphBuild& build) const {
    // not very good design. may be unstable

    auto it = std::find_if(
        build.node_milestone_map.begin(),
        build.node_milestone_map.begin(),
        [](size_t milestone_id) { return milestone_id = 1ULL; });

    // size_t next_idx =

    for (auto& node : build.nodes) {
    }
}

}  // namespace truck::motion_planner
