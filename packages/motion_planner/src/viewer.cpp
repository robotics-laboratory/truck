#include "motion_planner/viewer.h"
#include "motion_planner/graph_builder.h"
#include "motion_planner/search.h"

#include "common/exception.h"
#include "geom/polyline.h"

#include <algorithm>
#include <unordered_set>

namespace truck::motion_planner::viewer {

CVDrawer::CVDrawer(const ViewerConfig& cfg, double res, const geom::ComplexPolygon& polygon) :
    res_{res},
    bbox_{cv::boundingRect(toCVPoints(geom::Vec2(0, 0), polygon.outer))},
    origin_{geom::Vec2(bbox_.x, bbox_.y)},
    frame_{cv::Mat(bbox_.size(), CV_8UC3, toCVScalar(cfg.background_color))} {}

cv::Scalar CVDrawer::toCVScalar(const rgb_t& color_rgb) const {
    VERIFY(color_rgb.size() == 3);
    auto r = static_cast<double>(color_rgb[0]);
    auto g = static_cast<double>(color_rgb[1]);
    auto b = static_cast<double>(color_rgb[2]);
    return {b, g, r};
}

cv::Point CVDrawer::toCVPoint(const geom::Vec2& point) const {
    return cv::Point(point.x * res_, point.y * res_) - cv::Point(origin_.x, origin_.y);
}

cv::Point CVDrawer::toCVPoint(const geom::Vec2& origin, const geom::Vec2& point) const {
    return cv::Point(point.x * res_, point.y * res_) - cv::Point(origin.x, origin.y);
}

std::vector<cv::Point> CVDrawer::toCVPoints(const std::vector<geom::Vec2>& points) const {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(point));
    }

    return cv_points;
}

std::vector<cv::Point> CVDrawer::toCVPoints(
    const geom::Vec2& origin, const std::vector<geom::Vec2>& points) const {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(origin, point));
    }

    return cv_points;
}

void CVDrawer::drawPoint(const geom::Vec2& point, const DrawerParam& param) {
    cv::circle(frame_, toCVPoint(point), param.thickness, toCVScalar(param.rgb), cv::FILLED);
}

void CVDrawer::drawPoints(const std::vector<geom::Vec2>& points, const DrawerParam& param) {
    for (const auto& point : points) {
        drawPoint(point, param);
    }
}

void CVDrawer::drawSegment(const geom::Segment& seg, const DrawerParam& param) {
    cv::line(
        frame_, toCVPoint(seg.begin), toCVPoint(seg.end), toCVScalar(param.rgb), param.thickness);
}

void CVDrawer::drawPolyline(const geom::Polyline& polyline, const DrawerParam& param) {
    for (size_t i = 1; i < polyline.size(); i++) {
        drawSegment(geom::Segment{polyline[i - 1], polyline[i]}, param);
    }
}

void CVDrawer::fillPoly(
    const geom::ComplexPolygon& polygon, const rgb_t& inner_rgb, const rgb_t& outer_rgb) {
    cv::fillPoly(frame_, toCVPoints(polygon.outer), toCVScalar(outer_rgb));

    for (const geom::Polygon& inner : polygon.inners) {
        cv::fillPoly(frame_, toCVPoints(inner), toCVScalar(inner_rgb));
    }
}

void CVDrawer::dump(const std::string& filename) {
    /**
     * Rotate an image around 'x' axis to avoid .png map mirroring
     * relative to the x-axis of the .geojson map
     */
    cv::flip(frame_, frame_, 0);
    cv::imwrite(filename, frame_);
}

namespace {

void drawMilestones(const ViewerConfig& cfg, CVDrawer& drawer, const hull::Milestones& milestones) {
    for (const auto& milestone : milestones) {
        const geom::Segment segment = milestone.toSegment();
        drawer.drawSegment(segment, cfg.hull.milestones);
    }
}

void drawBounds(const ViewerConfig& cfg, CVDrawer& drawer, const hull::Milestones& milestones) {
    if (milestones.size() <= 1) {
        return;
    }

    for (size_t i = 1; i < milestones.size(); ++i) {
        const auto [pl, pr] = milestones[i - 1].toSegment();
        const auto [cl, cr] = milestones[i].toSegment();

        drawer.drawSegment(geom::Segment{pl, cl}, cfg.hull.bounds);
        drawer.drawSegment(geom::Segment{pr, cr}, cfg.hull.bounds);
    }
}

void drawEdges(const ViewerConfig& cfg, CVDrawer& drawer, const hull::Graph& graph) {
    for (const hull::Edge& edge : graph.edges) {
        const geom::Vec2& from = graph.nodes[edge.from].pose.pos;
        const geom::Vec2& to = graph.nodes[edge.to].pose.pos;

        drawer.drawSegment(geom::Segment{from, to}, cfg.graph.edges);
    }
}

void drawNodes(
    const ViewerConfig& cfg, CVDrawer& drawer, const hull::Graph& graph,
    const std::vector<bool>& occupancy_grid) {
    std::unordered_set<NodeId> occupied_nodes;

    for (std::size_t i = 0; i < occupancy_grid.size(); ++i) {
        if (occupancy_grid[i] == true) {
            occupied_nodes.insert(i);
        }
    }

    for (const hull::Node& node : graph.nodes) {
        drawer.drawPoint(
            node.pose.pos,
            occupied_nodes.contains(node.id) ? cfg.graph.occupied_nodes : cfg.graph.nodes);
    }
}

}  // namespace

Viewer::Viewer(const ViewerConfig& params, const geom::ComplexPolygon& polygon) :
    cfg_(params), drawer_{params, params.res, polygon} {}

void Viewer::addPolygon(const geom::ComplexPolygon& polygon) {
    drawer_.fillPoly(polygon, cfg_.polygon.inner, cfg_.polygon.outer);
}

void Viewer::addHull(const Reference& reference, const hull::GraphContext& graph_context) {
    geom::Polyline polyline(reference.Points().size());

    std::transform(
        reference.Points().begin(),
        reference.Points().end(),
        polyline.begin(),
        [](const geom::Pose& pose) -> geom::Vec2 { return pose.pos; });

    drawer_.drawPolyline(polyline, cfg_.motion.reference);

    drawMilestones(cfg_, drawer_, graph_context.milestones);
    drawBounds(cfg_, drawer_, graph_context.milestones);
}

void Viewer::addGraph(const hull::Graph& graph, const std::vector<bool>& occupancy_grid) {
    drawEdges(cfg_, drawer_, graph);
    drawNodes(cfg_, drawer_, graph, occupancy_grid);
}

void Viewer::addMotion(
    const search::Path& path, const geom::MotionStates& trajectory, const hull::Graph& graph) {
    geom::Polyline path_polyline(path.trace.size());
    geom::Polyline trajectory_polyline(trajectory.size());

    std::transform(
        path.trace.cbegin(),
        path.trace.cend(),
        path_polyline.begin(),
        [&graph](const NodeId& node_id) { return graph.nodes.at(node_id).pose.pos; });

    std::transform(
        trajectory.cbegin(),
        trajectory.cend(),
        trajectory_polyline.begin(),
        [&graph](const geom::MotionState& mstate) { return mstate.pos; });

    drawer_.drawPolyline(path_polyline, cfg_.motion.path);
    drawer_.drawPolyline(trajectory_polyline, cfg_.motion.trajectory);
}

void Viewer::draw() { drawer_.dump(cfg_.path); }

}  // namespace truck::motion_planner::viewer
