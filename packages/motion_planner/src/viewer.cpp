#include "motion_planner/viewer.h"
#include "motion_planner/motion_planner.h"

#include "common/exception.h"

namespace truck::motion_planner::viewer {

CVDrawer::CVDrawer(const ViewerConfig& cfg, double res, cv::Rect bbox) :
    res_{res},
    bbox_{bbox},
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

std::vector<cv::Point> CVDrawer::toCVPoints(const std::vector<geom::Vec2>& points) const {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(point));
    }

    return cv_points;
}

void CVDrawer::drawPoint(const geom::Vec2& point, const DrawerParam& param) {
    cv::circle(frame_, toCVPoint(point), param.thickness, toCVScalar(param.rgb), cv::FILLED);
}

void CVDrawer::drawPoints(const std::vector<geom::Vec2>& points, const DrawerParam& param) {
    for (const auto& point : points) {
        drawPoint(point, param.rgb, param.thickness);
    }
}

void CVDrawer::drawSegment(const geom::Segment& seg, const DrawerParam& param) {
    cv::line(
        frame_, toCVPoint(seg.begin), toCVPoint(seg.end), thickness, toCVScalar(rgb), cv::FILLED)
}

void CVDrawer::drawPolyline(const geom::Polyline& polyline, const DrawerParam& param) {
    for (size_t i = 1; i < path.size(); i++) {
        drawer_.drawSegment(geom::Segment{path[i - 1], path[i]}, param.rgb, param.thickness);
    }
}

void CVDrawer::fillPoly(
    const geom::ComplexPolygon& polygon, const rgb_t& inner_rgb, const rgb_t& outer_rgb) {
    cv::fillPoly(frame_, toCVPoints(polygon.outer), toCVScalar(outer_rgb));

    for (const geom::Polygon& inner : polygon.inners) {
        cv::fillPoly(frame_, toCVPoints(inner), toCVScalar(inner_rgb));
    }
}

namespace {

void drawMilestones(const ViewerConfig& cfg, CVDrawer& drawer, const hull::Milestones& milestones) {
    for (const auto& milestone : milestones) {
        drawer.drawSegment(milestone.toSegment(), cfg.hull.milestones);
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

void drawNodes(const ViewerConfig& cfg, CVDrawer& drawer, const graph::Graph& graph) {
    for (const graph::Node& node : graph.nodes) {
        drawer.drawPoint(node.pose.pos, cfg.graph.nodes);
    }
}

}  // namespace

Viewer::Viewer(const ViewerConfig& params, const geom::ComplexPolygon& polygon) :
    cfg_(params),
    drawer_{params.res, cv::boundingRect(toCVPoints(geom::Vec2(0, 0), params.res, polygon.outer))} {
}

void Viewer::addPolygon(const geom::ComplexPolygon& polygon) {
    drawer_.fillPoly(polygon, cfg_.polygon.inner, cfg_.polygon.outer);
}

void Viewer::addHull(const hull::GraphBuild& graph_build) {
    drawer_.drawPolyline(
        graph_build.reference.Points(),
        cfg_.color_rgb.motion.reference,
        cfg_.thickness.motion.reference);

    drawMilestones(cfg_, graph_build.milestones);
    drawBounds(cfg_, graph_build.milestones);
}

void Viewer::addGraph(const navigation::graph::Graph& graph) {
    drawEdges(cfg_, graph);
    drawNodes(cfg_, graph);
}

void Viewer::addMotion(const hull::TrajectoryBuild& trajectory_build) {
    drawer_.drawPolyline(trajectory_build.path, cfg_.motion.path);
    drawer_.drawPolyline(trajectory_build.trajectory, cfg_.motion.trajectory);
}

void Viewer::draw() {
    /**
     * Rotate an image around 'x' axis to avoid .png map mirroring
     * relative to the x-axis of the .geojson map
     */
    cv::flip(frame_, frame_, 0);
    cv::imwrite(cfg_.path, frame_);
}

}  // namespace truck::motion_planner::viewer
