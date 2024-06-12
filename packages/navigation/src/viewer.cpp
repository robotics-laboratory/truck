#include "navigation/viewer.h"

#include "common/exception.h"

namespace truck::navigation::viewer {

namespace {

cv::Scalar toCVScalar(const std::vector<int>& color_rgb) {
    VERIFY(color_rgb.size() == 3);
    return cv::Scalar(color_rgb[2], color_rgb[1], color_rgb[0]);
}

cv::Point toCVPoint(const geom::Vec2& origin, double res, const geom::Vec2& point) {
    return cv::Point(point.x * res, point.y * res) - cv::Point(origin.x, origin.y);
}

std::vector<cv::Point> toCVPoints(
    const geom::Vec2& origin, double res, const std::vector<geom::Vec2>& points) {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(origin, res, point));
    }

    return cv_points;
}

void drawPolygon(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::ComplexPolygon& polygon) {
    cv::fillPoly(
        frame,
        toCVPoints(origin, params.res, polygon.outer),
        toCVScalar(params.color_rgb.polygon.outer));

    for (const geom::Polygon& inner : polygon.inners) {
        cv::fillPoly(
            frame,
            toCVPoints(origin, params.res, inner),
            toCVScalar(params.color_rgb.polygon.inner));
    }
}

void drawSkeleton(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::Segments& skeleton) {
    for (const geom::Segment& seg : skeleton) {
        cv::line(
            frame,
            toCVPoint(origin, params.res, seg.begin),
            toCVPoint(origin, params.res, seg.end),
            toCVScalar(params.color_rgb.mesh_build.skeleton),
            params.thickness.mesh_build.skeleton);
    }
}

void drawLevelLines(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::Segments& level_lines) {
    for (const geom::Segment& seg : level_lines) {
        cv::line(
            frame,
            toCVPoint(origin, params.res, seg.begin),
            toCVPoint(origin, params.res, seg.end),
            toCVScalar(params.color_rgb.mesh_build.level_lines),
            params.thickness.mesh_build.level_lines);
    }
}

void drawMesh(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const std::vector<geom::Vec2>& mesh) {
    for (const geom::Vec2& point : mesh) {
        cv::circle(
            frame,
            toCVPoint(origin, params.res, point),
            params.thickness.mesh_build.mesh,
            toCVScalar(params.color_rgb.mesh_build.mesh),
            cv::FILLED);
    }
}

void drawEdges(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const graph::Graph& graph) {
    for (const graph::Edge& edge : graph.edges) {
        const geom::Vec2& from = graph.nodes[edge.from].point;
        const geom::Vec2& to = graph.nodes[edge.to].point;
        cv::line(
            frame,
            toCVPoint(origin, params.res, from),
            toCVPoint(origin, params.res, to),
            toCVScalar(params.color_rgb.graph.edges),
            params.thickness.graph.edges);
    }
}

void drawNodes(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const graph::Graph& graph) {
    for (const graph::Node& node : graph.nodes) {
        cv::circle(
            frame,
            toCVPoint(origin, params.res, node.point),
            params.thickness.graph.nodes,
            toCVScalar(params.color_rgb.graph.nodes),
            cv::FILLED);
    }
}

void drawPath(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::Polyline& path) {
    if (path.empty()) {
        return;
    }

    for (size_t i = 0; i < path.size() - 1; i++) {
        const geom::Vec2& from = path[i];
        const geom::Vec2& to = path[i + 1];
        cv::line(
            frame,
            toCVPoint(origin, params.res, from),
            toCVPoint(origin, params.res, to),
            toCVScalar(params.color_rgb.path),
            params.thickness.path);
    }
}

}  // namespace

Viewer::Viewer(const ViewerParams& params, const geom::ComplexPolygon& polygon) : params_(params) {
    bbox_ = cv::boundingRect(toCVPoints(geom::Vec2(0, 0), params.res, polygon.outer));
    bbox_origin_ = geom::Vec2(bbox_.x, bbox_.y);

    frame_ = cv::Mat(bbox_.size(), CV_8UC3, toCVScalar(params.color_rgb.background));
}

void Viewer::addPolygon(const geom::ComplexPolygon& polygon) {
    drawPolygon(params_, bbox_origin_, frame_, polygon);
}

void Viewer::addMeshBuild(const navigation::mesh::MeshBuild& mesh_build) {
    drawMesh(params_, bbox_origin_, frame_, mesh_build.mesh);
    drawLevelLines(params_, bbox_origin_, frame_, mesh_build.level_lines);
    drawSkeleton(params_, bbox_origin_, frame_, mesh_build.skeleton);
}

void Viewer::addGraph(const navigation::graph::Graph& graph) {
    drawEdges(params_, bbox_origin_, frame_, graph);
    drawNodes(params_, bbox_origin_, frame_, graph);
}

void Viewer::addPath(const geom::Polyline& path) { drawPath(params_, bbox_origin_, frame_, path); }

void Viewer::draw() {
    /**
     * Rotate an image around 'x' axis to avoid .png map mirroring
     * relative to the x-axis of the .geojson map
     */
    cv::flip(frame_, frame_, 0);
    cv::imwrite(params_.path, frame_);
}

}  // namespace truck::navigation::viewer
