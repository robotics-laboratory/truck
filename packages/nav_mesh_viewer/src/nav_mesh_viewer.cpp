#include "nav_mesh_viewer/nav_mesh_viewer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/exception.h"

namespace truck::nav_mesh_viewer {

namespace {

cv::Scalar toCVScalar(const std::vector<int>& color) {
    VERIFY(color.size() == 3);
    return cv::Scalar(color[2], color[1], color[0]);
}

cv::Point toCVPoint(const geom::Vec2& point, const geom::Vec2& origin, double res) {
    return cv::Point(point.x * res, point.y * res) - cv::Point(origin.x, origin.y);
}

std::vector<cv::Point> toCVPoints(
    const std::vector<geom::Vec2>& points, const geom::Vec2& origin, double res) {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(point, origin, res));
    }

    return cv_points;
}

void drawPolygon(
    const geom::ComplexPolygon& polygon, const NavMeshViewerParams& params,
    const geom::Vec2& origin, cv::Mat& frame) {
    cv::fillConvexPoly(
        frame,
        toCVPoints(polygon.outer, origin, params.res),
        toCVScalar(params.color.outer_polygon));

    for (const geom::Polygon& inner : polygon.inners) {
        cv::fillConvexPoly(
            frame,
            toCVPoints(inner, origin, params.res),
            toCVScalar(params.color.inner_polygon));
    }
}

void drawSkeleton(
    const geom::Segments& skeleton, const NavMeshViewerParams& params,
    const geom::Vec2& origin, cv::Mat& frame) {
    for (const geom::Segment& seg : skeleton) {
        cv::line(
            frame,
            toCVPoint(seg.begin, origin, params.res),
            toCVPoint(seg.end, origin, params.res),
            toCVScalar(params.color.skeleton),
            params.thickness.skeleton);
    }
}

void drawLevelLines(
    const geom::Segments& level_lines, const NavMeshViewerParams& params,
    const geom::Vec2& origin, cv::Mat& frame) {
    for (const geom::Segment& seg : level_lines) {
        cv::line(
            frame,
            toCVPoint(seg.begin, origin, params.res),
            toCVPoint(seg.end, origin, params.res),
            toCVScalar(params.color.level_lines),
            params.thickness.level_lines);
    }
}

void drawMesh(
    const std::vector<geom::Vec2>& mesh, const NavMeshViewerParams& params,
    const geom::Vec2& origin, cv::Mat& frame) {
    for (const geom::Vec2& point : mesh) {
        cv::circle(
            frame,
            toCVPoint(point, origin, params.res),
            params.thickness.mesh,
            toCVScalar(params.color.mesh),
            cv::FILLED);
    }
}

}  // namespace

NavMeshViewer::NavMeshViewer(const NavMeshViewerParams& params) : params_(params) {}

void NavMeshViewer::draw(
    const geom::ComplexPolygons& polygons, const nav_mesh::NavMeshBuild& nav_mesh_build) {
    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    // set image borders via outer polygon's borders
    cv::Rect bb = cv::boundingRect(toCVPoints(polygon.outer, geom::Vec2(0, 0), params_.res));
    cv::Mat frame = cv::Mat(bb.size(), CV_8UC3, toCVScalar(params_.color.background));

    geom::Vec2 bb_origin(bb.x, bb.y);

    if (params_.enable.polygon) {
        drawPolygon(polygon, params_, bb_origin, frame);
    }

    if (params_.enable.skeleton) {
        drawSkeleton(nav_mesh_build.skeleton, params_, bb_origin, frame);
    }

    if (params_.enable.level_lines) {
        drawLevelLines(nav_mesh_build.level_lines, params_, bb_origin, frame);
    }

    if (params_.enable.mesh) {
        drawMesh(nav_mesh_build.mesh, params_, bb_origin, frame);
    }

    cv::flip(frame, frame, 0);
    cv::imwrite(params_.path, frame);
}

}  // namespace truck::nav_mesh_viewer