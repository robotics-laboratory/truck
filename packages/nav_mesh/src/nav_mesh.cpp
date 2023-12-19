#include "nav_mesh/nav_mesh.h"

#include "common/exception.h"

#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

namespace truck::nav_mesh {

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;

namespace {

cv::Scalar toCVScalar(const std::vector<int> color) {
    VERIFY(color.size() == 3);
    return cv::Scalar(color[2], color[1], color[0]);
}

void drawSimplePolygon(const geom::Polygon& poly, const cv::Mat& frame, double scale, const cv::Scalar& color, bool fill) {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : poly.points) {
        cv_points.push_back(cv::Point(point.x * scale, point.y * scale));
    }

    if (fill) { cv::fillConvexPoly(frame, cv_points, color); }
    else { cv::polylines(frame, cv_points, 1, color); }
}

CGAL::Polygon_with_holes_2<CGAL_K> toCGALPolygonWithHoles(const geom::ComplexPolygon& poly) {
    CGAL::Polygon_2<CGAL_K> cgal_poly_outer;

    for (const geom::Vec2& point : poly.outer_poly.points) {
        cgal_poly_outer.push_back(CGAL_K::Point_2(point.x, point.y));
    }

    CGAL::Polygon_with_holes_2<CGAL_K> cgal_poly_with_holes(cgal_poly_outer);

    for (const geom::Polygon& inner_poly : poly.inner_polys) {
        CGAL::Polygon_2<CGAL_K> cgal_poly_inner;

        for (const geom::Vec2& point : inner_poly.points) {
            cgal_poly_inner.push_back(CGAL_K::Point_2(point.x, point.y));
        }

        cgal_poly_with_holes.add_hole(cgal_poly_inner);
    }

    return cgal_poly_with_holes;
}

}

NavMesh::NavMesh(const NavMeshParams& params) : params_(params) {}

NavMesh& NavMesh::setPolygons(const geom::ComplexPolygons& polygons) {
    VERIFY(polygons.size() == 1);
    obj_.polygons = polygons;
    return *this;
}

NavMesh& NavMesh::build() {
    buildStraightSkeleton();
    buildOffsetPolygons();
    buildMesh();
    return *this;
}

void NavMesh::buildStraightSkeleton() {
    const auto& poly = obj_.polygons[0];

    boost::shared_ptr<CGAL::Straight_skeleton_2<CGAL_K>> cgal_ss_ptr = \
        CGAL::create_interior_straight_skeleton_2(toCGALPolygonWithHoles(poly));

    for (const auto& cgal_edge_it : cgal_ss_ptr->halfedge_handles()) {
        CGAL_K::Point_2 cgal_p1 = cgal_edge_it->vertex()->point();
        CGAL_K::Point_2 cgal_p2 = cgal_edge_it->opposite()->vertex()->point();

        geom::Segment edge(
            geom::Vec2(cgal_p1.x(), cgal_p1.y()), 
            geom::Vec2(cgal_p2.x(), cgal_p2.y())
        );

        obj_.straight_skeleton.push_back(edge);
    }
}

void NavMesh::buildOffsetPolygons() {
    /** @todo
     * store in 'obj_.offset_polygons' 
     */
}

void NavMesh::buildMesh() {
    /** @todo
     * store in 'obj_.mesh' 
    */
}

const std::vector<geom::Vec2>& NavMesh::mesh() const { return obj_.mesh; }

void NavMesh::drawMesh(const DrawParams& draw_params) const {
    /** @todo */
}

void NavMesh::drawPolygons(const DrawParams& draw_params) const {
    const auto& poly = obj_.polygons[0];
    
    drawSimplePolygon(
        poly.outer_poly,
        frame_,
        draw_params.size.scale,
        toCVScalar(draw_params.color.outer_poly),
        true
    );

    for (const geom::Polygon& inner_poly : poly.inner_polys) {
        drawSimplePolygon(
            inner_poly,
            frame_,
            draw_params.size.scale,
            toCVScalar(draw_params.color.inner_poly),
            true
        );
    }
}

void NavMesh::drawOffsetPolygons(const DrawParams& draw_params) const {
    /** @todo */
}

void NavMesh::drawStraightSkeleton(const DrawParams& draw_params) const {
    for (const geom::Segment& edge : obj_.straight_skeleton) {
        cv::Point p1(edge.begin.x * draw_params.size.scale, edge.begin.y * draw_params.size.scale);
        cv::Point p2(edge.end.x * draw_params.size.scale, edge.end.y * draw_params.size.scale);
        cv::line(frame_, p1, p2, toCVScalar(draw_params.color.straight_skeleton));
    }
}

void NavMesh::draw(const DrawParams& draw_params) {
    frame_ = cv::Mat(
        draw_params.size.pixels,
        draw_params.size.pixels,
        CV_8UC3,
        toCVScalar(draw_params.color.background));

    if (draw_params.show.mesh) { drawMesh(draw_params); }
    if (draw_params.show.polys) { drawPolygons(draw_params); }
    if (draw_params.show.offset_polys) { drawOffsetPolygons(draw_params); }
    if (draw_params.show.straight_skeleton) { drawStraightSkeleton(draw_params); }

    cv::imwrite(draw_params.path, frame_);
}

}  // namespace truck::nav_mesh