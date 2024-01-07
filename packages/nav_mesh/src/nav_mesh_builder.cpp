#include "nav_mesh/nav_mesh_builder.h"

#include "common/math.h"
#include "common/exception.h"

#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

namespace truck::nav_mesh {

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;

namespace {

CGAL::Polygon_2<CGAL_K> toCGALPolygon(const geom::Polygon& polygon) {
    CGAL::Polygon_2<CGAL_K> cgal_poly;

    for (const geom::Vec2& point : polygon) {
        cgal_poly.push_back(CGAL_K::Point_2(point.x, point.y));
    }
    
    return cgal_poly;
}

CGAL::Polygon_with_holes_2<CGAL_K> toCGALPolygonWithHoles(const geom::ComplexPolygon& polygon) {
    CGAL::Polygon_with_holes_2<CGAL_K> cgal_poly_with_holes(toCGALPolygon(polygon.outer));

    for (const geom::Polygon& inner_poly : polygon.inners) {
        cgal_poly_with_holes.add_hole(toCGALPolygon(inner_poly));
    }

    return cgal_poly_with_holes;
}

void extractSegmentsFromCGALPolygon(geom::Segments& segments, const CGAL::Polygon_2<CGAL_K>& cgal_poly) {
    for (const auto& cgal_seg : cgal_poly.edges()) {
        segments.emplace_back(geom::Segment(
            geom::Vec2(cgal_seg.vertex(0).x(), cgal_seg.vertex(0).y()),
            geom::Vec2(cgal_seg.vertex(1).x(), cgal_seg.vertex(1).y())));
    }
}

}  // namespace

NavMeshBuilder::NavMeshBuilder(const NavMeshParams& params) : params_(params) {}

NavMeshBuild NavMeshBuilder::build(const geom::ComplexPolygons& polygons) const {
    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    NavMeshBuild nav_mesh_build;

    buildSkeleton(nav_mesh_build, polygon);
    buildLevelLines(nav_mesh_build, polygon, params_.offset);
    buildMesh(nav_mesh_build, params_.dist);
    
    return nav_mesh_build;
}

void NavMeshBuilder::buildSkeleton(NavMeshBuild& build, const geom::ComplexPolygon& polygon) const {
    boost::shared_ptr<CGAL::Straight_skeleton_2<CGAL_K>> cgal_skeleton_ptr =
        CGAL::create_interior_straight_skeleton_2(toCGALPolygonWithHoles(polygon));

    for (const auto& cgal_edge_it : cgal_skeleton_ptr->halfedge_handles()) {
        build.skeleton.emplace_back(geom::Segment(
            geom::Vec2(
                cgal_edge_it->vertex()->point().x(),
                cgal_edge_it->vertex()->point().y()),
            geom::Vec2(
                cgal_edge_it->opposite()->vertex()->point().x(),
                cgal_edge_it->opposite()->vertex()->point().y())));
    }
}

void NavMeshBuilder::buildLevelLines(NavMeshBuild& build, const geom::ComplexPolygon& polygon, double offset) const {
    double cur_offset = offset;

    auto cgal_polys_with_holes = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
        cur_offset, toCGALPolygonWithHoles(polygon));

    while (cgal_polys_with_holes.size() > 0) {
        for (const auto& cgal_poly_with_holes_ptr : cgal_polys_with_holes) {
            extractSegmentsFromCGALPolygon(build.level_lines, cgal_poly_with_holes_ptr->outer_boundary());

            for (const auto& cgal_poly_inner : cgal_poly_with_holes_ptr->holes()) {
                extractSegmentsFromCGALPolygon(build.level_lines, cgal_poly_inner);
            }
        }

        cur_offset += offset;

        cgal_polys_with_holes = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
            cur_offset, toCGALPolygonWithHoles(polygon));
    }
}

void NavMeshBuilder::buildMesh(NavMeshBuild& build, double dist) const {
    for (const geom::Segment& seg : build.level_lines) {
        for (size_t i = 0; i < ceil<size_t>(seg.len() / dist); i++) {
            build.mesh.emplace_back(seg.pos(i * dist / seg.len()));
        }
    }

    if (params_.filter.grid) {
        gridFilter();
    }
}

void NavMeshBuilder::gridFilter() const {
    /** @todo
     * remove some points from 'build.mesh'
     */
}

}  // namespace truck::nav_mesh