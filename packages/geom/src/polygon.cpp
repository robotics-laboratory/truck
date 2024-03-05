#include "geom/polygon.h"

#include "common/exception.h"
#include "common/math.h"

#include "geom/line.h"
#include "geom/intersection.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>

#include <unordered_map>
#include <boost/property_map/property_map.hpp>

namespace truck::geom {

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGAL_Vb = CGAL::Triangulation_vertex_base_2<CGAL_K>;
using CGAL_Fb = CGAL::Constrained_triangulation_face_base_2<CGAL_K>;
using CGAL_TDS = CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>;
using CGAL_Itag = CGAL::Exact_predicates_tag;
using CGAL_CDT = CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_TDS, CGAL_Itag>;
using CGAL_Face_handle = CGAL_CDT::Face_handle;
using CGAL_Point = CGAL_CDT::Point;
using CGAL_Polygon = CGAL::Polygon_2<CGAL_K>;

size_t Polygon::segmentNumber() const noexcept {
    return this->size() - 1;
}

Segment Polygon::segment(size_t i) const noexcept {
    VERIFY(i < segmentNumber());
    auto it = this->begin();
    return Segment(*(it + i), *(it + i + 1));
}

std::vector<Triangle> Polygon::triangles() const noexcept {
    CGAL_CDT cgal_cdt;
    std::vector<Triangle> triangles;

    CGAL_Polygon cgal_polygon;
    for (const Vec2& point : *this) {
        cgal_polygon.push_back(CGAL_Point(point.x, point.y));
    }

    cgal_cdt.insert_constraint(cgal_polygon.vertices_begin(), cgal_polygon.vertices_end(), true);

    std::unordered_map<CGAL_Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::unordered_map<CGAL_Face_handle, bool>> in_domain(
        in_domain_map);
    CGAL::mark_domain_in_triangulation(cgal_cdt, in_domain);

    for (const auto& cgal_face_it : cgal_cdt.finite_face_handles()) {
        if (get(in_domain, cgal_face_it)) {
            CGAL_Point p1 = cgal_face_it->vertex(0)->point();
            CGAL_Point p2 = cgal_face_it->vertex(1)->point();
            CGAL_Point p3 = cgal_face_it->vertex(2)->point();

            triangles.emplace_back(
                Triangle(Vec2(p1.x(), p1.y()), Vec2(p2.x(), p2.y()), Vec2(p3.x(), p3.y())));
        }
    }

    return triangles;
}

bool Polygon::isConvex() const noexcept {
    auto side_1 = *(this->end() - 1) - *(this->end() - 2);
    auto side_2 = *(this->begin()) - *(this->end() - 1);
    auto expected_sign = sign(cross(side_1, side_2));
    for (auto it = this->begin(); it + 1 != this->end(); ++it) {
        side_1 = side_2;
        side_2 = *(it + 1) - *it;
        auto current_sign = sign(cross(side_1, side_2));
        if (expected_sign != current_sign) {
            return false;
        }
    }
    return true;
}

Orientation Polygon::orientation() const noexcept {
    auto it =
        std::min_element(this->cbegin(), this->cend(), [](const auto& v1, const auto& v2) -> bool {
            return (v1.x == v2.x ? v1.y < v2.y : v1.x < v2.x);
        });
    auto prev_it = (it == this->cbegin() ? this->cend() - 1 : it - 1);
    auto next_it = (it + 1 == this->cend() ? this->cbegin() : it + 1);
    auto side_1 = *it - *prev_it;
    auto side_2 = *next_it - *it;
    int orientation_sign = sign(cross(side_1, side_2));
    VERIFY(orientation_sign != 0);
    return (orientation_sign > 0 ? Orientation::COUNTERCLOCKWISE : Orientation::CLOCKWISE);
}

/** Implementation of Sutherland–Hodgman algorithm
 *
 * See https://en.wikipedia.org/wiki/Sutherland–Hodgman_algorithm
 */
Polygon clip(
    const Polygon& clip_polygon, const Polygon& subject_polygon, const double eps) noexcept {
    VERIFY(clip_polygon.isConvex());

    int orientation_sign =
        sign(cross(clip_polygon[1] - clip_polygon[0], clip_polygon[2] - clip_polygon[1]));

    auto clipped_polygon = subject_polygon;

    auto clip_vertex_1 = clip_polygon.back();
    for (const auto& clip_vertex_2 : clip_polygon) {
        auto clip_edge = clip_vertex_2 - clip_vertex_1;

        auto is_inside_clip_edge =
            [eps, orientation_sign, clip_vertex_2, clip_edge](const auto& point) -> bool {
            return orientation_sign * cross(clip_edge, point - clip_vertex_2) >= eps;
        };

        geom::Polygon current_polygon = std::move(clipped_polygon);
        clipped_polygon.clear();
        auto current_vertex_1 = current_polygon.back();
        for (const auto& current_vertex_2 : current_polygon) {
            auto intersection_point = geom::intersect(
                geom::Line::fromTwoPoints(current_vertex_1, current_vertex_2),
                geom::Line::fromTwoPoints(clip_vertex_1, clip_vertex_2));
            if (is_inside_clip_edge(current_vertex_2)) {
                if (!is_inside_clip_edge(current_vertex_1)) {
                    clipped_polygon.push_back(*intersection_point);
                }
                clipped_polygon.push_back(current_vertex_2);
            } else if (is_inside_clip_edge(current_vertex_1)) {
                clipped_polygon.push_back(*intersection_point);
            }
            current_vertex_1 = current_vertex_2;
        }
        clip_vertex_1 = clip_vertex_2;
    }

    return clipped_polygon;
}

Segments Polygon::segments() const noexcept {
    const auto points = *this;
    Segments segments;
    segments.reserve(points.size());

    segments[0].begin = {points.back().x, points.back().y};
    segments[0].end = {points[0].x, points[0].y};

    for (size_t i = 1; i < points.size(); ++i) {
        Vec2 begin = {points[i - 1].x, points[i - 1].y};
        Vec2 end = {points[i].x, points[i].y};
        segments.emplace_back(begin, end);
    }

    return segments;
}

}  // namespace truck::geom