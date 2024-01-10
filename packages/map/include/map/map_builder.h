#pragma once

#include "geom/complex_polygon.h"
#include "geom/line.h"

#include "fastgrid/grid.h"

namespace truck::map {

class Map {
  public:
    Map(geom::ComplexPolygons polygons);

    static Map fromGeoJson(const std::string& path);

    const geom::ComplexPolygons& polygons() const;

    template<typename T>
    geom::ComplexPolygons clip(const fastgrid::Grid<T>& grid) const noexcept {
        geom::ComplexPolygons clipped_polygons;
        clipped_polygons.reserve(polygons_.size());
        for (const auto& polygon : polygons_) {
            geom::ComplexPolygon clipped_polygon;
            clipped_polygon.outer = clip(grid, polygon.outer);
            clipped_polygon.inners.reserve(polygon.inners.size());
            for (const auto& inner : polygon.inners) {
                clipped_polygon.inners.emplace_back(clip(grid, inner));
            }
            clipped_polygons.push_back(clipped_polygon);
        }
        return clipped_polygons;
    }

  private:
    template<typename T>
    geom::Polygon clip(const fastgrid::Grid<T>& grid, const geom::Polygon& polygon) const noexcept {
        const double x_min = 0.0;
        const double y_min = 0.0;
        const double x_max = grid.resolution * grid.size.width;
        const double y_max = grid.resolution * grid.size.height;

        const geom::Polygon clip_polygon = {
            geom::Vec2(x_min, y_min),
            geom::Vec2(x_max, y_min),
            geom::Vec2(x_max, y_max),
            geom::Vec2(x_min, y_max)};

        geom::Polygon clipped_polygon = polygon;
        for (auto& vertex : clipped_polygon) {
            vertex = grid.transform(vertex);
        }

        auto clip_vertex_1 = clip_polygon.back();
        for (const auto& clip_vertex_2 : clip_polygon) {
            geom::Polygon current_polygon = clipped_polygon;
            clipped_polygon.clear();
            auto current_vertex_1 = current_polygon.back();
            for (const auto& vertex : current_polygon) {
                auto current_vertex_2 = vertex;
                auto intersection_point = geom::intersect(
                    geom::Line::fromTwoPoints(current_vertex_1, current_vertex_2),
                    geom::Line::fromTwoPoints(clip_vertex_1, clip_vertex_2));
                if (cross(current_vertex_2 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) <= 0) {
                    if (cross(current_vertex_1 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) >
                        0) {
                        clipped_polygon.push_back(*intersection_point);
                    }
                    clipped_polygon.push_back(current_vertex_2);
                } else if (
                    cross(current_vertex_1 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) < 0) {
                    clipped_polygon.push_back(*intersection_point);
                }
                current_vertex_1 = current_vertex_2;
            }
            clip_vertex_1 = clip_vertex_2;
        }

        const auto inv_tf = grid.origin->tf.inv();
        for (auto& vertex : clipped_polygon) {
            vertex = inv_tf(vertex);
        }

        return clipped_polygon;
    }

    geom::ComplexPolygons polygons_;
};

}  // namespace truck::map