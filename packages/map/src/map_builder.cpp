#include "map/map_builder.h"

#include "common/exception.h"

#include "geom/line.h"

#include <iostream>

#include <nlohmann/json.hpp>

#include <fstream>

namespace truck::map {

Map Map::fromGeoJson(const std::string& path) {
    geom::ComplexPolygons polygons;
    nlohmann::json geojson_features = nlohmann::json::parse(std::ifstream(path))["features"];

    // iterate through every complex polygon
    for (const auto& elem : geojson_features) {
        // get list of outer (0 index) and inner polygons (1+ index) for a current complex polygon
        auto polys_list = elem["geometry"]["coordinates"]
                              .get<std::vector<std::vector<std::pair<double, double>>>>();

        int polys_cnt = 0;
        geom::Polygon outer;
        geom::Polygons inners;

        for (size_t i = 0; i < polys_list.size(); i++) {
            geom::Polygon simple_poly;
            for (const auto& point : polys_list[i]) {
                VERIFY(point.first >= 0 && point.second >= 0);
                simple_poly.push_back(geom::Vec2(point.first, point.second));
            }

            if (polys_cnt == 0) {
                outer = std::move(simple_poly);
            } else {
                inners.push_back(std::move(simple_poly));
            }

            polys_cnt++;
        }

        polygons.push_back(geom::ComplexPolygon(std::move(outer), std::move(inners)));
    }

    return Map(std::move(polygons));
}

Map::Map(geom::ComplexPolygons polygons) : polygons_(std::move(polygons)) {}

const geom::ComplexPolygons& Map::polygons() const { return polygons_; }

geom::ComplexPolygons Map::clip(const fastgrid::Domain& domain) const noexcept {
    geom::ComplexPolygons clipped_polygons;
    clipped_polygons.reserve(polygons_.size());
    for (const auto& polygon : polygons_) {
        geom::ComplexPolygon clipped_polygon;
        clipped_polygon.outer = clip(domain, polygon.outer);
        clipped_polygon.inners.reserve(polygon.inners.size());
        for (const auto& inner : polygon.inners) {
            clipped_polygon.inners.emplace_back(clip(domain, inner));
        }
        clipped_polygons.push_back(clipped_polygon);
    }
    return clipped_polygons;
}

geom::Polygon Map::clip(
    const fastgrid::Domain& domain, const geom::Polygon& polygon) const noexcept {
    const double x_min = 0.0;
    const double y_min = 0.0;
    const double x_max = domain.resolution * domain.size.width;
    const double y_max = domain.resolution * domain.size.height;

    const geom::Polygon clip_polygon = {
        geom::Vec2(x_min, y_min),
        geom::Vec2(x_max, y_min),
        geom::Vec2(x_max, y_max),
        geom::Vec2(x_min, y_max)};

    geom::Polygon clipped_polygon = polygon;
    for (auto& vertex : clipped_polygon) {
        vertex = domain.Transform(vertex);
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
            std::cerr << clip_vertex_1 << ' ' << clip_vertex_2 << ' ' << current_vertex_1 << ' '
                      << current_vertex_2 << ' ';
            if (intersection_point) {
                std::cerr << *intersection_point;
            }
            std::cerr << std::endl;
            if (cross(current_vertex_2 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) <= 0) {
                if (cross(current_vertex_1 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) > 0) {
                    clipped_polygon.push_back(*intersection_point);
                }
                clipped_polygon.push_back(current_vertex_2);
            } else if (cross(current_vertex_1 - clip_vertex_1, clip_vertex_2 - clip_vertex_1) < 0) {
                clipped_polygon.push_back(*intersection_point);
            }
            current_vertex_1 = current_vertex_2;
        }
        clip_vertex_1 = clip_vertex_2;
        std::cerr << std::endl;
    }

    const auto inv_tf = domain.origin.tf.inv();
    for (auto& vertex : clipped_polygon) {
        vertex = inv_tf(vertex);
    }

    return clipped_polygon;
}

}  // namespace truck::map