#include "map/map_builder.h"

#include <nlohmann/json.hpp>

#include <fstream>

namespace truck::map {

Map Map::fromGeoJson(const std::string& path) {
    geom::ComplexPolygons polygons;
    nlohmann::json geojson_features = nlohmann::json::parse(std::ifstream(path))["features"];

    // iterate through every complex polygon
    for (const auto& elem : geojson_features) {
        // get list of outer (0 index) and inner polygons (1+ index) for a current complex polygon
        auto polys_list = elem["geometry"]["coordinates"] \
            .get<std::vector<std::vector<std::pair<double, double>>>>();

        int polys_cnt = 0;
        geom::Polygon outer_poly;
        geom::Polygons inner_polys;

        for (const auto& points_list : polys_list) {
            std::vector<geom::Vec2> points;
            for (const auto& point : points_list) {
                points.push_back(geom::Vec2(point.first, point.second));
            }

            if (polys_cnt == 0) {
                outer_poly = geom::Polygon(points);
            } else {
                inner_polys.push_back(geom::Polygon(points));
            }

            polys_cnt++;
        }

        polygons.push_back(geom::ComplexPolygon(outer_poly, inner_polys));
    }

    return Map(polygons);
}

Map::Map(const geom::ComplexPolygons& polygons) : polygons_(polygons) {}

const geom::ComplexPolygons& Map::polygons() const { return polygons_; }

}  // namespace truck::map