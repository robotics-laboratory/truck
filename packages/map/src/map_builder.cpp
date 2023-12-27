#include "map/map_builder.h"

#include "common/exception.h"

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

}  // namespace truck::map