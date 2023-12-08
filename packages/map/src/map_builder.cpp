#include "map/map_builder.h"

#include <nlohmann/json.hpp>

#include <fstream>

namespace truck::map {

Map::Map(const std::string& path) : path_(path) { parseGeoJSON(); }

const geom::ComplexPolygon& Map::getComplexPolygon() const { return map_; }

void Map::parseGeoJSON() {
    nlohmann::json geojson_features = nlohmann::json::parse(std::ifstream(path_))["features"];

    for (auto it = geojson_features.begin(); it != geojson_features.end(); it++) {
        auto polys_list = (*it)["geometry"]["coordinates"] \
            .get<std::vector<std::vector<std::pair<double, double>>>>();
        map_ = geom::ComplexPolygon(polys_list);
    }
}

}  // namespace truck::map