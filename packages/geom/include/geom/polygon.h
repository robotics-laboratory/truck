#pragma once

#include "geom/vector.h"

#include <vector>

namespace truck::geom {

struct Polygon {
    Polygon() = default;

    Polygon(const std::vector<Vec2>& vertices) : vertices(vertices) {}

    Polygon(const std::vector<std::pair<double, double>>& vertices_list) {
        for (const auto& coord : vertices_list) {
            vertices.push_back(Vec2::fromPair(coord));
        }
    }

    std::vector<Vec2> vertices;
};

using Polygons = std::vector<Polygon>;

}  // namespace truck::geom