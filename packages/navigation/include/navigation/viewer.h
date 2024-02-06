#pragma once

#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"

namespace truck::navigation::viewer {

struct ViewerParams {
    double res = 50;
    std::string path = "";

    struct ColorRGB {
        std::vector<int> background = {0, 0, 0};
        std::vector<int> outer_polygon = {252, 252, 252};
        std::vector<int> inner_polygon = {227, 227, 227};
        std::vector<int> level_lines = {255, 0, 0};
        std::vector<int> skeleton = {0, 200, 0};
        std::vector<int> mesh = {0, 0, 255};
        std::vector<int> edges = {50, 50, 50};
        std::vector<int> route = {0, 200, 0};
    } color_rgb;

    struct Thickness {
        double level_lines = 1.0;
        double skeleton = 2.0;
        double mesh = 5.0;
        double edges = 1.0;
        double route = 20.0;
    } thickness;
};

class Viewer {
  public:
    Viewer();

    void draw(
        const ViewerParams& params,
        geom::ComplexPolygons polygons,
        std::optional<std::vector<geom::Vec2>> mesh = std::nullopt,
        std::optional<geom::Segments> skeleton = std::nullopt,
        std::optional<geom::Segments> level_lines = std::nullopt,
        std::optional<geom::Segments> edges = std::nullopt,
        std::optional<geom::Segments> route = std::nullopt);
};

}  // namespace truck::navigation::viewer