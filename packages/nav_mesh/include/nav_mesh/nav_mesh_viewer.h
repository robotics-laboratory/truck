#pragma once

#include "nav_mesh/nav_mesh_builder.h"

namespace truck::nav_mesh::viewer {

struct NavMeshViewerParams {
    double res = 50;
    std::string path = "";

    struct ColorRGB {
        std::vector<int> background = {0, 0, 0};
        std::vector<int> outer_polygon = {252, 252, 252};
        std::vector<int> inner_polygon = {227, 227, 227};
        std::vector<int> level_lines = {255, 0, 0};
        std::vector<int> skeleton = {0, 200, 0};
        std::vector<int> mesh = {0, 0, 255};
    } color_rgb;

    struct Thickness {
        double level_lines = 1.0;
        double skeleton = 2.0;
        double mesh = 5.0;
    } thickness;

    struct Enable {
        bool polygon = false;
        bool skeleton = false;
        bool level_lines = false;
        bool mesh = false;
    } enable;
};

class NavMeshViewer {
  public:
    NavMeshViewer();

    void draw(
        const NavMeshViewerParams& params,
        const geom::ComplexPolygons& polygons, const builder::NavMeshBuild& nav_mesh_build);
};

}  // namespace truck::nav_mesh::viewer