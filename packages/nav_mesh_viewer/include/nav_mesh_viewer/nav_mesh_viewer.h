#pragma once

#include "map/map_builder.h"
#include "nav_mesh/nav_mesh_builder.h"

namespace truck::nav_mesh_viewer {

struct NavMeshViewerParams {
    double res;
    std::string path;

    struct Color {
        std::vector<int> background;
        std::vector<int> outer_polygon;
        std::vector<int> inner_polygon;
        std::vector<int> level_lines;
        std::vector<int> skeleton;
        std::vector<int> mesh;
    } color;

    struct Thickness {
        double level_lines;
        double skeleton;
        double mesh;
    } thickness;

    struct Enable {
        bool polygon;
        bool skeleton;
        bool level_lines;
        bool mesh;
    } enable;
};

class NavMeshViewer {
  public:
    NavMeshViewer(const NavMeshViewerParams& params);

    void draw(const geom::ComplexPolygons& polygons, const nav_mesh::NavMeshBuild& nav_mesh_build);

  private:
    NavMeshViewerParams params_;
};

}  // namespace truck::nav_mesh_viewer