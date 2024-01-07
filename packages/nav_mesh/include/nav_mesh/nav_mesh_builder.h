#pragma once

#include "map/map_builder.h"

#include "geom/segment.h"

namespace truck::nav_mesh {

struct NavMeshParams {
    double dist;
    double offset;

    struct Filter {
        bool grid;
    } filter;
};

struct NavMeshBuild {
    geom::Segments skeleton;
    geom::Segments level_lines;
    std::vector<geom::Vec2> mesh;
};

class NavMeshBuilder {
  public:
    NavMeshBuilder(const NavMeshParams& params);

    NavMeshBuild build(const geom::ComplexPolygons& polygons) const;

  private:
    void buildSkeleton(NavMeshBuild& build, const geom::ComplexPolygon& polygon) const;
    void buildLevelLines(NavMeshBuild& build, const geom::ComplexPolygon& polygon, double offset) const;
    void buildMesh(NavMeshBuild& build, double dist) const;

    void gridFilter() const;

    NavMeshParams params_;
};

}  // namespace truck::nav_mesh