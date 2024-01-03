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

struct NavMeshAttrs {
    geom::Segments skeleton;
    geom::Segments level_lines;
    std::vector<geom::Vec2> mesh;
};

class NavMeshBuilder {
  public:
    NavMeshBuilder(const NavMeshParams& params, const geom::ComplexPolygons& polygons);

    const NavMeshAttrs& attrs() const;

  private:
    void buildSkeleton(const geom::ComplexPolygon& polygon);
    void buildLevelLines(const geom::ComplexPolygon& polygon, double offset);
    void buildMesh(double dist);

    void gridFilter();

    NavMeshAttrs attrs_;
};

}  // namespace truck::nav_mesh