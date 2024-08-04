#pragma once

#include "geom/vector.h"
#include "geom/segment.h"
#include "geom/polyline.h"
#include "geom/complex_polygon.h"

namespace truck::navigation::mesh {

struct MeshParams {
    double dist;
    double offset;

    struct RadialFilter {
        bool enabled;
        double search_radius;
    } radial_filter;
};

struct MeshBuild {
    geom::Segments skeleton;
    std::vector<geom::Polyline> level_lines;
    std::vector<geom::Vec2> mesh;
};

class MeshBuilder {
  public:
    MeshBuilder(const MeshParams& params);

    MeshBuild build(const geom::ComplexPolygons& polygons) const;

  private:
    void buildSkeleton(MeshBuild& mesh_build, const geom::ComplexPolygon& polygon) const;
    void buildLevelLines(
        MeshBuild& mesh_build, const geom::ComplexPolygon& polygon, double offset) const;
    void buildMesh(MeshBuild& mesh_build, double dist) const;

    void applyMeshFilter(MeshBuild& mesh_build, double search_radius) const;

    MeshParams params_;
};

}  // namespace truck::navigation::mesh
