#pragma once

#include "geom/vector.h"
#include "geom/segment.h"
#include "geom/complex_polygon.h"

namespace truck::navigation::mesh {

struct MeshParams {
    double dist;
    double offset;

    struct Filter {
        bool grid;
    } filter;
};

struct MeshBuild {
    geom::Segments skeleton;
    geom::Segments level_lines;
    std::vector<geom::Vec2> mesh;
};

class MeshBuilder {
  public:
    MeshBuilder(const MeshParams& params);

    MeshBuild build(const geom::ComplexPolygons& polygons) const;

  private:
    static void buildSkeleton(MeshBuild& mesh_build, const geom::ComplexPolygon& polygon);
    static void buildLevelLines(
        MeshBuild& mesh_build, const geom::ComplexPolygon& polygon, double offset);
    void buildMesh(MeshBuild& mesh_build, double dist) const;

    void gridFilter() const;

    MeshParams params_;
};

}  // namespace truck::navigation::mesh
