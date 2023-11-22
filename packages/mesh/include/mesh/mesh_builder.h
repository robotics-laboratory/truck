#pragma once

#include <geom/vector.h>
#include <geom/segment.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;

namespace truck::mesh {

struct MeshBuilderParams {
    std::string input_file;
    std::string output_file;
    std::string package_data_path;
    int img_size_pixel;
    int meter_to_pixel_scale;
    int thickness;

    struct Color {
        std::vector<int> background;
        std::vector<int> straight_skeleton;
        std::vector<int> poly_outer;
        std::vector<int> poly_hole;
        std::vector<int> contour;
    } color;
};

class MeshBuilder {
  public:
    MeshBuilder(const MeshBuilderParams& params);

    const std::vector<std::vector<geom::Vec2>>& getMapPoly() const;
    const std::vector<geom::Segment>& getMapStraightSkeleton() const;
    const std::vector<geom::Vec2> getMapMesh() const;

    void drawMap(bool poly=true, bool ss=true, bool mesh=true);

  private:
    void buildMap();
    void buildMapPoly();
    void buildMapStraightSkeleton();
    void buildMapMesh();

    void drawMapPoly() const;
    void drawMapStraightSkeleton() const;
    void drawMapMesh() const;

    struct Map {
        std::vector<std::vector<geom::Vec2>> poly;
        std::vector<geom::Segment> straight_skeleton;
        std::vector<geom::Vec2> mesh;
    } map_;

    MeshBuilderParams params_;

    cv::Mat frame_;
};

}  // namespace truck::mesh