#pragma once

#include <geom/vector.h>
#include <geom/segment.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;

namespace truck::mesh {

struct MeshBuilderParams {
    std::string input_file;
    std::string output_file;
    std::string package_data_path;

    int img_size;
    int scale;

    struct Color {
        std::vector<int> background;
        std::vector<int> outer_polygon;
        std::vector<int> inner_polygon;
        std::vector<int> straight_skeleton;
    } color;
};

using Polygon = std::vector<geom::Vec2>;

class MeshBuilder {
  public:
    MeshBuilder(const MeshBuilderParams& params);

    const std::vector<Polygon>& getMapPolygons() const;
    const std::vector<geom::Segment>& getMapStraightSkeleton() const;

    void drawMap(bool poly=true, bool ss=true);

  private:
    void buildMap();

    void buildMapPolygons();
    void buildMapStraightSkeleton();

    void drawMapPolygons() const;
    void drawMapStraightSkeleton() const;

    struct Map {
        std::vector<Polygon> polygons;
        std::vector<geom::Segment> straight_skeleton;
    } map_;

    MeshBuilderParams params_;

    cv::Mat frame_;
};

}  // namespace truck::mesh