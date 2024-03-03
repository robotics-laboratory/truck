#pragma once

#include "geom/polyline.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <optional>

namespace truck::navigation::viewer {

struct ViewerParams {
    double res = 50;
    std::string path = "";

    struct ColorRGB {
        std::vector<int> background = {0, 0, 0};
        
        struct Polygon {
            std::vector<int> inner = {227, 227, 227};
            std::vector<int> outer = {252, 252, 252};
        } polygon;

        struct MeshBuild {
            std::vector<int> level_lines = {255, 0, 0};
            std::vector<int> skeleton = {0, 200, 0};
            std::vector<int> mesh = {0, 0, 255};
        } mesh_build;

        struct Graph {
            std::vector<int> edges = {50, 50, 50};
            std::vector<int> nodes = {0, 0, 255};
        } graph;

        std::vector<int> path = {0, 200, 0};
    } color_rgb;

    struct Thickness {
        struct MeshBuild {
            double level_lines = 1.0;
            double skeleton = 2.0;
            double mesh = 5.0;
        } mesh_build;

        struct Graph {
            double edges = 1.0;
            double nodes = 5.0;
        } graph;

        double path = 20.0;
    } thickness;
};

class Viewer {
  public:
    Viewer(const ViewerParams& params, const geom::ComplexPolygon& polygon);

    void addPolygon(const geom::ComplexPolygon& polygon);
    void addMeshBuild(const mesh::MeshBuild& mesh_build);
    void addGraph(const graph::Graph& graph);
    void addPath(const geom::Polyline& path);

    void draw();

  private:
    cv::Mat frame_;
    cv::Rect bbox_;
    geom::Vec2 bbox_origin_;

    ViewerParams params_;
};

}  // namespace truck::navigation::viewer