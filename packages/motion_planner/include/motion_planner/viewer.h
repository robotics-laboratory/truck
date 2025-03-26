#pragma once

#include "geom/polyline.h"
#include "geom/complex_polygon.h"
#include "motion_planner/graph_builder.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <optional>

namespace truck::motion_planner::viewer {

using rgb_t = std::vector<int>;

struct DrawerParam {
    rgb_t rgb;
    double thickness;
};

struct ViewerConfig {
    ViewerConfig(std::string_view path) : path{path} {}

    rgb_t background_color = {0, 0, 0};

    struct Polygon {
        rgb_t inner = {227, 227, 227};
        rgb_t outer = {252, 252, 252};
    };

    struct Hull {
        DrawerParam milestones = {.rgb = {255, 0, 0}, .thickness = 1.0};
        DrawerParam bounds = {.rgb = {100, 20, 20}, .thickness = 2.0};
        DrawerParam reference = {.rgb = {255, 165, 0}, .thickness = 1.5};
    };

    struct Graph {
        DrawerParam edges = {.rgb = {100, 100, 100}, .thickness = 1.0};
        DrawerParam nodes = {.rgb = {0, 0, 255}, .thickness = 3.0};
    };

    struct Motion {
        DrawerParam path = {.rgb = {0, 200, 0}, .thickness = 5.0};
        DrawerParam trajectory = {.rgb = {100, 100, 0}, .thickness = 7.0};
        DrawerParam reference = {.rgb = {200, 100, 100}, .thickness = 5.0};
    };

    Polygon polygon = {};
    Hull hull = {};
    Graph graph = {};
    Motion motion{};

    double res = 50;
    std::string path = "";
};

class CVDrawer {
  public:
    CVDrawer(const ViewerConfig& cfg, double res, const geom::ComplexPolygon& polygon);

    void drawPoint(const geom::Vec2& point, const DrawerParam& param);
    void drawPoints(const std::vector<geom::Vec2>& points, const DrawerParam& param);
    void drawSegment(const geom::Segment& seg, const DrawerParam& param);
    void drawPolyline(const geom::Polyline& polyline, const DrawerParam& param);
    void fillPoly(
        const geom::ComplexPolygon& polygon, const rgb_t& inner_rgb, const rgb_t& outer_rgb);

    void dump(const std::string& filename);

  private:
    cv::Scalar toCVScalar(const rgb_t& color_rgb) const;
    cv::Point toCVPoint(const geom::Vec2& point) const;
    cv::Point toCVPoint(const geom::Vec2& origin, const geom::Vec2& point) const;
    std::vector<cv::Point> toCVPoints(const std::vector<geom::Vec2>& points) const;
    std::vector<cv::Point> toCVPoints(
        const geom::Vec2& origin, const std::vector<geom::Vec2>& points) const;

  private:
    double res_;

    cv::Rect bbox_;
    geom::Vec2 origin_;
    cv::Mat frame_;
};

class Viewer {
  public:
    Viewer(const ViewerConfig& params, const geom::ComplexPolygon& polygon);

    void addPolygon(const geom::ComplexPolygon& polygon);
    void addHull(const hull::GraphBuild& graph_build);
    void addGraph(const hull::GraphBuild& graph_build);
    void addMotion(const hull::TrajectoryBuild& trajectory_build);

    void draw();

  private:
    ViewerConfig cfg_;
    CVDrawer drawer_;
};

}  // namespace truck::motion_planner::viewer
