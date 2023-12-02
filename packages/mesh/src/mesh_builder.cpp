#include "mesh/mesh_builder.h"

#include <boost/assert.hpp>
#include <boost/shared_ptr.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <unordered_map>

namespace truck::mesh {

namespace {

cv::Scalar toCVScalar(const std::vector<int> color) {
    BOOST_VERIFY(color.size() == 3);
    return cv::Scalar(color[2], color[1], color[0]);
}

CGAL::Polygon_with_holes_2<CGAL_K> toCGALPolygonWthHoles(const std::vector<Polygon>& polygon_with_holes) {
    CGAL::Polygon_2<CGAL_K> cgal_polygon_outer;

    for (const geom::Vec2& point : polygon_with_holes[0]) {
        cgal_polygon_outer.push_back(CGAL_K::Point_2(point.x, point.y));
    }

    CGAL::Polygon_with_holes_2<CGAL_K> cgal_polygon_with_holes = \
        CGAL::Polygon_with_holes_2<CGAL_K>(cgal_polygon_outer);

    for (size_t i = 1; i < polygon_with_holes.size(); i++) {
        CGAL::Polygon_2<CGAL_K> cgal_polygon_inner;
        
        for (const geom::Vec2& point : polygon_with_holes[i]) {
            cgal_polygon_inner.push_back(CGAL_K::Point_2(point.x, point.y));
        }

        cgal_polygon_with_holes.add_hole(cgal_polygon_inner);
    }

    return cgal_polygon_with_holes;
}

void drawPolygonFill(const Polygon& polygon, const cv::Mat& frame, double scale, const cv::Scalar& color) {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : polygon) {
        cv_points.push_back(cv::Point(point.x * scale, point.y * scale));
    }

    cv::fillConvexPoly(frame, cv_points, color);
}

}  // namespace

MeshBuilder::MeshBuilder(const MeshBuilderParams& params) : params_(params) { buildMap(); }

const std::vector<Polygon>& MeshBuilder::getMapPolygons() const { return map_.polygons; }

const std::vector<geom::Segment>& MeshBuilder::getMapStraightSkeleton() const { return map_.straight_skeleton; }

void MeshBuilder::buildMap() {
    buildMapPolygons();
    buildMapStraightSkeleton();
}

void MeshBuilder::buildMapPolygons() {
    std::ifstream file(params_.package_data_path + "/" + params_.input_file);
    nlohmann::json json_features = nlohmann::json::parse(file)["features"];

    for (nlohmann::json::iterator it = json_features.begin(); it != json_features.end(); it++) {
        auto polys_coordinates = (*it)["geometry"]["coordinates"] \
            .get<std::vector<std::vector<std::pair<double, double>>>>();

        for (const auto& poly_coordinates : polys_coordinates) {
            Polygon poly;

            for (const auto& coord : poly_coordinates) {
                BOOST_VERIFY(coord.first >= 0 && coord.second >= 0);
                poly.push_back(geom::Vec2(coord.first, coord.second));
            }

            map_.polygons.push_back(poly);
        }
    }
}

void MeshBuilder::buildMapStraightSkeleton() {
    boost::shared_ptr<CGAL::Straight_skeleton_2<CGAL_K>> cgal_ss_ptr = \
        CGAL::create_interior_straight_skeleton_2(toCGALPolygonWthHoles(map_.polygons));
    
    for (auto cgal_edge_it = cgal_ss_ptr->halfedges_begin();
         cgal_edge_it != cgal_ss_ptr->halfedges_end();
         cgal_edge_it++) {

        CGAL_K::Point_2 cgal_start_point = cgal_edge_it->vertex()->point();
        CGAL_K::Point_2 cgal_end_point = cgal_edge_it->opposite()->vertex()->point();

        geom::Segment edge(
            geom::Vec2(cgal_start_point.x(), cgal_start_point.y()),
            geom::Vec2(cgal_end_point.x(), cgal_end_point.y())
        );

        map_.straight_skeleton.push_back(edge);
    }
}

void MeshBuilder::drawMap(bool poly, bool ss) {
    frame_ = cv::Mat(
        params_.img_size,
        params_.img_size,
        CV_8UC3,
        toCVScalar(params_.color.background)
    );

    if (poly) { drawMapPolygons(); }
    if (ss) { drawMapStraightSkeleton(); }

    cv::imwrite(params_.package_data_path + "/" + params_.output_file, frame_);
}

void MeshBuilder::drawMapPolygons() const {
    int poly_counter = 0;

    for (const Polygon& poly : map_.polygons) {
        if (poly_counter == 0) {
            drawPolygonFill(poly, frame_, params_.scale, toCVScalar(params_.color.outer_polygon));
        } else {
            drawPolygonFill(poly, frame_, params_.scale, toCVScalar(params_.color.inner_polygon));
        }

        poly_counter++;
    }
}

void MeshBuilder::drawMapStraightSkeleton() const {
    for (const geom::Segment& edge : map_.straight_skeleton) {
        cv::Point p1 = cv::Point(edge.begin.x * params_.scale, edge.begin.y * params_.scale);
        cv::Point p2 = cv::Point(edge.end.x * params_.scale, edge.end.y * params_.scale);
        cv::line(frame_, p1, p2, toCVScalar(params_.color.straight_skeleton));
    }
}

}  // namespace truck::mesh