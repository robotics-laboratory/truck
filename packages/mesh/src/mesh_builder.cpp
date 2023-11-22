#include "mesh/mesh_builder.h"

#include <boost/assert.hpp>
#include <boost/shared_ptr.hpp>
#include <nlohmann/json.hpp>

#include <fstream>

namespace truck::mesh {

namespace {

CGAL::Polygon_with_holes_2<CGAL_K> toCGALPolygon(const std::vector<std::vector<geom::Vec2>>& poly) {
    CGAL::Polygon_2<CGAL_K> cgal_poly_outer;

    for (const geom::Vec2& point : poly[0]) {
        CGAL_K::Point_2 cgal_point(point.x, point.y);
        cgal_poly_outer.push_back(cgal_point);
    }

    CGAL::Polygon_with_holes_2<CGAL_K> cgal_poly_with_holes = \
        CGAL::Polygon_with_holes_2<CGAL_K>(cgal_poly_outer);

    for (size_t i = 1; i < poly.size(); i++) {
        CGAL::Polygon_2<CGAL_K> cgal_poly_hole;
        
        for (const geom::Vec2& point : poly[i]) {
            CGAL_K::Point_2 cgal_point(point.x, point.y);
            cgal_poly_hole.push_back(cgal_point);
        }

        cgal_poly_with_holes.add_hole(cgal_poly_hole);
    }

    return cgal_poly_with_holes;
}

cv::Scalar toCVScalar(const std::vector<int> color) {
    BOOST_VERIFY(color.size() == 3);
    return cv::Scalar(color[2], color[1], color[0]);
}

}  // namespace

MeshBuilder::MeshBuilder(const MeshBuilderParams& params) : params_(params) {
    buildMap();
}

const std::vector<std::vector<geom::Vec2>>& MeshBuilder::getMapPoly() const { return map_.poly; }

const std::vector<geom::Segment>& MeshBuilder::getMapStraightSkeleton() const { return map_.straight_skeleton; }

const std::vector<geom::Vec2> MeshBuilder::getMapMesh() const { return map_.mesh; }

void MeshBuilder::buildMap() {
    buildMapPoly();
    buildMapStraightSkeleton();
    buildMapMesh();
}

void MeshBuilder::buildMapPoly() {
    std::ifstream file(params_.package_data_path + "/" + params_.input_file);
    nlohmann::json json_features = nlohmann::json::parse(file)["features"];

    for (nlohmann::json::iterator it = json_features.begin(); it != json_features.end(); it++) {
        auto polys = (*it)["geometry"]["coordinates"] \
            .get<std::vector<std::vector<std::pair<double, double>>>>();

        for (const auto& poly : polys) {
            std::vector<geom::Vec2> poly_vec2;

            for (const auto& point : poly) {
                BOOST_VERIFY(point.first >= 0 && point.second >= 0);

                poly_vec2.push_back(
                    geom::Vec2(point.first, point.second)
                );
            }

            map_.poly.push_back(poly_vec2);
        }
    }
}

void MeshBuilder::buildMapStraightSkeleton() {
    boost::shared_ptr<CGAL::Straight_skeleton_2<CGAL_K>> ss_ptr = \
        CGAL::create_interior_straight_skeleton_2(toCGALPolygon(map_.poly));
    
    CGAL::Straight_skeleton_2<CGAL_K>::Halfedge_const_iterator edge_it;

    for (edge_it = ss_ptr->halfedges_begin(); edge_it != ss_ptr->halfedges_end(); edge_it++) {
        CGAL_K::Point_2 start_point = edge_it->vertex()->point();
        CGAL_K::Point_2 end_point = edge_it->opposite()->vertex()->point();

        geom::Segment edge(
            geom::Vec2(start_point.x(), start_point.y()),
            geom::Vec2(end_point.x(), end_point.y())
        );

        map_.straight_skeleton.push_back(edge);
    }
}

void MeshBuilder::buildMapMesh() { /** @todo */ }

void MeshBuilder::drawMap(bool poly, bool ss, bool mesh) {
    frame_ = cv::Mat(
        params_.img_size_pixel,
        params_.img_size_pixel,
        CV_8UC3,
        toCVScalar(params_.color.background)
    );

    if (poly) { drawMapPoly(); }
    if (ss) { drawMapStraightSkeleton(); }
    if (mesh) { drawMapMesh(); }

    cv::imwrite(params_.package_data_path + "/" + params_.output_file, frame_);
}

void MeshBuilder::drawMapPoly() const {
    int poly_counter = 0;

    for (const auto& poly : map_.poly) {
        std::vector<cv::Point> cv_points;

        for (const auto& point : poly) {
            cv_points.push_back(
                cv::Point(
                    point.x * params_.meter_to_pixel_scale,
                    point.y * params_.meter_to_pixel_scale
                )
            );
        }

        if (poly_counter == 0) {
            cv::fillConvexPoly(frame_, cv_points, toCVScalar(params_.color.poly_outer));
        } else {
            cv::fillConvexPoly(frame_, cv_points, toCVScalar(params_.color.poly_hole));
        }

        cv::polylines(frame_, cv_points, 1, toCVScalar(params_.color.contour), params_.thickness);

        poly_counter++;   
    }
}

void MeshBuilder::drawMapStraightSkeleton() const {
    for (const geom::Segment& edge : map_.straight_skeleton) {
        cv::Point p1(
            edge.begin.x * params_.meter_to_pixel_scale,
            edge.begin.y * params_.meter_to_pixel_scale
        );

        cv::Point p2(
            edge.end.x * params_.meter_to_pixel_scale,
            edge.end.y * params_.meter_to_pixel_scale
        );

        cv::line(frame_, p1, p2, toCVScalar(params_.color.straight_skeleton), params_.thickness);
    }
}

void MeshBuilder::drawMapMesh() const { /** @todo */}

}  // namespace truck::mesh